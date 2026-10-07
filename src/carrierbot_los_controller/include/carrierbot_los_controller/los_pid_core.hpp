// Lõi thuật toán LOS + PID cho robot vi sai.
// File này KHÔNG phụ thuộc ROS để có thể biên dịch và mô phỏng/kiểm thử độc lập.
// Plugin Nav2 (los_pid_controller.cpp) chỉ chuyển kiểu dữ liệu ROS <-> các struct ở đây.
//
// Quy ước: mọi tọa độ trong cùng một hệ (frame của đường đi, thường là "map"),
// góc tính bằng rad, vận tốc m/s và rad/s.

#ifndef CARRIERBOT_LOS_CONTROLLER__LOS_PID_CORE_HPP_
#define CARRIERBOT_LOS_CONTROLLER__LOS_PID_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace carrierbot_los_controller
{

inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// ---------------------------------------------------------------------------
// PID có lọc thông thấp cho khâu D và chống bão hòa tích phân (anti-windup)
// ---------------------------------------------------------------------------
struct PidGains
{
  double kp{1.5};
  double ki{0.0};
  double kd{0.1};
  double i_max{0.5};        // giới hạn |tích phân| (đơn vị của sai số * s)
  double out_max{1.5};      // giới hạn |đầu ra|
  double d_filter{0.7};     // 0 = không lọc, càng gần 1 càng lọc mạnh
};

class Pid
{
public:
  void setGains(const PidGains & g) {g_ = g;}
  const PidGains & gains() const {return g_;}

  void reset()
  {
    integral_ = 0.0;
    d_filtered_ = 0.0;
    has_prev_ = false;
  }

  // error: sai số (với góc thì đã wrap về [-pi, pi]); dt > 0
  double update(double error, double dt, bool error_is_angle = true)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    // Khâu D: đạo hàm sai số, có lọc thông thấp bậc 1
    double d_raw = 0.0;
    if (has_prev_) {
      const double de = error_is_angle ? wrapAngle(error - prev_error_) : (error - prev_error_);
      d_raw = de / dt;
    }
    d_filtered_ = g_.d_filter * d_filtered_ + (1.0 - g_.d_filter) * d_raw;
    prev_error_ = error;
    has_prev_ = true;

    // Khâu I: tích phân có kẹp biên
    const double integral_prev = integral_;
    integral_ = std::clamp(integral_ + error * dt, -g_.i_max, g_.i_max);

    double out = g_.kp * error + g_.ki * integral_ + g_.kd * d_filtered_;
    const double out_sat = std::clamp(out, -g_.out_max, g_.out_max);

    // Anti-windup: nếu đầu ra đang bão hòa và sai số đẩy tiếp theo cùng chiều
    // thì không tích phân thêm ở bước này
    if (out != out_sat && (error * out) > 0.0) {
      integral_ = integral_prev;
      out = g_.kp * error + g_.ki * integral_ + g_.kd * d_filtered_;
    }
    return std::clamp(out, -g_.out_max, g_.out_max);
  }

  double integral() const {return integral_;}

private:
  PidGains g_{};
  double integral_{0.0};
  double prev_error_{0.0};
  double d_filtered_{0.0};
  bool has_prev_{false};
};

// ---------------------------------------------------------------------------
// LOS dựa trên khoảng nhìn trước (lookahead-based LOS, Fossen),
// có tùy chọn thêm khâu tích phân (ILOS) để bù trôi ngang.
//
//   psi_d = alpha_k + atan( -(e + kappa * y_int) / Delta )
//   dy_int/dt = Delta * e / (Delta^2 + (e + kappa * y_int)^2)
//
//   alpha_k : góc tiếp tuyến của đường tại điểm chiếu gần nhất
//   e       : sai số ngang (cross-track error), dương khi robot lệch TRÁI đường
//   Delta   : khoảng nhìn trước = max(lookahead_min, lookahead_time * |v|),
//             kẹp trên bởi lookahead_max
// ---------------------------------------------------------------------------
struct LosParams
{
  double lookahead_min{0.4};
  double lookahead_max{1.2};
  double lookahead_time{1.0};   // s; 0 = Delta cố định = lookahead_min
  double integral_gain{0.0};    // kappa của ILOS; 0 = LOS thường
  double tangent_window{0.3};   // m; đoạn đường dùng để tính alpha_k (lọc nhiễu path dày)
  double search_window{2.0};    // m; chỉ tìm điểm gần nhất trong đoạn này phía trước
};

struct LosResult
{
  bool valid{false};
  double psi_d{0.0};            // góc hướng mong muốn
  double path_angle{0.0};       // alpha_k
  double cross_track{0.0};      // e
  double lookahead{0.0};        // Delta đã dùng
  double dist_to_goal{0.0};     // max(độ dài cung còn lại, khoảng cách thẳng tới đích)
  Point2 projection{};          // điểm chiếu của robot lên đường
  bool terminal{false};         // true: đang ở pha tiếp cận cuối (nhắm thẳng vào đích)
};

class LosGuidance
{
public:
  void setParams(const LosParams & p) {p_ = p;}
  const LosParams & params() const {return p_;}

  void setPath(const std::vector<Point2> & path)
  {
    path_ = path;
    s_.assign(path_.size(), 0.0);
    for (std::size_t i = 1; i < path_.size(); ++i) {
      s_[i] = s_[i - 1] + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    }
    seg_ = 0;
    need_global_search_ = true;
  }

  void resetIntegral() {y_int_ = 0.0;}
  bool hasPath() const {return path_.size() >= 2;}

  LosResult compute(double x, double y, double speed, double dt)
  {
    LosResult r;
    if (!hasPath()) {
      return r;
    }

    // 1) Tìm đoạn [k, k+1] gần robot nhất (chỉ đi tới, không lùi lại)
    findClosestSegment(x, y);

    const Point2 & a = path_[seg_];
    const Point2 & b = path_[seg_ + 1];
    const double seg_len = s_[seg_ + 1] - s_[seg_];
    double t = 0.0;
    if (seg_len > 1e-9) {
      t = ((x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y)) / (seg_len * seg_len);
    }
    // Ở đoạn cuối cho phép t > 1 để robot chạy tiếp theo hướng đoạn cuối
    const bool last_seg = (seg_ + 2 == path_.size());
    t = last_seg ? std::max(0.0, t) : std::clamp(t, 0.0, 1.0);
    const Point2 p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
    const double s_proj = s_[seg_] + t * seg_len;

    // 2) Góc tiếp tuyến alpha_k: từ điểm chiếu tới điểm cách nó tangent_window phía trước
    const double alpha = tangentAngle(p, s_proj);

    // 3) Sai số ngang trong hệ tọa độ gắn với đường
    const double e = -(x - p.x) * std::sin(alpha) + (y - p.y) * std::cos(alpha);

    // 4) Khoảng nhìn trước
    double delta = p_.lookahead_min;
    if (p_.lookahead_time > 0.0) {
      delta = std::clamp(p_.lookahead_time * std::fabs(speed), p_.lookahead_min, p_.lookahead_max);
    }
    delta = std::max(delta, 1e-3);

    const double arc_left = std::max(0.0, s_.back() - s_proj);
    const Point2 & goal = path_.back();
    const double d_goal = std::hypot(goal.x - x, goal.y - y);

    r.valid = true;
    r.path_angle = alpha;
    r.cross_track = e;
    r.lookahead = delta;
    r.projection = p;
    // max(...) để khi robot đã vượt qua cuối đường (arc_left = 0) vẫn còn khoảng cách thật
    r.dist_to_goal = std::max(arc_left, d_goal);

    if (arc_left < delta) {
      // 5a) Pha tiếp cận cuối: đường còn lại ngắn hơn Delta -> nhắm thẳng vào đích.
      // Nếu robot đã vượt qua đích (lệch ngang > dung sai) thì hướng này kéo nó quay lại.
      r.terminal = true;
      r.psi_d = (d_goal > 1e-3) ? std::atan2(goal.y - y, goal.x - x) : alpha;
      return r;
    }

    // 5b) ILOS (khi integral_gain = 0 thì đây là LOS thường)
    const double ee = e + p_.integral_gain * y_int_;
    if (p_.integral_gain > 0.0 && dt > 0.0) {
      y_int_ += dt * delta * e / (delta * delta + ee * ee);
    }
    r.psi_d = wrapAngle(alpha + std::atan2(-ee, delta));
    return r;
  }

private:
  void findClosestSegment(double x, double y)
  {
    const std::size_t n_seg = path_.size() - 1;
    std::size_t first = seg_;
    double s_limit = std::numeric_limits<double>::infinity();
    if (need_global_search_) {
      first = 0;
      need_global_search_ = false;
    } else {
      s_limit = s_[seg_] + p_.search_window;
    }

    double best = std::numeric_limits<double>::infinity();
    std::size_t best_k = first;
    for (std::size_t k = first; k < n_seg && s_[k] <= s_limit; ++k) {
      const double d = distToSegment(x, y, path_[k], path_[k + 1]);
      if (d < best) {
        best = d;
        best_k = k;
      }
    }
    seg_ = best_k;
  }

  static double distToSegment(double x, double y, const Point2 & a, const Point2 & b)
  {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (l2 > 1e-12) {
      t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / l2, 0.0, 1.0);
    }
    return std::hypot(x - (a.x + t * dx), y - (a.y + t * dy));
  }

  double tangentAngle(const Point2 & p, double s_proj) const
  {
    const double s_target = s_proj + p_.tangent_window;
    // Tìm điểm trên đường có độ dài cung >= s_target
    for (std::size_t i = seg_ + 1; i < path_.size(); ++i) {
      if (s_[i] >= s_target) {
        const double dx = path_[i].x - p.x;
        const double dy = path_[i].y - p.y;
        if (std::hypot(dx, dy) > 1e-6) {
          return std::atan2(dy, dx);
        }
        break;
      }
    }
    // Gần cuối đường: dùng hướng của đoạn cuối có độ dài khác 0
    for (std::size_t i = path_.size() - 1; i > 0; --i) {
      const double dx = path_[i].x - path_[i - 1].x;
      const double dy = path_[i].y - path_[i - 1].y;
      if (std::hypot(dx, dy) > 1e-6) {
        return std::atan2(dy, dx);
      }
    }
    return 0.0;
  }

  LosParams p_{};
  std::vector<Point2> path_;
  std::vector<double> s_;      // độ dài cung tích lũy tại mỗi điểm
  std::size_t seg_{0};
  bool need_global_search_{true};
  double y_int_{0.0};
};

// ---------------------------------------------------------------------------
// Ghép LOS (sinh psi_d) + PID góc hướng (sinh omega) + hồ sơ vận tốc dài
// ---------------------------------------------------------------------------
struct SpeedParams
{
  double desired_linear_vel{0.5};      // m/s
  double max_angular_vel{1.5};         // rad/s
  double max_linear_accel{0.8};        // m/s^2
  double max_linear_decel{1.0};        // m/s^2
  double max_angular_accel{2.0};       // rad/s^2
  bool use_rotate_in_place{true};      // xe vi sai: quay tại chỗ khi lệch hướng lớn
  double rotate_in_place_angle{0.9};   // rad; |sai số hướng| > giá trị này thì v = 0
  double rotate_in_place_exit_angle{0.3}; // rad; quay đến khi |sai số| < giá trị này
  double min_approach_vel{0.05};       // m/s; tốc độ tối thiểu khi tới gần đích
  double approach_dist{0.6};           // m; bắt đầu giảm tốc từ khoảng cách này
};

struct ControlOutput
{
  double v{0.0};
  double w{0.0};
  LosResult los{};
  double heading_error{0.0};
  bool rotating_in_place{false};
};

class LosPidCore
{
public:
  LosGuidance & los() {return los_;}
  Pid & pid() {return pid_;}
  void setSpeedParams(const SpeedParams & p) {sp_ = p;}
  const SpeedParams & speedParams() const {return sp_;}

  // Hệ số giới hạn tốc độ (0..1) từ Nav2 speed filter; 1 = không giới hạn
  void setSpeedLimitScale(double scale) {speed_limit_scale_ = std::clamp(scale, 0.0, 1.0);}

  void reset()
  {
    pid_.reset();
    los_.resetIntegral();
    v_prev_ = 0.0;
    w_prev_ = 0.0;
    rotating_ = false;
  }

  void setPath(const std::vector<Point2> & path) {los_.setPath(path);}

  // x, y, yaw: tư thế robot (cùng frame với path); v_meas: vận tốc dài đo được
  ControlOutput compute(double x, double y, double yaw, double v_meas, double dt)
  {
    ControlOutput out;
    out.los = los_.compute(x, y, std::max(std::fabs(v_meas), std::fabs(v_prev_)), dt);
    if (!out.los.valid || dt <= 0.0) {
      return out;
    }

    const double e_psi = wrapAngle(out.los.psi_d - yaw);
    out.heading_error = e_psi;

    // --- Vận tốc góc: PID trên sai số hướng ---
    double w = pid_.update(e_psi, dt, true);
    w = std::clamp(w, -sp_.max_angular_vel, sp_.max_angular_vel);

    // --- Vận tốc dài ---
    // Quay tại chỗ có trễ (hysteresis) để tránh rung khi sai số quanh ngưỡng
    if (sp_.use_rotate_in_place) {
      if (!rotating_ && std::fabs(e_psi) > sp_.rotate_in_place_angle) {
        rotating_ = true;
      } else if (rotating_ && std::fabs(e_psi) < sp_.rotate_in_place_exit_angle) {
        rotating_ = false;
      }
    } else {
      rotating_ = false;
    }

    double v_target = 0.0;
    if (!rotating_) {
      // Giảm tốc theo sai số hướng: lệch 90 độ thì v = 0
      v_target = sp_.desired_linear_vel * std::max(0.0, std::cos(e_psi));
      // Giảm tốc khi tới gần đích để dừng êm
      const double d = out.los.dist_to_goal;
      if (d < sp_.approach_dist && sp_.approach_dist > 0.0) {
        const double v_approach = std::max(sp_.min_approach_vel,
            sp_.desired_linear_vel * d / sp_.approach_dist);
        v_target = std::min(v_target, v_approach);
      }
      v_target *= speed_limit_scale_;
    }
    out.rotating_in_place = rotating_;

    // --- Giới hạn gia tốc ---
    const double dv_up = sp_.max_linear_accel * dt;
    const double dv_dn = sp_.max_linear_decel * dt;
    double v = std::clamp(v_target, v_prev_ - dv_dn, v_prev_ + dv_up);
    const double dw = sp_.max_angular_accel * dt;
    w = std::clamp(w, w_prev_ - dw, w_prev_ + dw);

    v_prev_ = v;
    w_prev_ = w;
    out.v = v;
    out.w = w;
    return out;
  }

private:
  LosGuidance los_;
  Pid pid_;
  SpeedParams sp_{};
  double speed_limit_scale_{1.0};
  double v_prev_{0.0};
  double w_prev_{0.0};
  bool rotating_{false};
};

}  // namespace carrierbot_los_controller

#endif  // CARRIERBOT_LOS_CONTROLLER__LOS_PID_CORE_HPP_
