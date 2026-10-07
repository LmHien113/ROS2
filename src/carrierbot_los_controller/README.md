# carrierbot_los_controller

Plugin controller cho Nav2, thay cho Regulated Pure Pursuit ở `FollowPath`:
**LOS** (lookahead-based, tùy chọn ILOS) sinh góc hướng mong muốn `psi_d`,
**PID** trên sai số hướng `psi_d - psi` sinh vận tốc góc `omega`.
Đầu ra là `/cmd_vel` → `diff_drive_controller` → `CarrierbotInterface` → CAN.

## Cấu trúc
| File | Vai trò |
|---|---|
| `include/.../los_pid_core.hpp` | Toàn bộ thuật toán (LOS, PID, hồ sơ vận tốc). Không phụ thuộc ROS. **Sửa công thức của bạn ở đây.** |
| `include/.../los_pid_controller.hpp`, `src/los_pid_controller.cpp` | Vỏ plugin `nav2_core::Controller`: đổi frame, đọc tham số, publish debug |
| `los_pid_controller_plugin.xml` | Khai báo plugin cho pluginlib |

Hỗ trợ cả **Foxy** (robot thật) và **Humble** (mô phỏng). CMake tự nhận `ROS_DISTRO`.

## Build và chạy
```bash
cd ~/ros2            # workspace
colcon build --packages-select carrierbot_los_controller carrierbot_navigation
source install/setup.bash
ros2 launch carrierbot_bringup real_robot.launch.py   # như cũ
```
`carrierbot_navigation/config/controller_server.yaml` đã trỏ `FollowPath` sang
`carrierbot_los_controller::LosPidController`. Khối RPP cũ được giữ dạng comment để quay lại.

## Thuật toán
```
alpha  = góc tiếp tuyến của đường tại điểm chiếu gần nhất
e      = sai số ngang (dương: robot lệch TRÁI đường)
Delta  = clamp(lookahead_time * |v|, lookahead_min, lookahead_max)
psi_d  = alpha + atan(-(e + kappa*y_int) / Delta)        (kappa = los_integral_gain)
omega  = PID(wrap(psi_d - psi))
v      = desired_linear_vel * max(0, cos(e_psi)), giảm dần khi còn cách đích < approach_dist
```
- Lệch hướng > `rotate_in_place_angle` → quay tại chỗ (v = 0) cho tới khi < `rotate_in_place_exit_angle`.
- Khi đoạn đường còn lại < Delta → nhắm thẳng vào điểm đích (kéo robot quay lại nếu đã vượt quá).
- PID: lọc thông thấp khâu D, kẹp tích phân và chống bão hòa (anti-windup).
- PID/ILOS chỉ reset khi **đích** đổi, không reset mỗi lần Nav2 replan.

## Chỉnh hệ số khi đang chạy
```bash
ros2 param set /controller_server FollowPath.kp 2.0
ros2 param set /controller_server FollowPath.lookahead_min 0.6
ros2 topic echo /FollowPath/los_debug
```
`/FollowPath/los_debug` (Float64MultiArray):
`[0] e_cross  [1] psi_d  [2] psi  [3] e_psi  [4] v_cmd  [5] w_cmd  [6] dist_to_goal  [7] Delta  [8] quay_tai_cho  [9] dt  [10] pha_cuoi`

`/FollowPath/los_lookahead_point` (PointStamped): điểm LOS đang nhắm, xem trong RViz.

Thứ tự chỉnh gợi ý: `ki = 0, kd = 0` → tăng `kp` tới khi hướng bám nhanh mà chưa dao động →
thêm `kd` nếu vọt lố → chỉnh `lookahead_min` (nhỏ = bám sát hơn nhưng dễ lắc) →
chỉ dùng `los_integral_gain` (0.2–0.5) nếu còn lệch ngang ổn định do trượt bánh/lệch bánh.

## Chưa có
- Kiểm tra va chạm theo costmap (RPP có `max_allowed_time_to_collision`). Local costmap hiện chỉ có `static_layer` nên RPP cũng chỉ né được bản đồ tĩnh.
- Chạy lùi.
