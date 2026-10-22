# Nguồn mô hình và API

- `ur_description`: package hệ thống từ Universal Robots, version đã kiểm tra
  `4.3.1`, nhánh upstream cho Lyrical là `rolling`.
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling
- `src/robotiq_description`: bản sao nguyên package mô tả từ
  https://github.com/PickNikRobotics/ros2_robotiq_gripper,
  commit `a74d007d8f2f06dc6a503ad21038ba869d4999a6`.
  Giữ nguyên `LICENSE` BSD-3-Clause. Chỉnh CMake để cài kèm LICENSE vào thư mục
  share của package; các file mô tả và mesh giữ nguyên upstream.
  Không tải driver phần cứng vào workspace.
  Gripper Xacro upstream sử dụng `include_ros2_control:=false` khi ghép vào robot.
- Môi trường bàn, khay chia ô và vật thể được tạo riêng cho project bằng các
  primitive box. Không phụ thuộc Gazebo Fuel hoặc tải mesh môi trường khi chạy.
- MoveIt 2 Lyrical / Ubuntu 26.04:
  https://moveit.ai/install-moveit2/binary/
- ROS 2 Lyrical / Gazebo Jetty:
  https://gazebosim.org/docs/jetty/ros_installation/
- `gz_ros2_control` và mimic joint:
  https://control.ros.org/lyrical/doc/gz_ros2_control/doc/index.html
- Gazebo fixed constraints dùng component `DetachableJoint`:
  https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1components_1_1DetachableJoint.html

Phần ứng dụng dùng `rclpy`, các action/service trong `moveit_msgs` và Python
bindings `gz.transport`/`gz.msgs` có sẵn trên máy. Package `tray_workcell` dùng
`ament_python`, không biên dịch executable hay plugin C++ tự viết. Package
`robotiq_description` vẫn dùng CMake của upstream để cài tài nguyên mô tả.

Grasp assistance gọi hệ thống DetachableJoint được cung cấp sẵn bởi Gazebo.
Không giả định tuỳ chọn `initial_attach` trên nhánh upstream mới nhất tồn tại
trong Jetty đang cài: Python quản lý bước detach có xác nhận khi khởi động.
