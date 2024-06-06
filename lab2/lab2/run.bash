ros2 launch depthai_ros_driver camera.launch.py &
ros2 launch lab2 bringup.launch.py &
ros2 run lab2 saveImg &
ros2 run lab2 gui_service &
ros2 run lab2 gui_client_ssh &