# Dance_Detection
Final project for Computational Robotics F'23

To connect to the Neatos, use the following commands and adjust the IP address:
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.89 robot_name:=head udp_video_port:=5003 udp_sensor_port:=7778

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.90 robot_name:=left_a udp_video_port:=5004 udp_sensor_port:=7779

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.64 robot_name:=right_a udp_video_port:=5005 udp_sensor_port:=7777

ros2 launch neato_node2 bringup_multi.py host:=192.168.17.209 robot_name:=left_l udp_video_port:=5006 udp_sensor_port:=7776

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.64 robot_name:=right_l udp_video_port:=5007 udp_sensor_port:=7775