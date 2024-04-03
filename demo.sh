ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P1500 #1 P1300 #2 P500 #3 P1500 #5 P1500 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P2000 #1 P2200 #2 P1400 #3 P1050 #4 P1100 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P1500 #1 P1700 #2 P1400#3 P1500 #4 P1100 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#4 P500 T500'}"

sleep 1.0

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P1500 #1 P2050 #2 P1700 #3 P950 #4 P1100 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#1 P1300 #2 P1850 #3 P1500 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#5 P1700 T100'}"

sleep 0.3

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#5 P1300 T200'}"

sleep 0.3

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#5 P1700 T200'}"

sleep 0.3

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#5 P1500 T100'}"

sleep 0.3

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#4 P500 T500'}"