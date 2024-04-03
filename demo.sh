ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P2000 #1 P2100 #2 P1500 #3 P1100 #4 P1100 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P1500 #1 P1300 #2 P1850 #3 P1500 #4 P1100 T2000'}"

sleep 2.5

ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#4 P500 T1000'}"