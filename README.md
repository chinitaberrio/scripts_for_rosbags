# Scripts to modify rosbags content

I maintain a repository containing scripts to edit particular topics within rosbags. While I aim to make it accessible to everyone, I cannot guarantee its availability due to time constraints.

## odom reset

I encountered a problem with a rosbag where the odometry frame is established at the moment the robot is powered on. I captured the rosbag after moving the robot, causing subsequent odometry topics (visual, lidar, etc.) to originate from the starting point of the rosbag. To rectify this issue, I created a script that resets the initial odometry message to correspond with the origin of the rosbag when it commences, rather than when the robot powers on. This simplifies the process of comparing distinct odometry approaches.