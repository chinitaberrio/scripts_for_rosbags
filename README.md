# Scripts to modify rosbags content

Scripts to edit and check particular topics within rosbags. 

### odom reset

In rosbags the odometry frame was established at the moment the robot was powered on. Nevertheless odometry topics generated in postprocessing stages originate from the starting point of the rosbag. To address this problem, we developed a script that resets the initial odometry message to align with the origin of the rosbag at the beginning of its execution, rather than when the robot powers on. This solution simplifies the task of comparing different odometry methods by ensuring a consistent reference point.
```
python odom_reset.py -b 'path/to_the/rosbag.bag' -odom 'odometry/topic' 
```
### rosbag health check
The script performs various checks on a rosbag file, including synchronization verification, frequency mismatch detection, and topic requirements assessment. It requires both the rosbag file and a configuration file containing information about the topics, frequency checking requirements, and the desired rates. The script generates a report that highlights topics that are out of sync and provides their corresponding offsets. It also identifies topics with frequency mismatches, indicating the actual frequency observed and the expected frequency. Furthermore, the report specifies if any of the required topics were not found within the rosbag.   

```
python odom_reset.py -b 'path/to_the/rosbag.bag' -config 'path/to_the/config.yaml' 
```


