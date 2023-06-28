# Scripts for Rosbag Manipulation (TfNSW dataset)

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
### img extraction
The script extracts individual images from mp4 video files and save them to a designated folder. The generated image names follow a specific convention that includes the timestamp of each image. Additionally, the script offers the functionality of applying rectification to the images based on the corresponding camera model. Furthermore, users have the option to specify a divider, allowing them to extract images at regular intervals rather than consecutive frames. 

```python img_extraction.py -b='path/to_the/rosbag.bag' -v='path/to_the/video.mp4' -s=path/to_the/img_folder/```
with flags ```-r=True``` is to rectify image ```-d=30``` will save image every 30 frames

### img write rosbag
The dataset comprises both a rosbag file and separate video files for each camera. There may be instances where it is necessary to have the images included within the rosbag itself. To address this requirement, the script creates a new rosbag that contains the image_color topic, which corresponds to the provided video files. This script facilitates the generation of a consolidated rosbag dataset with synchronized image data, enabling seamless integration and compatibility for certain applications.   

```python img_write_rosbag.py -b='path/to_the/rosbag.bag' -v='path/to_the/video.mp4' -o=path/to_the/bag_folder/```

### video rectify

The script uses the camera_info data from a rosbag file and its corresponding video file to generate a new video file where the images are rectified. By extracting the camera_info, the script applies rectification to the images, resulting in a video with rectified frames. 

```python video_rectify.py -b='path/to_the/rosbag.bag' -v='path/to_the/video.mp4' -o=path/to_the/video.avi```
