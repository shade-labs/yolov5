# YOLOv5-ROS2 Wrapper

This is a ROS2 wrapper for the image detection package, [YOLOv5](https://github.com/ultralytics/yolov5). The main idea is for this container to act as a standalone interface and node, removing the necessity to integrate separate packages and solve numerous dependency issues.

# Installation Guide
## Using Docker Pull
1. Install [Docker](https://www.docker.com/) and ensure the Docker daemon is running in the background.
2. Run ```docker pull shade/yolov5-${ROS2_DISTRO}```
3. Follow the run commands in the usage section below

## Build Docker Image Natively
1. Install [Docker](https://www.docker.com/) and ensure the Docker daemon is running in the background.
2. Clone this repo with ```git pull -b ${ROS2_DISTRO} https://github.com/open-shade/yolov5_ros2.git```
3. Enter the repo with ```cd yolov5_ros2```
4. Build the container with ```docker build . -t [name]```. This will take a while.
5. Follow the run commands in the usage section below.

# Usage
## Communicate to local machine 
Run ```docker run -it --rm [name] ros2 run yolov5_ros2 interface```. Then, by running ```ros2 topic list,``` you should see all the possible pub and sub routes.

For more details explaining how ROS2 communicates between external environment or multiple docker containers, visit the official ROS2 docs [here](https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html#). 

## Run everything within the container
This method only works if you build this wrapper natively. Specifically, after you clone the repo, enter the Dockerfile and uncomment the last line in the file. Then, rebuild the container and run ```docker run -t [name]```. The container should then have the yolov5 node running within.

# Parameters
This wrapper utilizes 4 optional parameters to modify the data coming out of the published topics as well as the dataset YOLOv5 utilizes for comparison. Most parameters can be modified during runtime. However, if you wish to use your own dataset, you must pass that parameter in before runtime. If you are unsure how to pass or update parameters before or during runtime, visit the official ROS2 docs [here](https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Parameters.html?highlight=parameters#setting-initial-parameter-values-when-running-a-node).

The supported, *optional* parameters are...

| Name        | Type    | Default | Use                                                                 |
|-------------|---------|---------|---------------------------------------------------------------------|
| pub_image   | Boolean | False   | Enable or disable the pub of the processed image                    |
| pub_json    | Boolean | False   | Enable or disable the pub of the data in stringified json           |
| pub_boxes   | Boolean | True    | Enable or disable the pub of vision_msgs.msgs DetectionBoxesArray   | 
| dataset_url | String  | [yolov5m](https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5m.pt) | Download URL for your YOLOv5 training set. Has to be in .pt format. |   

You __do not__ need to specify any parameters, unless you wish to modify the defaults.

# Topics

| Name                   | IO  | Type                             | Use                                                               |
|------------------------|-----|----------------------------------|-------------------------------------------------------------------|
| yolov5/image_raw       | sub | [sensor_msgs.msg.Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)            | Takes the raw camera output to be processed                       |
 | yolov5/image           | pub | [sensor_msgs.msg.Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)            | Outputs the processed image with bounding boxes drawn on the image |
| yolov5/json            | pub | [std_msgs.msg.String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)              | Outputs the detected objects in a frame in stringified json format |
| yolov5/detection_boxes | pub | [vision_msgs.msg.Detection2DArray](http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection2DArray.html) | Outputs the detected bounding box location in a unified format    |

# Testing / Demo
To test and ensure that this package is properly installed, replace the Dockerfile in the root of this repo with what exists in the demo folder. Installed in the demo image contains a [camera stream emulator](https://github.com/klintan/ros2_video_streamer) by [klintan](https://github.com/klintan) which directly pubs images to the yolov5 node and processes it for you to observe the outputs.

To run this, run ```docker build . -t [name]```, then ```docker run -t [name]```. Observing the logs for this will show you what is occuring within the container. If you wish to enter the running container and preform other activities, run ```docker ps```, find the id of the running container, then run ```docker exec -it [containerId] /bin/bash```