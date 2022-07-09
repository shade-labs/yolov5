ARG ROS_DISTRO=humble
FROM shaderobotics/pytorch:${ROS_DISTRO}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO

WORKDIR /home/shade/shade_ws

# additional dependency requirements (cv2, cv_bridge, etc)
RUN apt update && \
    apt install -y \
      ffmpeg \
      libsm6 \
      libxext6 \
      python3 \
      python3-pip \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-vision-opencv && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt

# inject and build wrapper
COPY . ./src/yolov5

RUN colcon build && \
    echo "#!/bin/bash" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/shade/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source ./install/setup.sh" >> ./start.sh && \
    echo "ros2 run yolov5 interface" >> /home/shade/shade_ws/start.sh && \
    chmod +x /home/shade/shade_ws/start.sh

ENTRYPOINT ["/home/shade/shade_ws/start.sh"]