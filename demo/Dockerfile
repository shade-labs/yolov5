FROM nvidia/cuda:11.3.1-devel-ubuntu20.04

# avoids nvidia's expiring keys
RUN rm -f /etc/apt/sources.list.d/*.list

# ros2 install
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy

RUN apt update && \
    apt install -y --no-install-recommends \
      git \
      curl \
      python3-colcon-common-extensions \
      python3-pip  \
      ros-${ROS_DISTRO}-ros-core \
      python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# additional dependency requirements (cv2, cv_bridge, etc)
RUN apt update && \
    apt install -y --no-install-recommends \
      ffmpeg \
      libsm6 \
      libxext6 \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-vision-opencv && \
    rm -rf /var/lib/apt/lists/*

# yolov5 install
WORKDIR /app
RUN python3 -m pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt && \
    python3 -m pip install pandas==1.1.4 seaborn==0.11.0

# This is in a different folder, just to show running the workspace
RUN ["/bin/bash", "-c", "mkdir -p /app/shade_ws/src"]
WORKDIR /app/shade_ws/src

# testing functionality
RUN git clone https://github.com/klintan/ros2_video_streamer && \
    curl --output ./video.mp4 https://www.sample-videos.com/video123/mp4/360/big_buck_bunny_360p_10mb.mp4

RUN apt update && \
    apt install -y --no-install-recommends \
      python3-natsort && \
    rm -rf /var/lib/apt/lists/*

COPY ../yolov5_ros2 /app/shade_ws/src/yolov5_ros2

# build
WORKDIR /app/shade_ws
RUN colcon build

RUN echo '#!/bin/bash' >> run.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.sh' >> run.sh && \
    echo 'source ./install/setup.bash' >> run.sh && \
    echo 'ros2 run camera_simulator camera_simulator --type video --path /app/shade_ws/src/video.mp4 --loop &' >> run.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.sh' >> run.sh && \
    echo 'source ./install/setup.bash' >> run.sh && \
    echo 'ros2 run yolov5_ros2 interface --ros-args -r /yolov5/image_raw:=/image/image_raw &' >> run.sh && \
    echo 'bash' >> run.sh && \
    chmod +x run.sh

CMD ["./run.sh"]