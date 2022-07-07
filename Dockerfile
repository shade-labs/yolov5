ARG CUDA_VERSION=foxy-cudnn-runtime
FROM shaderobotics/ros:${CUDA_VERSION}

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
    rm -rf /var/lib/apt/lists/*

# yolov5 install
WORKDIR /app
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt --ignore-installed PyYAML

# inject wrapper
RUN ["/bin/bash", "-c", "mkdir -p /app/shade_ws/src"]
COPY yolov5_ws /app/shade_ws/src/yolov5

# build
WORKDIR /app/shade_ws
RUN colcon build
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source ./install/setup.bash && ros2 run yolov5 interface"]