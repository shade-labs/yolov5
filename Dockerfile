ARG CUDA_VERSION
FROM nvidia/cuda:${CUDA_VERSION}

# set enviromental variables
ARG ROS_DISTRO
ENV ROS_DISTRO ${ROS_DISTRO}

ARG UBUNTU_VERSION
ENV UBUNTU_VERSION ${UBUNTU_VERSION}

# avoids nvidia's expiring keys
RUN rm -f /etc/apt/sources.list.d/*.list

# ensure timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN echo "deb http://packages.ros.org/ros2/ubuntu ${UBUNTU_VERSION} main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt update && \
    apt install -y \
      git \
      curl \
      python3-colcon-common-extensions \
      python3-pip  \
      ros-${ROS_DISTRO}-ros-core \
      python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# additional dependency requirements (cv2, cv_bridge, etc)
RUN apt update && \
    apt install -y \
      ffmpeg \
      libsm6 \
      libxext6 \
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
WORKDIR /app/shade_ws/src
COPY yolov5_ros2 /app/shade_ws/src/yolov5_ros2

# build
WORKDIR /app/shade_ws
RUN colcon build
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source ./install/setup.bash && ros2 run yolov5_ros2 interface"]