# set enviromental variables
#ARG ROS_DISTRO
#ENV ROS_DISTRO ${ROS_DISTRO}
#
#ARG CUDA_VERSIONS
#FROM nvidia/cuda:${CUDA_VERSION}

FROM nvidia/cuda:11.3.1-devel-ubuntu20.04
ENV ROS_DISTRO foxy

# avoids nvidia's expiring keys
RUN rm -f /etc/apt/sources.list.d/*.list

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

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

# inject wrapper
RUN ["/bin/bash", "-c", "mkdir -p /app/shade_ws/src"]
WORKDIR /app/shade_ws/src
COPY yolov5_ros2 /app/shade_ws/src/yolov5_ros2

# build
WORKDIR /app/shade_ws
RUN colcon build

#CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.sh", "source ./install/setup.sh"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && source ./install/setup.bash && ros2 run yolov5_ros2 interface"]