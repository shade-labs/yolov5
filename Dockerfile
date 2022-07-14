ARG ROS_DISTRO=humble
FROM shaderobotics/pytorch:${ROS_DISTRO}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO

SHELL ["/bin/bash", "-c"]
WORKDIR /home/shade/shade_ws
COPY . ./src/yolov5

RUN apt update && \
    apt install -y \
      ffmpeg \
      libsm6 \
      libxext6 \
      python3-pip && \
    : "Install dependencies and build" && \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install ./src/yolov5 && \
    rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/${ROS_DISTRO}/setup.sh && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build && \
    : "Inject start script and shade cloud support" && \
    echo "#!/bin/bash" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/shade/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source ./install/setup.sh" >> ./start.sh && \
    echo "ros2 run yolov5 interface" >> /home/shade/shade_ws/start.sh && \
    chmod +x /home/shade/shade_ws/start.sh

ENTRYPOINT ["/home/shade/shade_ws/start.sh"]