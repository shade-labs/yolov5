FROM shaderobotics/ros:foxy-cudnn

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
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt

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

COPY yolov5_ws /app/shade_ws/src/yolov5

# build
WORKDIR /app/shade_ws
RUN colcon build

RUN echo '#!/bin/bash' >> run.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.sh' >> run.sh && \
    echo 'source ./install/setup.bash' >> run.sh && \
    echo 'ros2 run camera_simulator camera_simulator --type video --path /app/shade_ws/src/video.mp4 --loop &' >> run.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.sh' >> run.sh && \
    echo 'source ./install/setup.bash' >> run.sh && \
    echo 'ros2 run yolov5 interface --ros-args -r /yolov5/image_raw:=/image/image_raw &' >> run.sh
    chmod +x run.sh

CMD ["./run.sh"]