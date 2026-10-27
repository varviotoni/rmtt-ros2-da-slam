FROM nvidia/cuda:12.1.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release build-essential git python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Humble repository setup
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble & SLAM dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ffmpeg libavcodec-dev libavformat-dev libswscale-dev

# Install Python dependencies for Depth Anything V2 & Tello UDP video decoding
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cu121 && \
    pip3 install --no-cache-dir \
    transformers accelerate opencv-python "numpy<2.0" av
RUN pip3 install --no-cache-dir --upgrade \
    "pillow>=10.0.0" "transformers>=4.41.2,<5.0.0" "accelerate<1.2.0" "numpy<2.0"

WORKDIR /root/ros2_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
