# LYDIA IKA -- ROS 2 Humble Simulasyon Container
# Kullanim:
#   docker build -t lydia_sim .
#   docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix lydia_sim
#
# Veya docker-compose ile:
#   docker-compose up

FROM osrf/ros:humble-desktop

# Temel araclar
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    wget \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 eklentileri (Nav2, SLAM Toolbox, Gazebo, ackermann, usb-cam)
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-smac-planner \
    ros-humble-dwb-core \
    ros-humble-dwb-plugins \
    ros-humble-nav2-navfn-planner \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ackermann-msgs \
    ros-humble-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Workspace hazirligi
RUN mkdir -p /root/ika_ws/src
WORKDIR /root/ika_ws

# Entrypoint: ROS 2 ortamini source et
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/ika_ws/install/setup.bash ]; then source /root/ika_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
