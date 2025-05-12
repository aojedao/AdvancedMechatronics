# Dockerfile for ChoiRbot from https://github.com/aojedao/AdvancedMechatronics/tree/swarmIRL/Final%20Project/ChoiRbot

FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install core dependencies

#RUN apt-get update && apt-get install -y --no-install-recommends \
#    lsb-release gnupg curl build-essential \
#    locales sudo git wget nano xterm python3-pip \
#    python3-colcon-common-extensions python3-vcstool \
#    python3-rosdep python3-rosinstall-generator python3-pyqt5 \
#    dirmngr software-properties-common && \
#    rm -rf /var/lib/apt/lists/*

# 1. Install core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release gnupg curl build-essential \
    locales sudo git wget nano gedit xterm python3-pip \
    python3-rosdep python3-rosinstall-generator python3-pyqt5 \
    dirmngr software-properties-common && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install -U colcon-common-extensions vcstool

# 2. Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# 3. Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble

# 4. Initialize rosdep
RUN rosdep init || true && rosdep update

# 5. Create user and workspaces
RUN useradd -m -s /bin/bash ubuntu && \
    echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ubuntu
WORKDIR /home/ubuntu

# 6. Clone ChoiRbot repo
RUN git clone --single-branch --branch swarmIRL https://github.com/aojedao/AdvancedMechatronics.git
WORKDIR /home/ubuntu/AdvancedMechatronics/Final\ Project/ChoiRbot

# 7. Create dev_ws inside the ChoiRbot path
RUN mkdir -p dev_ws/src
WORKDIR /home/ubuntu/AdvancedMechatronics/Final\ Project/ChoiRbot/dev_ws

# 8. Copy ChoiRbot into workspace
RUN cp -r ../choirbot . && mv choirbot src/

# 9. Install Python dependencies
RUN pip3 install --upgrade pip && pip3 install -r src/choirbot/requirements.txt

# 10. Build with colcon
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install"

# 11. Install turtlebot3_simulations
RUN mkdir -p ~/turtlebot3_ws/src && \
    cd ~/turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cd ~/turtlebot3_ws && \
    /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# 12. Update .bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source ~/AdvancedMechatronics/Final\ Project/ChoiRbot/dev_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]

