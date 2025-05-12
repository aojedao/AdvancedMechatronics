# Base image with GPU/OpenGL support
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

# Environment variables
ENV ROS_DISTRO=humble \
    TURTLEBOT3_MODEL=burger \
    DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    ROS_DOMAIN_ID=30 \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

# 1. Add ROS 2 and Gazebo repositories (modern method)
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    # ROS 2 repository
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    # Gazebo repository (modern approach)
    curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor > /usr/share/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 2. Install tools and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget xterm python3-pip \
    python3-colcon-common-extensions python3-rosdep2 \
    build-essential nano gedit \
    ros-humble-desktop \
    ros-humble-turtlebot3* \
    gazebo libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. Initialize rosdep
RUN rosdep init || true && rosdep update

# 4. Create user with passwordless sudo
RUN groupadd -g 1000 ubuntu && \
    useradd -m -s /bin/bash -u 1000 -g 1000 ubuntu && \
    usermod -aG sudo ubuntu && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "Defaults !requiretty" >> /etc/sudoers

USER ubuntu
WORKDIR /home/ubuntu

# 5. Clone project and set up workspaces
RUN git clone --single-branch --branch swarmIRL https://github.com/aojedao/AdvancedMechatronics.git && \
    mkdir -p ~/AdvancedMechatronics/Final\ Project/ChoiRbot/dev_ws/src && \
    cp -r ~/AdvancedMechatronics/Final\ Project/ChoiRbot/choirbot ~/AdvancedMechatronics/Final\ Project/ChoiRbot/dev_ws/src/

# 6. Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install -r ~/AdvancedMechatronics/Final\ Project/ChoiRbot/dev_ws/src/choirbot/requirements.txt && \
    pip3 install disropt vcstool

# 7. Build turtlebot3_ws
RUN mkdir -p ~/turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git ~/turtlebot3_ws/src/turtlebot3_simulations && \
    bash -c "source /opt/ros/humble/setup.bash && \
    cd ~/turtlebot3_ws && \
    colcon build --symlink-install"

# 8. Configure environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc && \
    echo "alias build_ws='cd ~/dev_ws && colcon build --symlink-install'" >> ~/.bashrc

WORKDIR /home/ubuntu/dev_ws
ENTRYPOINT ["/bin/bash"]
