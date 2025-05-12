FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

# ----------------- ENVIRONMENT -----------------
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    ROS_DISTRO=humble \
    TURTLEBOT3_MODEL=burger \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    ROS_DOMAIN_ID=30 \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

# ----------------- CORE PACKAGES -----------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales lsb-release gnupg curl build-essential \
    sudo git wget nano gedit xterm python3-pip \
    python3-rosinstall-generator python3-pyqt5 \
    dirmngr software-properties-common && \
    pip3 install -U rosdep && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# ----------------- SETUP ROS & GAZEBO REPOS -----------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    curl -sSL http://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# ----------------- INSTALL ROS2 & GAZEBO -----------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-turtlebot3* \
    gazebo libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# ----------------- CREATE ROS USER -----------------
RUN useradd -m -s /bin/bash ubuntu && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ubuntu
WORKDIR /home/ubuntu

# ----------------- CLONE PROJECT -----------------
RUN git clone --single-branch --branch swarmIRL https://github.com/aojedao/AdvancedMechatronics.git

# ----------------- SETUP DEV_WS -----------------
WORKDIR /home/ubuntu/AdvancedMechatronics/Final Project/ChoiRbot
RUN mkdir -p dev_ws/src && cp -r choirbot dev_ws/src/

# ----------------- INSTALL PYTHON & BUILD TOOLS -----------------
RUN pip3 install --upgrade pip && pip3 install -r dev_ws/src/choirbot/requirements.txt && pip3 install disropt colcon-common-extensions vcstool

# ----------------- BUILD CHOIRBOT -----------------
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && \
    cd /home/ubuntu/AdvancedMechatronics/Final Project/ChoiRbot/dev_ws && \
    rosdep init || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install"

# ----------------- INSTALL TURTLEBOT3 SIM -----------------
RUN mkdir -p /home/ubuntu/turtlebot3_ws/src && \
    cd /home/ubuntu/turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cd .. && \
    /bin/bash -c ". /opt/ros/humble/setup.bash && colcon build --symlink-install"

# ----------------- BASHRC SETUP -----------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/AdvancedMechatronics/Final Project/ChoiRbot/dev_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc && \
    echo "alias build_ws='cd ~/AdvancedMechatronics/Final Project/ChoiRbot/dev_ws && colcon build --symlink-install'" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
