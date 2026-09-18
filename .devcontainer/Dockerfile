FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install essentials + ROS repo
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install lightweight ROS, VNC, and tools (remove supervisor, heavy desktop)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    vim \
    nano \
    git \
    xfce4 \
    xfce4-terminal \
    tigervnc-standalone-server \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*
    xvfb x11vnc novnc websockify fluxbox mesa-utils libgl1-mesa-dri

# Init rosdep
RUN rosdep init && rosdep update

# Create the 'vscode' user (UID 1000) to match Codespaces
RUN useradd -m -u 1000 -s /bin/bash vscode && \
    echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ❗ Do NOT hardcode the workspace name here.
# We'll use a generic /workspaces folder and let post-create.sh find the repo.
WORKDIR /workspaces

# Set ownership to vscode (the folder will be mounted by Codespaces at runtime)
RUN chown -R vscode:vscode /workspaces

# Source ROS automatically (remove the install/setup.bash line – it doesn't exist yet)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc

# Copy VNC start script and make it executable
COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN chmod +x /usr/local/bin/start-vnc.sh

# Switch to vscode user
USER vscode
ENV DEBIAN_FRONTEND=dialog