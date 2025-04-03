FROM osrf/ros:jazzy-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt upgrade -y

# Install Git
RUN sudo apt-get install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN sudo apt-get install -y tree
RUN sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz*
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros2-control
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros2-controllers
RUN sudo apt-get install -y ros-${ROS_DISTRO}-gz-ros2-control*
RUN sudo apt-get install -y ros-${ROS_DISTRO}-moveit*
RUN sudo apt-get install -y libserial-dev
RUN sudo apt-get install -y python3-pip



RUN mkdir -p ~/workspace