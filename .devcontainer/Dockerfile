FROM ubuntu:noble

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

RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository universe

RUN sudo apt-get update && sudo apt-get install -y curl
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get update && \
    sudo apt-get install -y ros-dev-tools

RUN sudo apt-get update && \
    sudo apt-get upgrade -y

RUN sudo apt-get install -y ros-jazzy-desktop

# Source the ROS setup file
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Rosdep update
RUN sudo rosdep init
RUN rosdep update

RUN sudo apt-get install -y ros-jazzy-joint-state-publisher-gui
RUN sudo apt-get install -y ros-jazzy-ros-gz*
RUN sudo apt-get install -y ros-jazzy-ros2-control
RUN sudo apt-get install -y ros-jazzy-ros2-controllers
RUN sudo apt-get install -y ros-jazzy-gz-ros2-control*
RUN sudo apt-get install -y ros-jazzy-moveit*
RUN sudo apt-get install -y libserial-dev
RUN sudo apt-get install -y python3-pip

RUN sudo apt-get install -y tree

COPY entrypoint.sh /entrypoint.sh
