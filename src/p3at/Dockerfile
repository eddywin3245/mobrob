FROM ros:jazzy-ros-base

# Install required ROS2 packages and tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-sick-scan-xd \
    python3-pip \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Build AriaCoda (library to talk to the Pioneer robot)
RUN git clone https://github.com/reedhedges/AriaCoda.git /AriaCoda \
    && cd /AriaCoda && make -j2

# Copy our ROS2 package into the container
COPY . /ros2_ws/src/p3at/

# Build our package
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select p3at"

# Source ROS2 automatically when container starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
