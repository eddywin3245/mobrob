FROM ros:jazzy-ros-base

# Switch to a more reliable Ubuntu mirror
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirror.us-tx.kamatera.com/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirror.us-tx.kamatera.com/ubuntu|g' /etc/apt/sources.list 2>/dev/null || true
RUN sed -i 's|http://security.ubuntu.com/ubuntu|http://mirror.us-tx.kamatera.com/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's|http://security.ubuntu.com/ubuntu|http://mirror.us-tx.kamatera.com/ubuntu|g' /etc/apt/sources.list 2>/dev/null || true

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
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-nmea-navsat-driver \  
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-usb-cam \
    python3-pip \
    python3-opencv \
    git \
    build-essential \
    cmake \
    iproute2 \
    iputils-ping \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Build Lakibeam LIDAR driver
RUN git clone https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver.git /lakibeam_ws/src/lakibeam1 && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd /lakibeam_ws && colcon build"
RUN echo "source /lakibeam_ws/install/setup.bash" >> /root/.bashrc

# Install DepthAI (OAK camera SDK) and Phidget22
RUN pip3 install --break-system-packages depthai Phidget22 matplotlib

# Build AriaCoda (library to talk to the Pioneer robot)
RUN git clone https://github.com/reedhedges/AriaCoda.git /AriaCoda \
    && cd /AriaCoda && make -j2

# Install AriaCoda headers and library so it can be found at runtime
RUN cp -r /AriaCoda/include/Aria /usr/local/include/ && \
    cp /AriaCoda/lib/libAria.so.3 /usr/local/lib/ && \
    ln -s /usr/local/lib/libAria.so.3 /usr/local/lib/libAria.so && \
    ldconfig

# Copy our ROS2 package into the container
COPY . /ros2_ws/src/p3at_robot/

# Build our package (includes ariaNode C++ node)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    source /lakibeam_ws/install/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select p3at_robot"

# Source ROS2 automatically when container starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /lakibeam_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
