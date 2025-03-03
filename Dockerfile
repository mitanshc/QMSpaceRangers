# Use ROS2 Humble base image
FROM ros:humble

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV ROS_DOMAIN_ID=10
ENV ROS_IP=0.0.0.0

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source ROS2 and workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]

