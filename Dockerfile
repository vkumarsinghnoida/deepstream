# Use the official ROS 2 Humble base image
FROM ros:humble-ros-base

# Set the working directory
WORKDIR /root/

# Install dependencies
RUN apt update && apt install -y git python3 python3-pip nano rsync && \
    pip3 install git-filter-repo setuptools==58.2.0 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Setup ROS 2 workspace and build the demon package
RUN /bin/bash -c "mkdir -p ros2_ws/src && \
    source /opt/ros/humble/setup.bash && \
    cd ros2_ws && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    cd src && \
    ros2 pkg create --build-type ament_python --license Apache-2.0 demon && \
    colcon build --packages-select demon"

# Clone the deepstream repository, filter it, and move the demon directory
RUN git clone https://github.com/vkumarsinghnoida/deepstream && \
    cd deepstream && \
    git filter-repo --path demon && \
    git gc --aggressive --prune=all && \
    rsync -av --delete demon/ /root/ros2_ws/src/demon/ && \
    rm -rf demon && \
    cd /root/ros2_ws/src/ && \
    rm -rf ../../deepstream

# Build the workspace
RUN cd /root/ros2_ws/ && \
    colcon build --symlink-install

# Source setup files for ROS 2 and the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command to launch the demon package
#CMD ["/bin/bash", "-c", "source /root/ros2_ws/install/setup.bash && ros2 launch demon lower_tk_ls.yaml"]

# Copy the entry point script into the container
COPY entrypoint.sh /usr/local/bin/entrypoint.sh

# Set the entry point script as the entry point
ENTRYPOINT ["entrypoint.sh"]
