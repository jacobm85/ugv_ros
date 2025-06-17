# Use Ubuntu Noble LTS as base
ARG RELEASE=noble
FROM ubuntu:${RELEASE}

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV USERNAME=ubuntu
ENV USER_UID=1000
ENV USER_GID=1000
ENV ROS_DISTRO=jazzy
ENV BOT_HOME=/home/${USERNAME}
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8


# Enable universe repo and update
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y curl
	
# Add ROS 2 Jazzy apt source
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y ros-dev-tools

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-${ROS_DISTRO}-desktop

# Install system dependencies
RUN apt-get install -y \
    gnupg2 \
    lsb-release \
    sudo \
    git \
    nano \
    passwd \
    locales \
    iputils-ping \
    usbutils \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-networkx \
    python3-pip \
    python3-opencv \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-serial \
    python3-pygame \
    python3-lgpio \
    python3-flask \
    python3-requests \
    libopencv-dev && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8


# Install additional ROS packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rtabmap \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-velodyne \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-depthai-* \
    ros-${ROS_DISTRO}-cartographer-* \
    ros-${ROS_DISTRO}-joint-state-publisher-* \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-rosbridge-* \
    ros-${ROS_DISTRO}-rqt-* \
    ros-${ROS_DISTRO}-rtabmap-* && \
    apt-get clean && rm -rf /var/lib/apt/lists/* 

# Dont run this now - Conditionally initialize rosdep
#RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || (rosdep init && rosdep update)

# Create user 'ugv' and set password
#RUN if ! getent group ${USER_GID} >/dev/null; then \
#        groupadd --gid ${USER_GID} ${USERNAME}; \
#    fi && \
#    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash && \
#    echo "${USERNAME}:${USERNAME}" | chpasswd
RUN usermod -aG sudo ${USERNAME}


# Set up ROS environment for ugv user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "eval \"\$(register-python-argcomplete ros2)\"" >> /home/${USERNAME}/.bashrc && \
    echo "eval \"\$(register-python-argcomplete colcon)\"" >> /home/${USERNAME}/.bashrc

# Switch to ugv user and set up workspace
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Clone and set up workspace
RUN mkdir -p /home/${USERNAME}/ugv_ws && \
    cd /home/${USERNAME}/ugv_ws && \
    git clone https://github.com/jacobm85/ugv_ros /tmp/ugv_ros && \
    mv /tmp/ugv_ros/* /home/${USERNAME}/ugv_ws/ && \
    rm -rf /tmp/ugv_ros

# Build workspace (explicit source in each RUN)
WORKDIR /home/${USERNAME}/ugv_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select \
        emcl2 explore_lite openslam_gmapping \
        slam_gmapping ldlidar rf2o_laser_odometry robot_pose_publisher \
        vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server \
        ugv_base_node ugv_interface && \
    colcon build --packages-select \
        ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam \
        ugv_tools ugv_vision ugv_web_app --symlink-install && \
    colcon build --packages-select \
        costmap_converter_msgs costmap_converter teb_msgs teb_local_planner"

# Add workspace setup to bashrc
RUN echo "source /home/${USERNAME}/ugv_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# Fix ownership
USER root
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Fix USB udev permissions
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules

# Final config
RUN cp /home/${USERNAME}/ugv_ws/ros_entrypoint.sh /ros_entrypoint.sh && \
    chmod +x /home/${USERNAME}/ugv_ws/ros_entrypoint.sh /ros_entrypoint.sh
# Set the entrypoint for the container
ENTRYPOINT ["/ros_entrypoint.sh"]
USER ${USERNAME}
WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]
