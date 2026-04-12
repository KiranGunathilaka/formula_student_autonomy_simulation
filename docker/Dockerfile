FROM nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install prerequisites
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release wget git \
    python3-pip python3-venv \
    x11-apps sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 Humble apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop (includes ROS, Gazebo integration) and build tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Foxglove Studio directly from GitHub releases
RUN ARCH=$(dpkg --print-architecture) && \
    wget -qO /tmp/foxglove-studio.deb "https://github.com/foxglove/studio/releases/download/v2.9.0/foxglove-studio-2.9.0-linux-${ARCH}.deb" && \
    apt-get update && \
    apt-get install -y /tmp/foxglove-studio.deb && \
    rm /tmp/foxglove-studio.deb && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update

# Set up global python venv to mirror setup_falcon.sh
ENV FALCON_VENV=/opt/falcon_venv
# Copy requirements file: context is the repository root
COPY scripts/requirements-falcon-env.txt /tmp/requirements-falcon-env.txt

# Create venv and install dependencies exactly mapping setup_falcon.sh logic
RUN python3 -m venv --system-site-packages ${FALCON_VENV} \
    && . ${FALCON_VENV}/bin/activate \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip uninstall -y empy EmPy \
    && pip install -r /tmp/requirements-falcon-env.txt \
    && pip install numpy'>=1.21.0,<2' \
    && pip install colcon-common-extensions \
    && pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126 \
    && pip install 'empy>=3.3,<4' --force-reinstall \
    && rm -rf /root/.cache/pip

# Working directory
WORKDIR /workspace

# Set up entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Environment settings for X11 forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
