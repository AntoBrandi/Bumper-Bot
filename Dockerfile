FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    terminator \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    vim \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --shell /bin/bash --create-home user && \
    adduser user sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER user

ENV WS_DIR=/home/user/bumperbot_ws
RUN mkdir -p $WS_DIR/src
WORKDIR $WS_DIR

RUN git clone https://github.com/AntoBrandi/Bumper-Bot.git src/Bumper-Bot
RUN sudo rosdep init || true
RUN rosdep update
RUN sudo apt-get update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/jazzy/setup.bash && colcon build
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/user/.bashrc
RUN echo "source $WS_DIR/install/setup.bash" >> /home/user/.bashrc

CMD ["terminator"]