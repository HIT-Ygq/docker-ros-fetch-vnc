FROM yuokamoto1988/ros-melodic-vnc-ws:latest

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

ENV DISPLAY=:1 \
    VNC_PORT=5900 \
    NO_VNC_PORT=6080
EXPOSE $VNC_PORT $NO_VNC_PORT

## Envrionment config
ENV VNCPASSWD=123456
ENV HOME=/root \
    INST_SCRIPTS=/root/install \
    NO_VNC_HOME=/root/noVNC \
    DEBIAN_FRONTEND=noninteractive \
    VNC_COL_DEPTH=16 \
    VNC_RESOLUTION=1920x1080 \
    VNC_PW=$VNCPASSWD \
    VNC_VIEW_ONLY=false

WORKDIR $HOME


# 设置中国镜像源
COPY sources.list /etc/apt/sources.list

# 设置工作空间 catkin_ws/src
COPY ${PWD}/gazebo_my /root/catkin_ws/src/gazebo_my
RUN /bin/sh -c '. /opt/ros/melodic/setup.sh; cd ~/catkin_ws ; catkin_make -j1'
RUN /bin/bash -c "echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc"

# 加载配置文件，避免系统出错
RUN rm -f /root/.ignition/fuel/config.yaml
COPY ${PWD}/.ignition /root/.ignition/
COPY ${PWD}/.gazebo /root/.gazebo/

# 安装 fetch-gazebo-demo
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update -q --fix-missing && \
     apt-get install -y ros-melodic-fetch-gazebo-demo 

# 安装 图片查看器
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update -q --fix-missing && \
     apt-get install -y gwenview

WORKDIR /root

