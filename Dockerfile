ARG base_image="robotnik/ros"
ARG ros_distro="humble"
ARG image_version=0.4.0
FROM ${base_image}:${ros_distro}-builder-${image_version}

USER root

# Install compiled packages
RUN --mount=type=bind,\
target=/tmp/requirements.txt,\
source=requirements/packages.txt \
    true \
    && apt-get update \
    && apt-get install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

USER ${USER_NAME}

RUN --mount=type=bind,\
source=./repos/common.repo.yml,\
target=/tmp/common.repo.yml,ro \
     vcs import \
        --input /tmp/common.repo.yml  \
        --shallow


RUN --mount=type=bind,\
source=requirements/python.txt,\
target=/tmp/requirements.txt,ro \
    python3 -m pip install -r /tmp/requirements.txt

USER $USER_NAME

RUN \
    compile_workspace.sh

# Add workspace to source
ENV ROS_SETUP_FILES "/home/$USER_NAME/robot_ws/install/local_setup.bash"

# Configure startup command
ENV STARTUP_TYPE "run"
# script of program to launch with all its arguments
ENV ROS_BU_PKG "cpu_monitor"

ENV ROS_BU_LAUNCH "cpu_monitor"

ENV EDGE_IP "10.10.10.212"
ENV IPERF3_PORT "5201"
ENV INFLUXDB_HOST "localhost"
ENV INFLUXDB_PORT "8086"
ENV INFLUXDB_USER "admin"
ENV INFLUXDB_PASS "admin"
ENV INFLUXDB_DB_NAME "openwrt"
ENV PUBLISH_RATE "1.0"
ENV BATTERY_TOPIC "/robot/battery_estimator/data"