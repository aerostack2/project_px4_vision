FROM osrf/ros:humble-desktop

# Clone px4 autopilot
WORKDIR /root/
RUN git clone https://github.com/PX4/PX4-Autopilot.git -b v1.14.3 --recursive
WORKDIR /root/PX4-Autopilot/

# Upgrading the system
RUN apt update && apt upgrade -y
RUN apt-get install wget -y

# Install dependencies
RUN ./Tools/setup/ubuntu.sh --no-nuttx
RUN make px4_sitl
