#!/bin/bash

# Exit on error
set -e

# Function to print status messages
echo_info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}

# Function to print error messages and exit
echo_error() {
    echo -e "\033[1;31m[ERROR] $1\033[0m"
    exit 1
}

# Check if running on Ubuntu 24.04
if [[ "$(lsb_release -rs)" != "24.04" ]]; then
    echo_error "This script requires Ubuntu 24.04"
fi


# Update and upgrade the system
echo_info "Updating and upgrading system packages"
sudo apt update && sudo apt upgrade -y

# Install basic tools
echo_info "Installing basic tools"
sudo apt install -y wget curl git software-properties-common

# Set up locale for ROS 2
echo_info "Configuring locale"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


# Add ROS 2 Jazzy repository
echo_info "Adding ROS 2 GPG key and repository"
#sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Add Gazebo GPG key and repository
echo_info "Adding Gazebo GPG key and repository"
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
echo_info "Updating package lists"
sudo apt update

# Install ROS 2 Jazzy desktop and development tools
echo_info "Installing ROS 2 Jazzy Jalisco desktop"
sudo apt install -y ros-jazzy-desktop ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-ament-lint ros-jazzy-ros-workspace

# Install Python dependencies for ROS 2
echo_info "Installing ROS 2 development tools"
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Install Gazebo Harmonic and development libraries
echo_info "Installing Gazebo Harmonic"
sudo apt install -y gz-harmonic libgz-math7-dev 

# Initialize rosdep
echo_info "Initializing rosdep"
sudo rosdep init || true
rosdep update

# Add Gazebo sources to rosdep
echo_info "Adding Gazebo sources to rosdep"
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update


# Install ROS-Gazebo integration packages
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# Install ArduPilot SITL dependencies
echo_info "Installing ArduPilot SITL dependencies"
sudo apt install -y python3-future python3-lxml python3-wxgtk4.0 python3-matplotlib python3-opencv python3-serial python3-yaml

# Clone ArduPilot repository (for SITL build outside workspace)
echo_info "Cloning ArduPilot repository"
cd ~
if [ ! -d "ardupilot" ]; then
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git submodule update --init --recursive
else
    cd ardupilot
fi

# Install ArduPilot build tools
echo_info "Install ArduPilot prereqs"
Tools/environment_install/install-prereqs-ubuntu.sh -y


# Build ArduPilot SITL
echo_info "Building ArduPilot SITL"
cd ~/ardupilot
./waf configure --board sitl
./waf build --target bin/arducopter

# Create ROS 2 workspace
echo_info "Creating ROS 2 workspace"
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws

# Clone ArduPilot ROS 2 repositories
#echo_info "Cloning ArduPilot ROS 2 repositories"
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/refs/heads/jazzy/ros2_gz.repos --recursive src

# Install Micro-ROS and DDS dependencies
#echo_info "Install Micro-ROS and DDS dependencies"
sudo apt install -y ros-jazzy-micro-ros-msgs ros-jazzy-geographic-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs openjdk-17-jre gradle
#sudo update-alternatives --set javac /usr/lib/jvm/java-17-openjdk-amd64/bin/javac

# Install Micro XRCE-DDS Generator
echo_info "Install Micro XRCE-DDS generator"
cd ~
if [ ! -d "Micro-XRCE-DDS-Gen" ]; then
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
    cd Micro-XRCE-DDS-Gen
    ./gradlew assemble
    chmod +x scripts/microxrceddsgen
    echo "Installed microxrceddsgen in ~/Micro-XRCE-DDS-Gen/scripts."
else
    cd Micro-XRCE-DDS-Gen
    git pull
    ./gradlew assemble
    chmod +x scripts/microxrceddsgen
    echo "Updated microxrceddsgen in ~/Micro-XRCE-DDS-Gen/scripts."
fi
cd ~

# Verify microxrceddsgen version
echo_info "Checking microxrceddsgen version..."
$HOME/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen -version

# Add Micro-XRCE-DDS-Gen/scripts to PATH
if ! grep -q "Micro-XRCE-DDS-Gen/scripts" ~/.bashrc; then
    echo 'export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts' >> ~/.bashrc
    echo "Added ~/Micro-XRCE-DDS-Gen/scripts to PATH in ~/.bashrc."
fi

# Create alias for microxrceddsgen
if ! grep -q "alias microxrceddsgen" ~/.bashrc; then
    echo "alias microxrceddsgen='$HOME/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen'" >> ~/.bashrc
    echo "Added microxrceddsgen alias to ~/.bashrc."
fi

echo_info "Switch WS repositories to Jazzy"

cd ~/ardu_ws/src

# Clone or update Micro-ROS agent with explicit jazzy branch
if [ ! -d "micro_ros_agent" ]; then
    git clone -b jazzy https://github.com/micro-ROS/micro-ros-agent.git
    echo_info "Cloned micro_ros_agent with jazzy branch."
else
    cd micro_ros_agent
    git fetch origin
    git checkout jazzy || { echo "jazzy branch not found, trying main"; git checkout main; }
    git pull
    echo_info "Updated micro_ros_agent to jazzy branch."
    cd ..
fi

# Clone or update ros_gz with explicit jazzy branch
if [ ! -d "ros_gz" ]; then
    git clone -b jazzy https://github.com/gazebosim/ros_gz.git ros_gz
    echo_info "Cloned ros_gz with jazzy branch."
else
    cd ros_gz
    git fetch origin
    git checkout jazzy || { echo "jazzy branch not found, trying main"; git checkout main; }
    git pull
    echo_info "Updated ros_gz to jazzy branch."
    cd ..
fi

# Clone or update sdformat_urdf with compatible branch
if [ ! -d "sdformat_urdf" ]; then
    git clone -b jazzy https://github.com/ros/sdformat_urdf.git sdformat_urdf
    echo_info "Cloned sdformat_urdf with main branch."
else
    cd sdformat_urdf
    git fetch origin
    git checkout jazzy || { echo "main branch not found, keeping current branch"; }
    git pull
    echo_info "Updated sdformat_urdf to jazzy branch."
    cd ..
fi

# Clone or update ardupilot with ros2 branch (preferred for ROS 2 Jazzy)
if [ ! -d "ardupilot" ]; then
    git clone -b ros2 https://github.com/ArduPilot/ardupilot.git ardupilot || git clone -b master https://github.com/ArduPilot/ardupilot.git ardupilot
    echo_info "Cloned ardupilot with ros2 branch (or master as fallback)."
else
    cd ardupilot
    git fetch origin
    git checkout ros2 || { echo "ros2 branch not found, trying master"; git checkout master; }
    git pull
    echo_info "Updated ardupilot to ros2 branch (or master)."
    cd ..
fi

# Patch CMakeLists.txt to remove invalid microxrceddsgen argument
if [ -f "~/ardu_ws/src/ardupilot/libraries/AP_DDS/wscript" ]; then
    sed -i 's/-default-container-prealloc-size[^ ]*//g' ~/ardu_ws/src/ardupilot/libraries/AP_DDS/wscript
    echo_info "Patched ~/ardu_ws/src/ardupilot/libraries/AP_DDS/wscript to remove -default-container-prealloc-size."
else
    echo "No wscript file"
fi

# Install workspace dependencies
echo_info "Install ROS deps for WS"
cd ~/ardu_ws
rosdep install --from-paths src --ignore-src -r -y

# Install ArduPilot SITL dependencies
#sudo apt install -y python3-pip python3-dev python3-setuptools
#pip3 install --user empy pexpect future pymavlink MAVProxy


# Set up WSL GUI support (requires X server on Windows, e.g., VcXsrv)
echo_info "Write to .bashrc"
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
#echo "source /usr/share/gz-harmonic/setup.bash" >> ~/.bashrc
#echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
#echo "export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf):0" >> ~/.bashrc

# Install useful tools
echo_info "Install useful tools"
sudo apt install -y mesa-utils glmark2 mc

# Clean up
echo_info "Cleanup"
sudo apt autoremove -y
sudo apt autoclean

echo "Environment setup complete! Source ROS 2 with 'source /opt/ros/jazzy/setup.bash'."
echo "To build your workspace, run: cd ~/ardu_ws && colcon build --symlink-install"
