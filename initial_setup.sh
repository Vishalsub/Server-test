#!/bin/bash

# Exit on errors
set -e

# ─── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

info()    { echo -e "${CYAN}${BOLD}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}${BOLD}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}${BOLD}[WARN]${RESET}  $*"; }
step()    { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${RESET}"; }

echo -e "${BOLD}${GREEN}"
echo "  ██████╗  ██████╗ ███████╗██████╗ "
echo "  ██╔══██╗██╔═══██╗██╔════╝╚════██╗"
echo "  ██████╔╝██║   ██║███████╗ █████╔╝"
echo "  ██╔══██╗██║   ██║╚════██║██╔═══╝ "
echo "  ██║  ██║╚██████╔╝███████║███████╗"
echo "  ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚══════╝"
echo -e "${RESET}"
echo -e "${BOLD}  ROS2 Humble + Gazebo Ignition Fortress${RESET}"
echo ""

# ─── Pre-flight: Fix any broken packages ─────────────────────────────────────
info "Checking for broken packages..."
# podman-docker owns /usr/bin/docker and blocks docker-ce-cli installation
sudo dpkg --purge --force-all podman-docker 2>/dev/null || true
sudo dpkg --purge --force-all docker-ce docker-ce-cli docker-ce-rootless-extras \
    docker-buildx-plugin docker-compose-plugin docker-model-plugin \
    containerd.io docker docker-engine docker.io 2>/dev/null || true
sudo rm -f /var/lib/dpkg/info/docker-ce.* /var/lib/dpkg/info/docker-ce-cli.*
sudo dpkg --configure -a
sudo apt --fix-broken install -y
sudo apt-get autoremove -y 2>/dev/null || true
success "System is clean"

# ─── Locale Setup ────────────────────────────────────────────────────────────
step "[1/6] Setting up locales"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
success "Locales configured"

# ─── ROS2 Humble Repository ──────────────────────────────────────────────────
step "[2/6] Adding ROS2 apt repository"
sudo apt install -y software-properties-common curl wget
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
success "ROS2 repository added"

# ─── Gazebo Ignition Fortress Repository ─────────────────────────────────────
step "[3/6] Adding Gazebo Ignition Fortress apt repository"
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
success "Gazebo repository added"

# NOTE: Avoid full upgrade — it can break ROS2's pinned library dependencies
info "Refreshing package lists..."
sudo apt update

# ─── ROS2 Humble ─────────────────────────────────────────────────────────────
step "[4/6] Installing ROS2 Humble"
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
sudo apt install -y ros-humble-ament-cmake
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y libmodbus-dev socat
success "ROS2 Humble installed"

# ─── Gazebo Ignition Fortress + ROS2 Bridge ──────────────────────────────────
step "[5/6] Installing Gazebo Ignition Fortress and ROS2-GZ bridge"
sudo apt install -y ignition-fortress
sudo apt install -y ros-humble-ros-gz
success "Gazebo Ignition Fortress installed"

# ─── Shell Environment ───────────────────────────────────────────────────────
# Done BEFORE pip/docker so a crash there doesn't leave ROS2 unsourced
step "[6/6] Configuring shell environment"
if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
    info "Added ROS2 sourcing to .bashrc"
else
    warn "ROS2 sourcing already in .bashrc — skipped"
fi
if ! grep -q "RMW_IMPLEMENTATION" "$HOME/.bashrc"; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$HOME/.bashrc"
    info "Added RMW_IMPLEMENTATION to .bashrc"
else
    warn "RMW_IMPLEMENTATION already in .bashrc — skipped"
fi
success "Shell environment configured"

# ─── Python3 ─────────────────────────────────────────────────────────────────
info "Installing Python3 pip..."
sudo apt install -y python3-pip
success "Python3 pip installed"

# ─── Docker ──────────────────────────────────────────────────────────────────
info "Installing Docker..."
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
# Install CLI and runtime first so docker-ce dependency is already satisfied
sudo apt-get install -y docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt-get install -y docker-ce docker-ce-rootless-extras
sudo groupadd docker 2>/dev/null || true
sudo usermod -aG docker "$USER"
success "Docker installed"

echo ""
echo -e "${BOLD}${GREEN}════════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${GREEN}  Installation completed successfully!${RESET}"
echo -e "${GREEN}  ROS2 Humble  : /opt/ros/humble${RESET}"
echo -e "${GREEN}  Gazebo        : Ignition Fortress${RESET}"
echo -e "${GREEN}  RMW           : rmw_cyclonedds_cpp${RESET}"
echo ""
echo -e "${YELLOW}  Run 'source ~/.bashrc' or open a new terminal${RESET}"
echo -e "${YELLOW}  before using ros2 commands.${RESET}"
echo -e "${BOLD}${GREEN}════════════════════════════════════════════════════════${RESET}"
