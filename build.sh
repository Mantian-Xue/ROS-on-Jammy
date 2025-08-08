#!/bin/bash

# Build script for custom ROS Noetic packages on Ubuntu 22.04
# This script builds the following packages: ros-base, rosmon, catkin, behaviortree-cpp,
# image-transport, cv-bridge, teleop-twist-keyboard, foxglove-bridge, foxglove-msgs,
# backward-ros, costmap-2d, tf2-sensor-msgs, libg2o, nodelet, pcl-ros, pcl-conversions,
# image-transport-plugins, imagezero-image-transport, polled-camera, grid-map, sound-play,
# eigen-conversions, geodesy, geographic-msgs, geographic-info, tf-conversions, tf2-ros,
# tf2-eigen, robot-state-publisher, camera-info-manager, stage-ros

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if running as root for certain operations
check_sudo() {
    if [[ $EUID -eq 0 ]]; then
        SUDO=""
    else
        SUDO="sudo"
    fi
}

print_status "Starting ROS Noetic custom packages build for Ubuntu 22.04"

# Step 1: Install build dependencies
print_status "Step 1: Installing build dependencies..."
check_sudo

# Update package list
$SUDO apt-get update

# Install required Python packages
if ! command_exists pip3; then
    print_status "Installing pip3..."
    $SUDO apt-get install -y python3-pip
fi

print_status "Installing Python build tools..."
$SUDO pip3 install -U rosdep rosinstall_generator vcstool

# Install additional build dependencies
print_status "Installing additional build dependencies..."
$SUDO apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-yaml \
    python3-setuptools \
    python3-catkin-tools \
    curl \
    wget \
    lsb-release \
    ca-certificates \
    gnupg2

print_success "Build dependencies installed successfully"

# Step 2: Prepare rosdep
print_status "Step 2: Preparing rosdep..."

# Create rosdep sources directory if it doesn't exist
$SUDO mkdir -p /etc/ros/rosdep/sources.list.d/

# Download the sources list (using Chinese mirror as per README)
print_status "Downloading rosdep sources list..."
$SUDO curl -o /etc/ros/rosdep/sources.list.d/20-default.list \
    https://gitee.com/qinyinan/rosdistro/raw/master/rosdep/sources.list.d/20-default.list

# Set up environment variable for rosdep update
export ROSDISTRO_INDEX_URL=https://gitee.com/qinyinan/rosdistro/raw/master/index-v4.yaml

# Add to bashrc for persistence
if ! grep -q "ROSDISTRO_INDEX_URL" ~/.bashrc; then
    echo 'export ROSDISTRO_INDEX_URL=https://gitee.com/qinyinan/rosdistro/raw/master/index-v4.yaml' >> ~/.bashrc
    print_status "Added ROSDISTRO_INDEX_URL to ~/.bashrc"
fi

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_error "Failed to download rosdep sources list"
    exit 1
fi

# Update rosdep
print_status "Updating rosdep database..."
rosdep update

print_success "rosdep prepared successfully"

# Step 3: Get source code
print_status "Step 3: Getting source code..."

# Create source directory
mkdir -p src

# Import packages using custom rosinstall file
if [ -f "custom-packages.rosinstall" ]; then
    print_status "Importing packages from custom-packages.rosinstall..."
    vcs import --input custom-packages.rosinstall ./src
else
    print_error "custom-packages.rosinstall file not found!"
    exit 1
fi

print_success "Source code downloaded successfully"

# Step 3.5: Apply source patches (Qt deprecations)
print_status "Step 3.5: Applying source patches..."
if [ -d "src/rosmon/rqt_rosmon" ]; then
    print_status "Patching rqt_rosmon Qt roles..."
    # Replace deprecated Qt roles to avoid -Werror build failures
    find src/rosmon/rqt_rosmon -type f \( -name '*.cpp' -o -name '*.h' \) -print0 | xargs -0 sed -i \
        -e 's/Qt::BackgroundColorRole/Qt::BackgroundRole/g' \
        -e 's/Qt::TextColorRole/Qt::ForegroundRole/g'
    print_success "Patched rqt_rosmon."
else
    print_warning "rqt_rosmon source not found; skipping rqt_rosmon patch."
fi

# Step 4: Create install directories
print_status "Step 4: Creating install directories..."

$SUDO mkdir -p /opt/ros/noetic
$SUDO chmod 777 /opt/ros/noetic

print_success "Install directories created"

# Step 5: Install build dependencies using rosdep
print_status "Step 5: Installing package dependencies using rosdep..."

# Set PYTHON_EXECUTABLE to ensure Python 3 is used
export PYTHON_EXECUTABLE=/usr/bin/python3

# Install dependencies
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

print_success "Package dependencies installed"

# Step 6: Build and install packages
print_status "Step 6: Building and installing ROS packages..."

# Check if catkin_make_isolated exists
if [ ! -f "./src/catkin/bin/catkin_make_isolated" ]; then
    print_error "catkin_make_isolated not found! Make sure catkin package was downloaded correctly."
    exit 1
fi

# Set build parameters
export CMAKE_PREFIX_PATH="/opt/ros/noetic:$CMAKE_PREFIX_PATH"
export PYTHON_EXECUTABLE=/usr/bin/python3

# Build packages
print_status "Starting build process... This may take a while."
./src/catkin/bin/catkin_make_isolated \
    --install-space /opt/ros/noetic \
    --install \
    -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_ENABLE_TESTING=OFF \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -j$(nproc)

print_success "Build completed successfully!"

# Step 7: Set up environment
print_status "Step 7: Setting up environment..."

# Create setup script
cat > /tmp/ros_setup.sh << 'EOF'
#!/bin/bash
# ROS Noetic Environment Setup
source /opt/ros/noetic/setup.bash
export ROS_PYTHON_VERSION=3
export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH"
EOF

$SUDO mv /tmp/ros_setup.sh /opt/ros/noetic/
$SUDO chmod +x /opt/ros/noetic/ros_setup.sh

# Add to user's bashrc if desired
read -p "Add ROS environment to ~/.bashrc? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# ROS Noetic Environment" >> ~/.bashrc
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
        print_success "ROS environment added to ~/.bashrc"
    else
        print_warning "ROS environment already in ~/.bashrc"
    fi
fi

# Step 8: Verification
print_status "Step 8: Verifying installation..."

# Source the environment
source /opt/ros/noetic/setup.bash

# Check if key packages are available
PACKAGES_TO_CHECK=(
    "roscpp"
    "rospy" 
    "roslaunch"
    "catkin"
    "tf2_ros"
    "image_transport"
    "cv_bridge"
    "pcl_ros"
    "robot_state_publisher"
)

print_status "Checking installed packages..."
for pkg in "${PACKAGES_TO_CHECK[@]}"; do
    if rospack find "$pkg" >/dev/null 2>&1; then
        print_success "✓ $pkg found"
    else
        print_warning "✗ $pkg not found"
    fi
done

# Final status
print_success "==================================="
print_success "ROS Noetic custom build completed!"
print_success "==================================="
print_status "Installation location: /opt/ros/noetic"
print_status "To use ROS, run: source /opt/ros/noetic/setup.bash"
print_status "Or add the source command to your ~/.bashrc"

# Create a summary log
echo "Build completed at: $(date)" > build_summary.log
echo "Built packages count: $(find /opt/ros/noetic/share -maxdepth 1 -type d | wc -l)" >> build_summary.log
echo "Installation size: $(du -sh /opt/ros/noetic | cut -f1)" >> build_summary.log

print_status "Build summary saved to build_summary.log"
print_success "Build script finished successfully!"