# ROS Noetic Custom Packages Build Instructions

This repository contains a custom build script for compiling specific ROS Noetic packages on Ubuntu 22.04 Jammy.

## 构建的包列表 (Package List)

The build script will compile the following packages:

- `ros-noetic-ros-base` - ROS base packages
- `ros-noetic-rosmon` - ROSMon process monitor
- `ros-noetic-catkin` - Catkin build system
- `ros-noetic-behaviortree-cpp` - BehaviorTree.CPP library
- `ros-noetic-image-transport` - Image transport plugins
- `ros-noetic-cv-bridge` - OpenCV bridge
- `ros-noetic-teleop-twist-keyboard` - Keyboard teleoperation
- `ros-noetic-foxglove-bridge` - Foxglove bridge
- `ros-noetic-foxglove-msgs` - Foxglove messages
- `ros-noetic-backward-ros` - Backward debugging tools
- `ros-noetic-costmap-2d` - 2D costmap
- `ros-noetic-tf2-sensor-msgs` - TF2 sensor message transforms
- `ros-noetic-libg2o` - Graph optimization library
- `ros-noetic-nodelet` - Nodelet framework
- `ros-noetic-pcl-ros` - PCL ROS interface
- `ros-noetic-pcl-conversions` - PCL conversions
- `ros-noetic-image-transport-plugins` - Image transport plugins
- `ros-noetic-imagezero-image-transport` - ImageZero transport
- `ros-noetic-polled-camera` - Polled camera interface
- `ros-noetic-grid-map` - Grid map library
- `ros-noetic-sound-play` - Sound playback
- `ros-noetic-eigen-conversions` - Eigen conversions
- `ros-noetic-geodesy` - Geodesy utilities
- `ros-noetic-geographic-msgs` - Geographic messages
- `ros-noetic-geographic-info` - Geographic information
- `ros-noetic-tf-conversions` - TF conversions
- `ros-noetic-tf2-ros` - TF2 ROS interface
- `ros-noetic-tf2-eigen` - TF2 Eigen interface
- `ros-noetic-robot-state-publisher` - Robot state publisher
- `ros-noetic-camera-info-manager` - Camera info manager
- `ros-noetic-stage-ros` - Stage robot simulator

## 系统要求 (System Requirements)

- **Operating System**: Ubuntu 22.04 Jammy (amd64)
- **RAM**: Minimum 4GB, recommended 8GB+
- **Disk Space**: At least 10GB free space
- **Internet Connection**: Required for downloading packages and dependencies

## 使用说明 (Usage Instructions)

### 方法 1: 自动化构建 (Method 1: Automated Build)

```bash
# 1. Clone the repository or ensure you have the necessary files
cd /path/to/your/workspace

# 2. Make sure you have the required files:
#    - build.sh (build script)
#    - custom-packages.rosinstall (package list)

# 3. Run the build script
./build.sh
```

The script will:
1. Install all necessary build dependencies
2. Configure rosdep for Ubuntu 22.04
3. Download source code for all packages
4. Install package dependencies
5. Build and install packages to `/opt/ros/noetic`
6. Set up the ROS environment
7. Verify the installation

### 方法 2: 手动步骤 (Method 2: Manual Steps)

If you prefer to run the steps manually:

```bash
# 1. Install build dependencies
sudo apt-get update
sudo pip3 install -U rosdep rosinstall_generator vcstool
sudo apt-get install -y build-essential cmake git python3-dev python3-yaml python3-setuptools

# 2. Prepare rosdep
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://gitee.com/qinyinan/rosdistro/raw/master/rosdep/sources.list.d/20-default.list
export ROSDISTRO_INDEX_URL=https://gitee.com/qinyinan/rosdistro/raw/master/index-v4.yaml
rosdep update

# 3. Get source code
mkdir -p src
vcs import --input custom-packages.rosinstall ./src

# 4. Create install directories
sudo mkdir -p /opt/ros/noetic
sudo chmod 777 /opt/ros/noetic

# 5. Install dependencies
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

# 6. Build packages
./src/catkin/bin/catkin_make_isolated --install-space /opt/ros/noetic --install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 构建时间 (Build Time)

Expected build time varies by system:
- **4-core CPU**: ~60-90 minutes
- **8-core CPU**: ~30-45 minutes
- **16-core CPU**: ~15-25 minutes

## 使用构建的包 (Using the Built Packages)

After successful build:

```bash
# Source the ROS environment
source /opt/ros/noetic/setup.bash

# Verify installation
rospack list | grep -E "(roscpp|rospy|tf2_ros|image_transport)"

# Test basic functionality
roscore &
rostopic list
```

## 环境设置 (Environment Setup)

The build script will offer to automatically add ROS environment to your `~/.bashrc`. If you declined or want to set it up manually:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
source ~/.bashrc
```

## 故障排除 (Troubleshooting)

### Common Issues:

1. **Build fails due to missing dependencies**:
   ```bash
   # Retry dependency installation
   rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --reinstall
   ```

2. **Python 2/3 compatibility issues**:
   ```bash
   export PYTHON_EXECUTABLE=/usr/bin/python3
   export ROS_PYTHON_VERSION=3
   ```

3. **Memory issues during build**:
   ```bash
   # Reduce parallel jobs
   ./src/catkin/bin/catkin_make_isolated --install-space /opt/ros/noetic --install -j2
   ```

4. **Permission denied errors**:
   ```bash
   sudo chown -R $USER:$USER /opt/ros/noetic
   ```

### 查看构建日志 (View Build Logs):

Build logs are saved in the build directory. To check for errors:

```bash
# Look for the most recent build log
find . -name "*.log" -type f -exec ls -la {} \; | tail -10

# Check for common error patterns
grep -r "error:" build_isolated/ || echo "No errors found"
```

## 验证安装 (Verification)

After build completion, verify key packages:

```bash
# Source environment
source /opt/ros/noetic/setup.bash

# Check package availability
rospack find catkin
rospack find roscpp
rospack find tf2_ros
rospack find image_transport
rospack find cv_bridge
rospack find pcl_ros

# Test basic ROS functionality
roscore &
sleep 2
rostopic list
rosnode list
pkill roscore
```

## 文件结构 (File Structure)

After successful build:

```
/opt/ros/noetic/
├── bin/                    # Executable files
├── include/                # Header files
├── lib/                    # Libraries
├── share/                  # Package resources
├── setup.bash              # Environment setup script
└── ros_setup.sh           # Custom setup script
```

## 支持 (Support)

If you encounter issues:

1. Check that your system meets the requirements
2. Verify internet connectivity for package downloads
3. Ensure sufficient disk space (>10GB)
4. Review the troubleshooting section above
5. Check build logs for specific error messages

## 注意事项 (Important Notes)

- This build is specifically tested for Ubuntu 22.04 Jammy
- The packages will be installed to `/opt/ros/noetic` (standard ROS location)
- The build process requires internet connectivity
- Some packages may have additional runtime dependencies
- The built packages should be compatible with standard ROS Noetic applications