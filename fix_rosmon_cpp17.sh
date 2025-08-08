#!/bin/bash

# Fix rosmon_core C++17 compatibility issue
echo "Fixing rosmon_core C++17 compatibility issue..."

# Find the rosmon_core CMakeLists.txt file
ROSMON_CMAKE=""

# Check common locations
if [ -f "/home/max/ROS-on-Jammy/src/rosmon/rosmon_core/CMakeLists.txt" ]; then
    ROSMON_CMAKE="/home/max/ROS-on-Jammy/src/rosmon/rosmon_core/CMakeLists.txt"
elif [ -f "src/rosmon/rosmon_core/CMakeLists.txt" ]; then
    ROSMON_CMAKE="src/rosmon/rosmon_core/CMakeLists.txt"
elif [ -f "rosmon/rosmon_core/CMakeLists.txt" ]; then
    ROSMON_CMAKE="rosmon/rosmon_core/CMakeLists.txt"
else
    echo "Error: Could not find rosmon_core/CMakeLists.txt"
    echo "Please specify the path to your ROS workspace"
    exit 1
fi

echo "Found CMakeLists.txt at: $ROSMON_CMAKE"

# Create a backup
cp "$ROSMON_CMAKE" "${ROSMON_CMAKE}.backup"
echo "Created backup: ${ROSMON_CMAKE}.backup"

# Apply the fix using sed
# Add C++17 settings after the find_package(catkin ...) block
sed -i '/find_package(catkin REQUIRED COMPONENTS/,/)/a\\n# Set C++17 standard for log4cxx compatibility\nset(CMAKE_CXX_STANDARD 17)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)\nset(CMAKE_CXX_EXTENSIONS OFF)\n' "$ROSMON_CMAKE"

# Comment out or replace the -std=c++11 flag
sed -i 's/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")/#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # Commented out - using CMAKE_CXX_STANDARD instead/' "$ROSMON_CMAKE"

echo "Fix applied successfully!"
echo ""
echo "You can now rebuild rosmon_core with:"
echo "  cd <your_catkin_workspace>"
echo "  catkin build rosmon_core"
echo ""
echo "Or if using catkin_make:"
echo "  catkin_make --only-pkg-with-deps rosmon_core"