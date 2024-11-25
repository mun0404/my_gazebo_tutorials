## Author
- **Name** : Mohammed Munawwar
- **UID** : 120241642

## Building the Project

   ```bash
   # Source the ROS 2 setup script to configure your environment
   source /opt/ros/humble/setup.bash

   # Create a new ROS 2 workspace directory
   mkdir -p ~/my_gazebo_tutorials/src && cd ~/my_gazebo_tutorials/src

   # Clone the repository
   git clone https://github.com/mun0404/my_gazebo_tutorials.git

   # Install package dependencies using rosdep
   cd ..
   rosdep install -i --from-path src --rosdistro humble -y

   # Build the package using colcon
   colcon build

   # Source the package
   source ./install/setup.bash
   ```
## Export turtlebot
```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Running the Simulation

To start the Gazebo world with the turtlebot:
```bash
cd ~/my_gazebo_tutorials
ros2 launch walker sim.launch.py

# To run the controller:
ros2 run walker walker 
```

## ROS2 Bags
```bash
# Run the launch file
ros2 launch walker sim.launch.py record_ros_bag:=true

# Inspect the ros2 bag
ros2 bag info talkerbag

# Play back the ros2 bag
ros2 bag play talkerbag
```

## Style check

Perform these to ensure the quality of the code is maintained:
cppcheck, cpplint and clangd.

```bash
# Run cppcheck for code analysis
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $(find . -name "*.cpp" | grep -vE -e "^./build/") --check-config > results/cppcheck.txt

# Run cpplint for style checking
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp > results/cpplint.txt

# Run clang tidy
clang-tidy -extra-arg=-std=c++17 src/*.cpp

# To save the results of clang tidy
echo $? > results/clangtidy_output.txt

# To directly format the code according to the google style C++
clang-tidy -extra-arg=-std=c++17 src/*.cpp
```
