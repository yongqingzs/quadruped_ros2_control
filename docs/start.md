# ocs2_quadruped_controller start
## build
base
```bash
# ubuntu 24.04
# ros2-jazzy
mkdir -p ~/jazzy_ws/src
cd ~/jazzy_ws/src
git clone https://github.com/legubiao/quadruped_ros2_control.git

cd ~/jazzy_ws
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j4" colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install

sudo apt-get install ros-jazzy-ros-gz
MAKEFLAGS="-j4" colcon build --packages-up-to gz_quadruped_playground --symlink-install
```

ocs2_ros2
```bash
cd ~/jazzy_ws/src
git clone https://github.com/legubiao/ocs2_ros2

cd ocs2_ros2
git submodule update --init --recursive

cd ..
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j4" colcon build --packages-up-to ocs2_quadruped_controller  --symlink-install
```

rl
```bash
cd ~
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip

echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc

cd ~/ros2_ws
MAKEFLAGS="-j4" colcon build --packages-up-to rl_quadruped_controller --symlink-install
```

## start
```bash
# 终端1
# base
ros2 launch unitree_guide_controller gazebo.launch.py
# mpc
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
# rl
ros2 launch rl_quadruped_controller gazebo.launch.py pkg_description:=go2_description

# 终端2
ros2 run keyboard_input keyboard_input
```

demo
![](resource/demo.gif)