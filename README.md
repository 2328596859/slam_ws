# slam_ws

**更新时间：2025.6.7**

## Cartographer 编译配置说明

- Cartographer 版本：官方 tag `v2.0.0`

---

## 依赖安装

1. **Ceres Solver**
   ```bash
   sudo apt install libceres-dev
   ```

2. **Lua 5.2**
   ```bash
   sudo apt install liblua5.2-dev
   ```

3. **Abseil-cpp**
   ```bash
   git clone https://github.com/abseil/abseil-cpp.git
   cd abseil-cpp
   git fetch origin
   git checkout 9c02e2cbe4174d4c410c3f6c20700f9975189c93
   mkdir build && cd build
   cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
   make -j$(nproc)
   sudo make install
   ```
   - **如需卸载/重装 Abseil：**
     ```bash
     sudo rm -rf /usr/local/include/absl
     sudo rm -rf /usr/local/lib/cmake/absl
     sudo rm -f /usr/local/lib/libabsl*
     ```

---

## 编译 Cartographer

```bash
cd ~/ALi-workspace-study/slam_ws/thirdPart/cartographer
rm -rf build
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=/usr/local ..
make -j$(nproc)
```

---

## 编译 cartographer_ros

```bash
catkin_make_isolated --install --use-ninja --cmake-args -DPYTHON_EXECUTABLE=$(which python3)
```

---
## 运行

```bash
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/home/leju-ali/ALi-workspace-study/slam_ws/rosbag/cartographer_paper_deutsches_museum.bag
```

> created by lisong