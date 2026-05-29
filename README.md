# IMU_sensor
This is basically a mini experiment on Trajectory mapping using IMU sensor 
IMU used is ADXL 345

# Setup
Create the package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python imu_sim
cd imu_sim/imu_sim
```

Install pyserial
```bash
pip install pyserial
```

Build and run
```bash
cd ~/ros2_ws
colcon build --packages-select imu_sim
source install/setup.bash
ros2 launch imu_sim sim.launch.py
```