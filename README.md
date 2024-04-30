# f1tenth-software-stack

F1TENTH software stack is the integrated repository of ESE 615 Team 6 of Spring 2023. 

For the setup of F1TENTH cars, please review Repository [f1tenth_system](https://github.com/f1tenth/f1tenth_system). Modifications are needed based on specific cars. 

For the setup of F1TENTH ROS Simulator, please review Repository [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros). 

For the setup of F1TENTH Gym, please review Repository [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym). 


## Run Lab 1 Package
```bash
cd ~/sim_ws
colcon build --packages-select lab1_pkg
source /opt/ros/foxy/setup.bash
. install/setup.bash
ros2 launch lab1_pkg lab1_launch.py
```

## Run AEB in Sim

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run safety_node safety_node.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # control the keyboard
```

## Run Wall Following in Sim

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run wall_follow wall_follow_node.py
```

## Run Gap Following in Sim

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run gap_follow reactive_node.py
```

## Run Pure Pursuit, LQR, and MPC in Sim

Modify sim.yaml to load the levine 2nd floor map by using
```yaml
map_path: '/home/derek/sim_ws/src/f1tenth_gym_ros/maps/levine_2nd'
map_img_ext: '.pgm'
```

Go to pure_pursuit_node.py and change the code with
```python
self.is_real = False
```
```python
# self.ref_speed = csv_data[:, 5] * 0.6  # max speed for levine 2nd - real is 2m/s
self.ref_speed = csv_data[:, 5]  # max speed - sim is 10m/s
```
```python
# sim params
self.L = 1.0
self.steering_gain = 0.5
```

Go to lqr_node.py and switch the mode to sim
```python
self.is_real = False
```

Go to mpc_node.py and change the code with
```python
self.is_real = False
```
```python
MAX_SPEED: float = 5.0  # maximum speed [m/s] ~ 5.0 for levine sim
```
All the other parameters remain the same. 

Go to levine_sim_launch.py, comment & uncomment the corresponding code of pure pursuit / LQR / MPC and execute through the launch file. 

```bash
cd ~/sim_ws/
colcon build
ros2 launch bringup levine_sim_launch.py
```

## Run Motion Planning (RRT* + Pure Pursuit) in Sim

Copy the "slam_maps" folder in "lab6_pkg" folder to "maps" folder of "f1tenth_gym_ros" folder and rename it as "motion_planning". Modify sim.yaml to load the levine 2nd floor map for lab 6 config by using
```yaml
map_path: '/home/derek/sim_ws/src/f1tenth_gym_ros/maps/motion_planning/levine_2nd'
map_img_ext: '.pgm'
```

Run the following commands in 3 terminals
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run lab6_pkg rrt_node.py
ros2 run lab6_pkg motion_planning_pure_pursuit_node.py
```

## Run Final Race (Gap Following + Pure Pursuit) in Sim

Modify sim.yaml of f1tenth_gym_ros to load the clear Skirkanich map by using

```yaml
map_path: '/home/derek/sim_ws/src/f1tenth_gym_ros/maps/skir_2_draw'
map_img_ext: '.pgm'
```

To load the Skirkanich map with random obstacles, change the map name as **skir_2_draw_obs**.

Make sure we are using the normal speed by checking code is commented in final_race_pure_pursuit_node.py as following

```python
# self.drive_msg.drive.speed = (0.2 if gf_point_obs else 1.0) * speed  # if allows braking for close obstacles
self.drive_msg.drive.speed = speed
# self.drive_msg.drive.speed = self.test_speed
```

Modify the parameters in final_race_sim_launch.py as

```python
{"pure pursuit confidence ratio": 1.0},         # weight of pure pursuit versus gap follow
```

```python
{"reference speed gain": 0.7},                  # weight of reference speed, original - 0.7
```

For obstacle avoidance, change the 2 parameters as **0.4** and **0.5** respectively.

```bash
cd ~/sim_ws/
colcon build
ros2 launch bringup final_race_sim_launch.py
```

<!-- ## Run MEGA-DAgger Agent Node

Add the map data and the trajectories, create the **f110_mega-dagger** conda environment, and run following commands

```bash
# first terminal
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
# second terminal
conda activate f110_mega-dagger
ros2 run mega_dagger_agent agent_node.py  # switch self.is_real to change the sim / real states
```

This is a general method to run deep learning methods on F1TENTH cars without TensorRT deployment. For detailed implementation, please check the **upper-lower machine communication** in ESE 615 Tutorial 6. For a general conda env config, please check the readme in [here](https://github.com/derekhanbaliq/LQR-based-Path-Tracking). -->

## Wireless Visualization via Rviz2

Thanks to the [ESE 615 tips](https://docs.google.com/document/d/1PhaZvV0ZKzfTiwoJAoGcjTY9W2EPkMq2NKQgz8E-glk/edit)!

```bash
ifconfig  # check wlan0, should be same to the car's
echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc
sudo ufw disable  # disable the firewall
rviz2  # on your native ubuntu and add the topics, start rviz2 before pf!
```

## Intro to Supported Repos

**f1tenth_traj_gen**: trajectory generation repo for solving min curvature QP with F1TENTH params. The dev is based on [global traj optim repo of TUM](https://github.com/TUMFTM/global_racetrajectory_optimization). For f1tenth dev, the tuned parameter is locally stored in IL folder of Derek's OMEN-16 via Windows 11 OS.
