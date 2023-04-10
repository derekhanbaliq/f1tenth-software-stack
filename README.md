# ESE-615-Final-Project
ESE 615 Final Project repository of Team 6. For various resources and software development.

## Intro to Submodules

**LQR**: This repo contains the path tracking methodes including Pure pursuit, Stanley, LQR steering, LQR steering speed. 
LQR submodule is based on the [repository of the F1TENTH Gym environment](https://github.com/f1tenth/f1tenth_gym) and its corresponding [documentation](https://f1tenth-gym.readthedocs.io/en/latest/).

**f1tenth_traj_gen**: trajectory generation repo for solving min curvature QP with F1TENTH params. The dev is based on [TUM's global traj optim repo](https://github.com/TUMFTM/global_racetrajectory_optimization). 


## Wireless Visualization via Rviz2

Thanks to the [ESE 615 tips](https://docs.google.com/document/d/1PhaZvV0ZKzfTiwoJAoGcjTY9W2EPkMq2NKQgz8E-glk/edit)!
```bash
ifconfig  # check wlan0, should be same to the car's
echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc
sudo ufw disable  # disable the firewall
rviz2  # on your native ubuntu and add the topics, start rviz2 before pf!
```

