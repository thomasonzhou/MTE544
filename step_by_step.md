# Lab 1

https://github.com/thomasonzhou/MTE544/tree/labOne?tab=readme-ov-file

## Teleoperation (Parts 1 and 2)

Connect to the robot: https://github.com/UW-MTE544/MTE544_student/blob/main/connectToUWtb4s.md

Checklist:
- fastDDS file
- connected to VPN
- ros2 topic list

```sh
CURR_ROS_DOMAIN_ID=<REPLACE>
cat << 'EOF' >> ~/.tb4_env
source ~/robohub/turtlebot4/configs/.bashrc
export ROS_DOMAIN_ID=$CURR_ROS_DOMAIN_ID
EOF
echo "source ~/.tb4_env" >> ~/.bashrc
```
