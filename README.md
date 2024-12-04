# **Unity-Frank ROS Control Workspace**

This repository integrates Unity, ROS, and Franka Panda robot arms, including the FR3 model, for simulation and real-world robot control. It supports VR hand-tracking controllers and includes autonomous and manual control interfaces.

---

## **Requirements**

### **Software and Environment**
- **WSL 1 Setup**: Ubuntu 20.04
- **Unity Package**: Run this alongside Unity VR Repository Instructions here: https://github.com/zoujoey/VR_Arm_Control_Interface.git
- **FrankaROS**: Build FrankaROS from source to support the FR3 model.

---

## **Setup**

### **FrankaROS Commands**
#### **Real Robot Setup (External Linux Computer)**  
1. Set up ROS environment variables:
   ```bash
   export ROS_MASTER_URI=http://192.168.2.1:11311/
   export ROS_IP=192.168.2.1
   ```

2. In WSL1, configure as follows:
   ```bash
   wsl -d Ubuntu-20.04
   export ROS_IP=192.168.2.5
   export ROS_MASTER_URI=http://192.168.2.1:11311/
   ```

3. Verify connection:
   ```bash
   rostopic pub /chatter std_msgs/String "data: 'Hello, world'" -r 1
   ```

#### **Gazebo Simulation Setup**
1. Set up the ROS environment variables:
   ```bash
   export ROS_IP=0.0.0.0
   export ROS_MASTER_URI=http://127.0.0.1:11311/
   export DISPLAY=localhost:0
   ```

2. Launch the simulation with the appropriate file.

---

### **Launch Files**
#### **Unity Control**
```bash
roslaunch ROS_Arm_Control_Interface unity_controller_interface.launch
```

#### **Autonomous Control**
```bash
roslaunch ROS_Arm_Control_Interface base_controller_test.launch
```

---

## **Simulation Commands**

### **Panda Simulation**
```bash
roslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find franka_gazebo)/world/stone.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true interactive_marker:=false
```

### **FR3 Simulation**
1. Generate the URDF:
   ```bash
   xacro $(rospack find franka_description)/robots/fr3/fr3.urdf.xacro gazebo:=true
   ```

2. Launch the simulation:
   ```bash
   roslaunch franka_gazebo fr3.launch x:=-0.5 \
       world:=$(rospack find franka_gazebo)/world/stone.sdf \
       controller:=cartesian_impedance_example_controller \
       rviz:=false interactive_marker:=false
   ```

3. Alternative launch without a custom world:
   ```bash
   roslaunch franka_gazebo fr3.launch x:=-0.5 \
       controller:=cartesian_impedance_example_controller \
       rviz:=false interactive_marker:=false
   ```

---

## **VR Hand-Tracking Controller Setup**

1. Navigate to the VR control workspace:
   ```bash
   cd ~/STAR_Labs/VR_Unity_franka_control_ws/
   source devel/setup.bash
   ```

2. Run the relevant ROS script:
   ```bash
   rosrun ROS_Arm_Control_Interface unity_control.py  # Unity Controller
   rosrun ROS_Arm_Control_Interface basic_move_stone.py  # Stone Controller
   rosrun ROS_Arm_Control_Interface unity_control.py  # Controller Basic Testing
   ```

---

## **Usage**
### **Real Robot Control**
- Follow the commands in the **FrankaROS Commands** section to connect and verify your ROS network.
- Use Unity scripts for VR control as needed.

### **Simulation**
- Choose between Panda or FR3 simulation setups depending on your model requirements.

---

## **Troubleshooting**
- Ensure your IP configuration matches the ROS environment.
- Verify network communication using:
  ```bash
  rostopic echo /chatter
  ```
- For Gazebo simulations, ensure `DISPLAY` is set correctly for GUI rendering.

---

## **License**
TBD

---
