

# ROS Turtle Rectangle Path Task

This task demonstrates control of a simulated turtle in a rectangular path using ROS (Robot Operating System), testing adaptability to new software stacks and programming fundamentals.

## About
This ROS package controls the movement of a bot inside the Turtlesim package. It allows the bot to perform various movements, including moving in a straight line, circular path, and now, a custom rectangular path.

## Requirements
- **Operating System**: Ubuntu 20.04 LTS
- **Software**: ROS (Robot Operating System) - ROS-Base version, ROS Turtlesim package

## Setup Instructions

### 1. Install Ubuntu 20.04 LTS
Objective: Install Ubuntu 20.04 LTS by either dual-booting your system or using a virtual machine like Oracle VirtualBox.

1. Download the Ubuntu 20.04 LTS ISO file from the [official Ubuntu website](https://ubuntu.com/download/desktop).
2. Choose between:
   - **Dual Boot**: Install Ubuntu alongside your current operating system.
   - **Virtual Machine**: Install [Oracle VirtualBox](https://www.virtualbox.org/) and create a virtual machine for Ubuntu.

### 2. Install ROS (Robot Operating System)
Objective: Set up ROS on the fresh Ubuntu installation.

1. Follow the official ROS installation guide for Ubuntu 20.04. Install the **ROS-Base** version.
2. Test your ROS installation by running the ROS master with:
   ```bash
   roscore
   ```

### 3. Install Turtlesim Package
Install the Turtlesim package to simulate the turtle’s movement:

```bash
sudo apt-get install ros-noetic-turtlesim
```

### 4. Clone and Build the ROS Package
1. **Set up a Catkin Workspace**:
   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   ```

2. **Clone the Package**:
   Clone this repository into your Catkin workspace:
   ```bash
   git clone https://github.com/topguns837/ros_session.git
   ```

3. **Compile the Code**:
   Navigate to the root of your workspace, compile the package, and source the workspace:
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

## ROS Turtlesim Task Instructions

Objective: Practice ROS concepts such as Packages, Nodes, Subscribers, Publishers, Messages, and Topics.

### Steps:
1. **Run ROS Core**:
   Open a new terminal and start the ROS core with:
   ```bash
   roscore
   ```

2. **Start Turtlesim Node**:
   In another terminal, launch the Turtlesim node:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

3. **Run the Movement Code**:
   In a third terminal, run the code to control the turtle’s movement:
   ```bash
   rosrun ros_session move.py
   ```

4. **Implement the Rectangle Path**:
   Modify the provided code to make the turtle move in a rectangular path.
   - Choose your own dimensions for the rectangle (length and breadth).

5. **Expected Outcome**:
   - The turtle should move in a rectangle similar to the example shown in this [video](https://www.loom.com/share/d6452d81716e404889dcef0bb3e0429d).

## Deliverables
1. **GitHub Repository**:
   - Create a public GitHub repository with your ROS package.
   - Share the repository link.

2. **Screen Recording**:
   - Record a screen video showing the turtle moving in a rectangular path.
   - Include a link to the video ([Loom video here](https://www.loom.com/share/d6452d81716e404889dcef0bb3e0429d)).
