

# ROS Turtle Rectangle Path Task

This task demonstrates control of a simulated turtle in a rectangular path using ROS (Robot Operating System), testing adaptability to new software stacks and programming fundamentals.

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

### 3. ROS Turtlesim Task
Objective: Practice ROS concepts such as Packages, Nodes, Subscribers, Publishers, Messages, and Topics.

#### Steps:
1. **Install the ROS Turtlesim Package**:
   - Follow the instructions [here](https://wiki.ros.org/turtlesim) to install Turtlesim.

2. **Clone the Provided Package**:
   - Clone the ROS package from the public repository [here](Link to public repository).

3. **Run the Turtlesim Package**:
   - The package includes code to make the turtle move in a straight line and a circular trajectory.

4. **Implement the Rectangle Path**:
   - Modify the code to make the turtle move in a rectangular path.
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
