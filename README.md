# Cart-Pole Co-simulation
Cart-Pole Matlab & ROS/Gazebo Co-simulation framework developed by erc-dynamics.

This repository contains a co-simulation project that features a cart-pole robot controlled by an LQR controller. The objective is to offer a simulation framework suitable for reinforcement learning and optimization research.

## Setup
This setup utilizes two computers: 
- one running Ubuntu 20.04 and the other running Windows. The Ubuntu machine operates ROS Noetic and Gazebo, handling real-time simulation. 
- The Windows computer is required to run a version of MATLAB that includes the ROS Toolbox. The goal is to simplify the complexities of ROS and Linux, while offering a real-time simulation environment that closely mirrors real-world applications.
- To make this setup function, both computers need to be connected to a network. Ensure that these two machines can interact and communicate with each other over the network.

## Installation (Ubuntu 20.04 machine)

The Ubuntu computer uses ROS for robot simulation in the Gazebo setting. We highly recommend using the ROS Noetic version that was originally used for this project's development.
- `ROS Noetic Desktop-Full Install`: To install ROS Noetic please follow the instructions at [installation](https://wiki.ros.org/noetic/Installation/Ubuntu) document.
- Create a [catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), follow the instructions.

## Installation (Windows machine)
The Windows machine runs Matlab, which needs to interface with ROS.
- Matlab: Ensure that your Matlab has the [ROS Toolbox](https://www.mathworks.com/products/ros.html) installed.
- Later, we will have to generate a Matlab custom message for ROS.  Based on the Matlab documentation provided [here](https://www.mathworks.com/support/requirements/supported-compilers.html), your machine should have one of the following installed to generate a ROS custom message: `Microsoft Visual C++ 2022 product family`, `Microsoft Visual C++ 2019 product family` or `Microsoft Visual C++ 2017 product family`. You can download `Microsoft Visual C++ 2017 product family` from this [link](https://learn.microsoft.com/en-ca/visualstudio/releasenotes/vs2017-relnotes). Links for other Visual Studio versions are readily available online.

## Build (Ubuntu 20.04)
Clone this repository inside `src/` folder of your ROS workspace. A recommended directory structure would look like so:
```bash
├── catkin_ws/
│   ├── src/
│   │   └── cart_pole/
│   ├── devel/
│   └── build/
```
## License and Usage
This co-simulation framework is developed in the erc-dynamics Lab. Please use wisely, and recommend improvements!
If you use this co-simulation for your work, please cite the ISAS 2023 paper that describes it in full detail.
