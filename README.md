# README #

[![Build Status](https://ci.dsor.isr.tecnico.ulisboa.pt/buildStatus/icon?job=GitHub+DSOR%2Ffarol_gazebo%2Fmain)](https://ci.dsor.isr.tecnico.ulisboa.pt/job/GitHub%20DSOR/job/farol_gazebo/job/main/)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/dsor-isr/farol_gazebo/main)
![GitHub contributors](https://img.shields.io/github/contributors/dsor-isr/farol_gazebo)
[![GitHub issues](https://img.shields.io/github/issues/dsor-isr/farol_gazebo)](https://github.com/dsor-isr/farol_gazebo/issues)
[![GitHub forks](https://img.shields.io/github/forks/dsor-isr/farol_gazebo)](https://github.com/dsor-isr/farol_gazebo/network)
[![GitHub stars](https://img.shields.io/github/stars/dsor-isr/farol_gazebo)](https://github.com/dsor-isr/farol_gazebo/stargazers)
[![GitHub license](https://img.shields.io/github/license/dsor-isr/farol_gazebo)](https://github.com/dsor-isr/farol_gazebo/blob/main/LICENSE)

### Objectives

The objectives of this environment are:
* Visual, physical and (hydro)dynamic models of generalized vehicle, manipulator and sensor elements.
* Simulation of sensing specific to underwater robotics including perception (e.g., sonar, underwater lidar and optical imaging) and navigation (e.g., DVL and USBL). 
* Parameterized representations of the ocean environment including seafloor bathymetry and ocean currents.
* Allow the incorporation and simulations done by the people of [DSOR](http://dsor.isr.ist.utl.pt/) but with the mindset of keeping it available to external people as well.
* Refine the requirements of [DSOR](http://dsor.isr.ist.utl.pt/) regarding the simulation and validation of marine robotic systems.

### Documentation
https://dsor-isr.github.io/farol_gazebo

### Environment Setup
If running the farol_gazebo without the farol stack, then the dsor_utils repository is required. You can clone it manually at:
```
https://github.com/dsor-isr/dsor_utils
```

### Hardware

* A modern multi-core CPU, e.g. Intel Core i5
* 8 Gb of RAM
* A discrete Graphics Card, e.g. Nvidia GTX 650
    * The environment can be run without a GPU, but the Gazebo simulation will run much faster (should run in real-time) with access to a GPU. Without a GPU the simulation is likely to run slower than real-time.

### Software
 - Recommended
   * Ubuntu Desktop 20.04 Focal (64-bit) 
   * Gazebo 11
   * ROS Noetic

### Original Code

The **Unmanned Underwater Vehicle Simulator** (UUV Simulator) is a set of packages that include plugins and ROS applications that allow simulation of underwater vehicles in [Gazebo](http://gazebosim.org/). 

Although the UUV Simulator has been released for ROS `Kinetic`, `Lunar` and `Melodic`, the current environment makes use of a UUV Simulator for the ROS `Noetic` distribution.

If you are using this simulator for your publication, please cite:

```
@inproceedings{Manhaes_2016,
	doi = {10.1109/oceans.2016.7761080},
	url = {https://doi.org/10.1109%2Foceans.2016.7761080},
	year = 2016,
	month = {sep},
	publisher = {{IEEE}},
	author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
	title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
	booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
```

### Nvidia Driver
We assume you have an Nvidia graphics card configured to use the proprietary driver. There are many online guides for configuring the Nvidia driver correctly on Ubuntu. 

If your computer does not support GPU ray-tracing, go to ```uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sonar_snippets.xacro```  and replace the lines:

```html
<sensor type="gpu_ray" name="sonar${suffix}"> 
<plugin name="sonar${suffix}_controller" filename="libgazebo_ros_gpu_laser.so">
<!-- <plugin name="switchable_sonar{suffix}_ros_interface" filename="libuuv_gazebo_ros_switchable_gpu_ray_sensor.so">
```

by

```html
<sensor type="ray" name="sonar${suffix}">
<plugin name="sonar${suffix}_controller" filename="libgazebo_ros_laser.so">
<!-- <plugin name="switchable_sonar{suffix}_ros_interface" filename="libuuv_gazebo_ros_switchable_ray_sensor.so">
```

### License
FarolGazebo is open-sourced under the MIT license. See the [LICENSE](https://github.com/dsor-isr/farol_gazebo/LICENSE) file for details.
