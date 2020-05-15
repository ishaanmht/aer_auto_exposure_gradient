# aer_auto_exposure_gradient

## Description
This package is our implementation of Gradient based auto exposure by [Shim et al. 2018](https://ieeexplore.ieee.org/document/8379436). We have extended this algorithm to include rules for gain compensation and tested it on self driving car [Zeus.](https://www.autodrive.utoronto.ca/) We have subimitted our results to [17th Conference on Computer and Robot Vision](http://www.computerrobotvision.org/). This package interfaces with Blackfly S cameras (model number: BFSU3-51S5C-C) equipped with Sony IMX250 CMOS global shutter sensors with a 2448 by 2048 resolution and a 70.74 dB dynamic range.
## Installation 

This package was written for Black Fly series cameras. The setup guidlines for the camera drivers can be found [here](https://flir.app.boxcn.net/v/SpinnakerSDK) and in order to setup ros interface for the cameras use [this](https://github.com/ros-drivers/flir_camera_driver).
In order to install this package simply clone the repo and do `catkin build`. Note: This package was developed and tested on ROS Kinetic.

## Usage
In order to use the package follow these instructions:

1. Update the config file with the relevant parameters like the name of image topic, frame rate etc.
2. Launch the ros camera driver in order to get images from your camera.
3. Then use `roslaunch aer_auto_exposure_gradient exp_node.launch`.

## Video

[![](http://img.youtube.com/vi/vGS4-n6Pf30/0.jpg)](http://www.youtube.com/watch?v=vGS4-n6Pf30 "Video")


