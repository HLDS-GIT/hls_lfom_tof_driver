# hls_lfom_tof_driver

ROS package for HLDS HLS-LFOM 3D LiDAR(TOF)

ROS Wiki: [hls_lfom_tof_driver](http://wiki.ros.org/hls_lfom_tof_driver)

Manufacturer of the TOF: [Hitachi-LG Data Storage,Inc.](https://hitachi-lg.com/)

Product Webpage: [HLDS 3D LiDAR(TOF)](http://hlds.co.jp/product-eng)

# Overview

* Object distance measurement with high accuracy in realtime  

* Easy to install with Ethernet (POE+) connection  

* Multiple use cases can be accommondated with the SDK  

![HLS-LFOM](http://i1.wp.com/hlds.co.jp/product-eng/wp-content/uploads/2016/11/hlds_top02.png?w=500)


# User's Guide

## Requirements
Ubuntu 16.04 Desktop Linux  
ROS Kinetic  
HLDS HLS-LFOM SDK 2.1.0 or later 

## Install
* Install ROS Kinetic  

* Download HLS-LFOM SDK latest version(v2.1.0 or later) for Ubuntu 16.04 LTS (x64)  and extract it(http://hlds.co.jp/product-eng/tofsdk)  

* Install HLS-LFOM SDK  
```
  $ sudo dpkg -i libtof-dev_<version_number>ubuntu16_amd64.deb  
```

* Build the HLS-LFOM TOF driver  
```
  $ cd ~/catkin_ws/src  
  $ git clone https://github.com/HLDS-GIT/hls_lfom_tof_driver.git  
  $ cd ~/catkin_ws  
  $ catkin_make  
```
If using SDK older than v2.3, please use CMakeLists_old.txt.

## Setting IP address
Modify IP address of tof.ini file in launch folder. It should be the same as the IP address of the TOF sensor you are trying to connect to. (Default is 192.168.0.105)  
If you want to change IP address, please refer to Setup TOF sensor of API Reference Manual in SDK.  


## Run hlds_3dtof_node
```
roslaunch hls_lfom_tof_driver hlds_tof_driver.launch
```

## Run hlds_3dtof_node with RViz
```
roslaunch hls_lfom_tof_driver view_hlds_tof_driver.launch
```

