# hls_lfom_tof_driver

ROS package for HLDS HLS-LFOM 3D LiDAR(TOF)

ROS Wiki: [hls_lfom_tof_driver](http://wiki.ros.org/hls_lfom_tof_driver)

Manufacturer of the TOF: [Hitachi-LG Data Storage,Inc.](http://hlds.co.kr/v2/e_index.html)

Product Webpage: [HLDS 3D LiDAR(TOF)](http://hlds.co.jp/product-eng)

# Overview

* Conducts real-time sensing of the distance object image to obtain 3-dimensional data  

* 3D data and QVGA IR image can be simultaneously outputted  

* Light Emitting part: Near Infrared LD, Light Receiving Part: CCD image sensor  

![HLS-LFOM](http://i1.wp.com/hlds.co.jp/product-eng/wp-content/uploads/2016/11/hlds_top02.png?w=500)


# User's Guide

## Requirements
Ubuntu 16.04 Desktop Linux  
ROS Kinetic  
HLDS HLS-LFOM SDK 2.0.0 or later 

## Install
* Install ROS Kinetic  

* Install cmake and OpenCV library  

* Download HLS-LFOM SDK for Ubuntu 16.04 and extract it(http://hlds.co.jp/product-eng/tofsdk)  

* Install HLS-LFOM SDK  
```
  $ sudo dpkg -i libtof-dev_<version_number>ubuntu16_amd64.deb  
```

* Install the HLS-LFOM TOF driver  
```
  $ cd ~/catkin_ws/src  
  $ git clone https://github.com/HLDS-GIT/hls_lfom_tof_driver.git  
  $ cd ~/catkin_ws  
  $ catkin_make  
```

## Run hlds_3dtof node
```
roslaunch hls_lfom_tof_driver hlds_3dtof.launch
```

## Run hlds_3dtof node with RViz
```
roslaunch hls_lfom_tof_driver view_hlds_3dtof.launch
```

