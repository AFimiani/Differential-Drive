Differential-Drive Robot --- Technical project #2
====================================

This repository comprises code for trajectory planning, localization, and control of a differential drive robot.

The differential drive's URDF used in the package has been created modifying the URDF taken from the following repository 
> "roomba_robot by goncabrita", [Link](https://github.com/goncabrita/roomba_robot)
## Simulation video 
To download the video of the complete exploration:
>  [Link to the simulation video](https://drive.google.com/file/d/1Dw4lYnRBIIyTnJCxfjGyyukD3-S18ccd/view?usp=sharing)
## Requirements
#### Gmapping
To save the map inside the simulation is needed Gmapping, to install:
```
$ sudo apt-get install ros-noetic-gmapping
```
#### Eigen
For the computation of matrix equations is needed Eigen:
```
$ sudo apt-get install libeigen3-dev 
```

#### OpenCV and Zbar
Needed for camera vision and markers detection
```
$ sudo apt install python3-opencv
```
and
```
$ sudo apt-get install libzbar-dev
```

## Installation

 1. Clone this repository inside the ROS workspace
 2. Compile using catkin_make

## Usage

To run the simulation launch the *final.launch* file using
```
$ roslaunch progetto_esame final.launch
```
Once the robot returns to its starting position the map is automatically saved.

To get information on the task status use 
```
$ rostopic echo /my_robot/task_status
```



## Test other cases

 To change the room order of exploration, in the *planner_node.cpp* file inside the `src/` folder , modify the assignment of the *_rooms* variable in the class constructor
 
	ArtificialPotentialPlanner::ArtificialPotentialPlanner(double  x0, double  y0, double  yaw0){
	..
	//------- CASO 1
	_rooms << 	11, 6, // Room 3 >Room 2 >Room 1 >Start
			7,11,
			3,8,
			x0,y0;
	..
	}
 To change the starting position, in the *final.launch* file, inside the `launch/` folder,  change the value of the args *x0*,  *y0*, *yaw0* 
 
	<arg  name="x0"  value="5.5"  />
	<arg  name="y0"  value="1.5"  />
	<arg  name="yaw0"  value="1.570796327"  />

To change the desired id, modify the value of  *id_wanted* parameter in the *final.launch* file inside the `launch/` folder 
	
	<param  name="id_wanted"  value="id1"  />
	
## Create other QR code
Inside the repository is added a package taken from GitHub useful to build QR code. The package is 
> "cpp-qr-to-png by RaymiiOrg", [Link](https://github.com/RaymiiOrg/cpp-qr-to-png)

To build this package :

	cd progetto_esame/cpp-qr-to-png-master
	mkdir build
	cd build
	cmake ..
	make all

The file in which is possible to assign new data to create qr code is the main.cpp inside the `cpp-qr-to-png-master/src/` folder, modify the value of 

	std::string  qrText = "id1";
	std::string  fileName = "QR_1.png";
once modified the desried value,  in `cpp-qr-to-png-master/build/src/` folder use 

	sudo ./qr-to-png 

 Then add the QRcode on the simulation by copying and modifying the files inside  `marker_1/` folder, in the `model/` folder 

 **Please note:** as a design hypothesis, the data of the qr code must start with "*id*" to be recognized by the robot 

