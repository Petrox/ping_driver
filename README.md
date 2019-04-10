# ping_driver

This repository holds the driver to interface Ohio State's Underwater Robotics Team's Systems with the Blue Robotics Ping device.

In short, this connects to the Ping and outputs the Ping's "distance" and "confidence" measurements to the topic "/ping/raw". If anything other than distance and confidence is needed, the code is heavily documented and organized. It shouldn't be a difficult addition. 

The driver also optionally supports fake data simulation via Python's pseudoterminal functionality in order to test the driver without having access to the Ping itself. To enable this, change the "readingFromFakeStream" boolean variable to True. The class should automatically readjust, though this function wasn't extensively tested.

## Hardware Setup 

### Wiring Setup

Picture of the correct wiring setup: https://drive.google.com/open?id=1Q1uSh616rQzBKoS8JJXwo_EBXh4CzepP

Technical Wiring Document: <I found this once, but I can't find it again. I know it exists though.>

Also see https://www.bluerobotics.com/store/sensors-sonars-cameras/sonar/ping-sonar-r2-rp/, which (under the technical details tab) outlines which wires go where. The White and Green wires go in the opposite ports as outlined on that page, though. 

#### Wire Connection: 

Black Wire -> GND (Ground) 

Red Wire -> VIN (Voltage In) 

Green Wire -> TX 

White Wire -> RX 

## Software Setup

### Requirements / Dependencies

You will need Blue Robotics's Ping Library (Python Version). Instructions for install can be fouund at https://pypi.org/project/bluerobotics-ping. Our group used pip to install the package, but many options are available. 

TODO: Put offline install instructions here 

You will obviously also need ROS. Note that Python is used here, and not C++. You shouldn't need to make any modifications to accomodate this; It should be dealt with by the CMake, which imports necessary stuff, basically. 



### Setup With ROS

To run this node, you first have to set up a ROS workspace, which we named ping_ws. The name does not affect compilation, but is for organizationalurposes. Inside that workspace, you can then create a src folder. Inside that src folder, you can clone this repository. The end heirarchy will look like this: 

ping_ws > src (and build, devel after running catkin_make) > ping_driver (this repo)

If ROS is not recongizing the package If you end up needing to run "source devel/setup.bash" in order to use rosrun and rostopic echo, you need to add "source ~/osu-uwrt/ping-ws/devel/setup.bash" to your .bashrc file. The file path should be replaced with whatever your path is to devel, though it should be the same as mine.

## Usage

"catkin_make" should be run in ping_ws in order to compile everything. 

"rosrun ping_driver pingDriver.py" should be used to run the node itself. This will run the file and start publishing data.

"rostopic echo /ping/raw" should be used to view the data being published on the topic. 

#### Offline Install - Overview

If your robot doesn't have internet access, you naturally can't use Pip to install packages from the internet. This functionality is still being developed, but in short, you can download the packages themselves for usage offline. You can then compress the entire package, transfer it to the robot, then unzip it for easy usage. 

#### Offline Install - Technical Instructions 

The modules themselves are actually a part of this repository and are not .gitignored, meaning this section is actually already done. However, if you're having issues, the following few paragraphs detail how to install your packages to the packages folder instead of user-wide. 

Again, this file is already in the repo, but you need to set up a requirements.txt file first so Python knows which modules exactly it needs. This file should be created in the root of your repository, and should have the following contents: 

```
System==0.1.14
catkin_pkg==0.4.11
logger==1.4
repr==0.3.1
unittest2==1.1.0
bluerobotics-ping==0.0.7
```

Afterward, run the following command to install the packages to the packages/ folder (relative to where you currently are, which should be the root of this repository): 

```
pip install -r requirements.txt --system -t packages/
```

The --system command is only necessary due to a Ubuntu 16.04-specific bug, from what I can tell. All I know is that it's necessary for OSU's software setup. 

To sum up exactly what this command is doing, it is installing the packages mandated in requirements.txt to the packages/ folder. The -t command is short for "target" and specifies that you want to install them to a specific folder, rather than user-wide. 

From there, the driver itself is set up to automatically find these modules and import them correctly. What this means is that src/pingDriver.py automatically includes the "packages" folder into the PYTHONPATH during node initialization so that Python knows to look for the modules there. 

## Debug / Common Issues

#### ROS Not Recognizing Ping_Driver As A Package

If ROS is not recognizing ping_driver as a package when you try to use rosrun, you need to add the line "source ~/osu-uwrt/ping_ws/devel/setup.bash" to your .bashrc folder.  Your file path could be different, but shouldn't be. You can alternatively run "source devel/setup.bash" in each terminal you use when testing, but adding it to the .bashrc removes this need, as it does it automatically in each terminal you open. 

#### Permissions Issues 

If you are on Linux and you get a permissions error when attempting to open a port, you probably need to add your account to the dialout group. Google this for a guide, but you basically have to run a single command then restart your computer, which fixed the issue for me. 

Related to permissions issues in general, if you're ever getting a permissions issue, you may have to chmod the file. Google this for a guide, but I believe the following line clears up any permissions issues: 

```
chmod a+x <file name>
```

I'm still kind of sketchy on the command itself, but the gist is that you're modifying the file's permissions level. By default, it may not have read or execute permissions (or whatever else it needs - Again, I'm sketchy on the details). 

#### Ping Issues

If you get an error about failing to initialize the ping, or failing to find a device on that serial port, please check the serial port that you're plugging the ping into. If that doesn't work, you can change the port in the Ping1D constructor found in the code. 

There are commands that you can find on the internet that will find which ports are currently being used. There's probably a more efficient way to do this, but you can use that to find the correct port. 




