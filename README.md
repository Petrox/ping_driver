# ping_driver

This repository holds drivers to interface Ohio State's Underwater Robotics Team's Systems with the Blue Robotics Ping device.

## Requirements / Dependencies

You will need Blue Robotics's Ping Library (Python Version). Instructions for install can be fouund at https://pypi.org/project/bluerobotics-ping. Our group used pip to install the package, but many options are available. 

You will obviously also need ROS. Note that Python is used here, and not C++. You shouldn't need to make any modifications to accomodate this; It should be dealt with by the CMake, which imports necessary stuff, basically. 

## Setup

To run this node, you first have to set up a ROS workspace, which we named ping_ws. The name does not affect compilation, but is for organizationalurposes. Inside that workspace, you can then create a src folder. Inside that src folder, you can clone this repository. The end heirarchy will look like this: 

ping_ws > src (and build, devel after running catkin_make) > ping_driver (this repo)

If ROS is not recongizing the package If you end up needing to run "source devel/setup.bash" in order to use rosrun and rostopic echo, you need to add "source ~/osu-uwrt/ping-ws/devel/setup.bash" to your .bashrc file. The file path should be replaced with whatever your path is to devel, though it should be the same as mine.

## Wiring Setup

Picture of the wiring setup: https://drive.google.com/open?id=1Q1uSh616rQzBKoS8JJXwo_EBXh4CzepP

Also see https://www.bluerobotics.com/store/sensors-sonars-cameras/sonar/ping-sonar-r2-rp/, which (under the technical details tab) outlines which wires go where. The White and Green wires are actually switched due to some weird stuff.

Technical Wiring Document: <I found this once, but I can't find it again. I know it exists though.>

#### Which Wires Go Where?

Black -> GND (Ground) 

Red -> VIN (Voltage In) 

Green -> TX 

White -> RX 

## Usage

"catkin_make" should be run in ping_ws in order to compile everything. 

"rosrun ping_driver pingDriver.py" should be used to run the node itself. This will run the file and start publishing data.

"rostopic echo /ping/raw" should be used to view the data being published on the topic. 

## Debug / Common Issues

If ROS is not recognizing ping_driver as a package when you try to use rosrun, you need to add the line "source ~/osu-uwrt/ping_ws/devel/setup.bash" to your .bashrc folder.  Your file path could be different, but shouldn't be. You can alternatively run "source devel/setup.bash" in each terminal you use when testing, but adding it to the .bashrc removes this need, as it does it automatically in each terminal you open. 

If you are on Linux and you get a permissions error, you need to add your account to the dialout group. Google this for a guide, but you basically have to run a single command then restart your computer.

If you get an error about failing to initialize the ping, or failing to find a device on that serial port, please check the serial port that you're plugging the ping into. If that doesn't work, you can change the port in the Ping1D constructor found in the code. 

If you get a speed of sound in the medium error, that's likely also the driver detecting the ping as being in the wrong port, or a faulty connection or something like that. Odds are, if it doesn't give you an initialization error, it won't give you a speed of sound error either, though the check is there for speed of sound regardless. 




