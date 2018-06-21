# Airborne Wind Energy Simulation Platform for Rigid Wings - Gazebo repo

This repository contains the gazebo specific simulations files motivated to have an Airborne Windenergy System modelled in gazebo using PX4 software. The main improvement from the repository (PX4/sitl_gazebo) is the implemented tether model. Compared to the general approach for the PX4 gazebo modelling the .sdf.jinja format is used wich generates the desired .sdf model. Having a jinja generator for the model the tether parameters (eg. position, length, diameter, number of elements)  can easily be adapted to generate the desired .sdf output and no hardcoding is needes as macros had been implemented.

## Available models with tether are:
- solo_with_tether
- kitex_with_tether
![kitex_with_tether](https://user-images.githubusercontent.com/22919543/41724840-c13d949e-756e-11e8-9793-930c0dbce17f.png)

Check out the list of tweakable parameters right at the begining of the .sdf.jinja file. Possible parameters are at the moment: 
- number of elements
- length of the tether element [m]
- radius of the tether element [m]
- radius of the tether element visual [m ]
- mass of the element [kg], given the length
- the ratio of the drag coefficient
- physical properties between two tether elements (as they are connected through joints, see setup http://sdformat.org/spec?ver=1.6&elem=joint):
	- damping
	- friction 
	- spring stiffness
	- spring refference
- anchor point (x,y)
- tether attachment point
- distance of Y-element from body of aircraft

## Current status
- 21.06.18 functioning tether model, at the moment in the process of integration with the flight controller from KiteX for the powerproduction
- 02.04.18 merged PX4/master into master_rw_github (hash: 28921bca)
- 26.03.18 tether development started

## How to integrate the sitl_gazebo with your own PX4
This repository only provides you with the model of the tether and the aircrafts. If you want to test your own flightcontroller within your own PX4 flight stack, please follow the next steps: 

### Adding a new remote 
1. go to your PX4 repo (called PX4_Firmware)
```bash
cd PX4_Firmware/Tools/sitl_gazebo
```
2. add the RigidWing gazebo repo as a remote
```bash
git remote add <name_of_remote> git@github.com:RigidWing/sitl_gazebo.git
```
3. checkout the the master branch from the rigid wing and make a new branch 
```bash
git fetch <name_of_remote>
git checkout master_rw_github
git checkout -b <own_branch>
```
4. As a submodule has now been changed within the PX4 repo, the subproject commit needs to be updated. Thus commit within PX4_Firmware folder the updated submodule (its needed otherwise it checks out a wrong branch after you do a submodule update recursively). 

**Important to know**: An airframe model first needs to be defined and implemented first as a basic .sdf file (eg. for the solo there existis the solo and the solo_with_tether model). The solo_with_tether model links the tether to the solo (includes the solo.sdf as a basic model within the jinja file). Thus as a first step you need to know how to create a new model and in a second step you need to know how to link tether and airframe model together.


### Create a new model
1. Make a new world:
```
PX4_Firmware/Tools/sitl_gazebo/worlds/new_model.world
```
2. make a new model (and add an model.config and new_model_with_tether.sdf.jinja file):
```
PX4_Firmware/Tools/sitl_gazebo/models/new_model_with_tether
```
3. make a init file (based on existing examples in that dir):
```
PX4_Firmware/posix-configs/SITL/init/ekf2/new_mod
```
4. make a separate mixer file (open issue), as the index for the mixer files within gazebo starts at 0, and the index for the  PX4 mixer files starts at 1.
```
PX4_Firmware/ROMFS/sitl/mixer/new_model_with_tether.main.mix
```
5. add the model as a gazebo make target:
```
PX4_Firmware/src/firmware/posix/sitl_target.cmake @line:83
```

**Tipp**: grep for the solo or standard_vtol to get to known to the structur how to add a new model and a world (everything is related to the make target name) 


### Link the tether to any model  

1. If you have followed all the steps above you can now link the tether to your model
2. Just copy the whole content of the solo_with_tether.sdf.jinja file in your new_model_with_tether.sdf.jinja
3. Choose either your own model or choose an already existing model to which the tether can be attached to. For this you only need to change three lines in your code. (In this example the solo model had been replaced with the standard_vtol) . 
```
@line 251: <uri>model://standard_vtol</uri> 
@line 271: <child>solo_left</child>
	     <parent>standard_vtol::base_link</parent>
@line 290: child>solo_right</child>
	      <parent>standard_vtol::base_link</parent>
```
4. Beside of the desired tether parameters be aware to specify the position of the tether attachment point. This can be done by giving the x, y, z coordinate of one tether attachment point (vehicle_attach_x,...), the second one is mirrored. 

7. finaly build and run the simulation. This should build the tether but also the airplane model linked to it as descriped in step 3 (you will within your new_model_with_tether folder a new .sdf file which includes the complete sdf description for tether and airframe generated by the jinja builder).
```bash
make posix_sitl_default gazebo_new_model_with_tether
```

**Important to know**: The model is build up from the anchor point of the tether. Also be aware that your own repo should already contain the jinja builder (later than March 2018). Otherwise update it. 

For any questions  or if you want to contribute please conntact: info@kitepower.nl

# README FROM PX4/sitl_gazebo repo: 
# Gazebo for MAVLink SITL and HITL
##### upstream  [![Upstream Master Build Status](https://travis-ci.org/PX4/sitl_gazebo.svg?branch=master)](https://travis-ci.org/PX4/sitl_gazebo)
##### master  [![Master Build Status](https://travis-ci.org/RigidWing/sitl_gazebo.svg?branch=master_rw_github)](https://travis-ci.org/RigidWing/sitl_gazebo)

This is a flight simulator for multirotors, VTOL and fixed wing. It uses the motor model and other pieces from the RotorS simulator, but in contrast to RotorS has no dependency on ROS. This repository is in the process of being re-integrated into RotorS, which then will support ROS and MAVLink as transport options: https://github.com/ethz-asl/rotors_simulator


**If you use this simulator in academic work, please cite RotorS as per the README in the above link.**

## Install Gazebo Simulator

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo. Mac OS and Linux users should install Gazebo 7.


## Protobuf

Install the protobuf library, which is used as interface to Gazebo.

### Ubuntu Linux

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev \
			gazebo7 libgazebo7-dev libxml2-utils python-rospkg python-jinja2
```

### Mac OS

```bash
pip install rospkg jinja2
brew install graphviz libxml2 sdformat3 eigen opencv
brew install gazebo7
```

An older version of protobuf (`< 3.0.0`) is required on Mac OS:

```bash
brew tap homebrew/versions
brew install homebrew/versions/protobuf260
```

## Build Gazebo Plugins (all operating systems)

Clone the gazebo plugins repository to your computer. IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.

```bash
mkdir -p ~/src
cd src
git clone https://github.com/Dronecode/sitl_gazebo.git
```

Create a build folder in the top level of your repository:

```bash
mkdir Build
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:


```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/Build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

You also need to add the the root location of this repository, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:
```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
cd Build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make
```

### GStreamer Support
If you want support for the GStreamer camera plugin, make sure to install
GStreamer before running `cmake`. Eg. on Ubuntu with:
```
sudo apt-get install gstreamer1.0-* libgstreamer1.0-*
```

### Geotagging Plugin
If you want to use the geotagging plugin, make sure you have `exiftool`
installed on your system. On Ubuntu it can be installed with:
```
sudo apt-get install libimage-exiftool-perl
```

## Install

If you wish the libraries and models to be usable anywhere on your system without
specifying th paths, install as shown below.

**Note: If you are using ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```

## Testing

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
. /usr/share/gazebo/setup.sh
. /usr/share/mavlink_sitl_gazebo/setup.sh
gazebo worlds/iris.world
```

Please refer to the documentation of the particular flight stack how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)

## Packaging

### Deb

To create a debian package for ubuntu and install it to your system.

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
