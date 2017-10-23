# HTC Vive teleop stuff

## Install
Get my fork of pyopenvr and install it 
(it contains a workaround to be able to read the controller's
state correctly, only necessary until [PR #42](https://github.com/cmbruns/pyopenvr/pull/42 is merged).
```bash
git clone https://github.com/awesomebytes/pyopenvr
cd pyopenvr
sudo python setup.py install
```

To run my node you'll need to have installed basic ROS TF stuff:

```bash
# In case you are missing it
sudo apt-get install ros-kinetic-tf ros-kinetic-tf2*

```

Then check out this repository in your catkin workspace:
```bash
mkdir -p vive_ws/src
cd vive_ws/src
git clone 


```

## Run node




## To run PR2 Teleop demo

sudo apt-get install libnlopt-dev
sudo apt-get install ros-kinetic-moveit-ros
sudo apt-get install swig

git clone https://bitbucket.org/awesomebytes/trac_ik
cd trac_ik
git checkout python_wrapper_swig_based

cd ../..
catkin_make



