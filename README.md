# NIPS 2016
Autonomous exploration, active learning and human guidance with open-source Poppy humanoid robot platform and Explauto library.
The folder [ros](ros) contains a ROS package to be symlinked to your ROS workspace.

# Initial setup
## Raspberry Pi part
  - Install ROS Comm http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
  - Compile with `catkin_make_isolated -DPYTHON_EXECUTABLE=/home/poppy/miniconda/` 
  - clone https://github.com/ros/common_msgs.git
  - Set `export LC_ALL=C` in bashrc to prevent `terminate called after throwing an instance of 'std::runtime_error'   what():  locale::facet::_S_create_c_locale name not valid` errors 
  - Add the NIPS package to ros_ws/src now
  -  `catkin_make_isolated --pkg nips2016` 
  - `sudo apt-get install swig scons`
  - `pip install RPi.GPIO`
  - `cd ~/Repos && git clone https://github.com/ymollard/rpi_ws281x.git`
  - `scons && cd python && sudo python setup.py install`  (install in system dir, not in miniconda, as root will run this code)
  - Replace the home page for the Poppy monitor, the ROS node will serve its server


## Workstation setup
```
sudo apt-get install python-pyaudio
sudo pip install flask flask-cors
ln -s ~/NIPS2016/ros/workstation.sh ~/catkin_ws/fuzz.sh
```

# Launch the process
## 1. Start the Raspberry Pi part
```
# Login as root as a fix to be able to connect to the LED strip (RPi's PWM pin)
ssh root@fuzz.local -X
cd /home/poppy/ros_ws/
./fuzz.sh
roslaunch nips2016 raspberry_pi.launch light:=false
```
The `light` parameter tells wether we want to control WS2812B lights on GPIO18.
If successful, this command must show "Ergo is ready and starts joystick servoing..." and moving joysticks must move the Ergo Jr robot accordingly. You can then

## 2. Start the Workstation part
### 2.1 Setup th ROS environment
```
cd ~/catkin_ws/fuzz.sh
./fuzz.sh
```
The BASH script hereabove setups ROS vairable to use `fuzz.local` as the ROS master.
Then choose to start either the services/publishers OR the full experiment (services/publishers + controller) on your workstation, 3 or 3bis.

### 3. Services and publishers only
```
roslaunch nips2016 workstation.launch
```
You are then able to monitor topics and call services of all nodes running on the workstation.

Make sure the camera can visualize the arena properly and the ball and arena are properly tracked in OpenCV windows that are automatically opened. If not, you might tweak the H, S, V ranges in config file for environment recognition.

### 3bis. Full experiment: services, publishers and controller
This command requires that not `workstation.launch` is running, as it's already launching it by itself.
```
roslaunch nips2016 start.launch
```
You can also pass the following arguments:
 - `source:=string` that will be used as a source experiment in case of time travel
 - `name:=<string>` that will be used to output log files (a pickle file in `logs/`). Be careful that this name is prefixed by the timestamp before using it as a source file
 - `iterations:=<integer>` that will automatically stop the controller after a certain number of iteration (aka trials). If you're using a source file then `iterations` must be greater than the number of iterations recorded in the source file.
 
 For example:
```
roslaunch nips2016 start.launch name:=new_experiment old:=2016-11-29_14-48-12_old_experiment iterations:=6000
```

Robot immedialtely starts the requested number of iterations. Press Ctrl+C to interrupt and cause log saving.
### Results folder
Logs are saved on a regular basis and at shutdown (Ctrl+C).
This command will bring you to the folder containing recored experiments, if any:
```
roscd nips2016/logs
```
