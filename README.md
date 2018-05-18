# NIPS 2016
Autonomous exploration, active learning and human guidance with open-source Poppy humanoid robot platform and Explauto library.
The folder [ros](ros) contains a ROS package to be symlinked to your ROS workspace.

# Initial setup
## Raspberry Pi part

⚠️ **You can directly download the preconfigured SD card image for the Raspberry Pi [HERE](https://github.com/sebastien-forestier/NIPS2016/releases/tag/v0.1).** Or you can follow the instructions bellow to build your own image.

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

### Wiring
  - For the use of lights, use pin 12 (GPIO18). It is recommended to power the WS2812B lights or similar with a external source.
  - For the use of the button, use pin 40 (GPIO21). Make sure the button is set to activate on being pushed.
  - For the use of the joysticks, make sure both joysticks are plugged in before starting the processes.  

## Workstation setup
```
sudo apt-get install python-pyaudio
sudo pip install flask flask-cors
ln -s ~/NIPS2016/ros/workstation.sh ~/catkin_ws/fuzz.sh
```
⚠️  Make sure to have installed the python libraries pypot, explauto and PoppyTorso.

# Launch the process
## 1. Start the Raspberry Pi part
⚠️  Don't forget the `-X` for allowing X11 forwarding, which is necessary to use pygame, used for having the feedback of joysticks.
 
```
ssh -X pi@fuzz.local
cd /home/pi/ros_ws/
./fuzz.sh
roslaunch nips2016 raspberry_pi.launch light:=false
```
The `light` parameter tells whether we want to control WS2812B lights on GPIO18.
If successful, this command must show "Ergo is ready and starts joystick servoing..." and moving joysticks must move the Ergo Jr robot accordingly. You can then

## 2. Start the Workstation part
### 2.1 Setup the ROS environment
```
cd ~/catkin_ws/fuzz.sh
./fuzz.sh
```
The BASH script here above setups ROS variable to use `fuzz.local` as the ROS master.
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
 - `name:=<string>` that will be used to output log files (a pickle file in `logs/`)
 - `iterations:=<integer>` that will automatically stop the controller after a certain number of iteration (aka trials).
 - `save:=<bool>` that will dump the database in folder logs/<name>.pickle (can't revert to previous state when save:=true)

 For example:
```
roslaunch nips2016 start.launch name:=new_experiment iterations:=6000 save:=false
```

Robot immediately starts the requested number of iterations. Press Ctrl+C to interrupt and cause log saving.

### Database dump

To record a database file, you must use parameter `save:=true`.
If file logs/<name>.pickle exists it will be pursued from the end, otherwise a new file is created.
With `save:=true` it is not possible to revert to a previous state.

**By default saving is not enabled** and experiment will either restart from last iteration of <name>.pickle or restart from scratch without saving.

This command will bring you to the folder containing recorded database, if any:
```
roscd nips2016/logs
```



If you want to use a **pre-trained environment**, we provide a [backup](https://github.com/sebastien-forestier/NIPS2016/releases/download/v0.1/experiment.pickle) of 5000 iterations experiment. 
Download the pickle on the logs folder, and start the experiment with the default name
```
roscd nips2016/logs
wget https://github.com/sebastien-forestier/NIPS2016/releases/download/v0.1/experiment.pickle -O experiment.pickle
roslaunch nips2016 start.launch iterations:=20000
```
The big number of iterations is to avoid to stop the experiment before starting, because the number of iterations on the backup are already higher than the default maximum iterations value.


#Troubleshooting

* Poppy torso is using the laptop's camera instead of the external webcam.
  * Change the camera used in the file tracking.py from VideoCapture(1) to VideoCapture(0).
* Poppy torso does not face forward when activate. 
  * Change the first motor in the file torso.py for rest and run functions.
* The ball and / or arena are not being recognized.
  * Change the HSV values in the config file environment.json to the ones needed for your setup. 
  * To obtain the values, run the project and hover the cursor over the item you want to detect in each of the three images (Hue, Saturation and Value). Use those files to modify the config file accordingly.
  * After the objects are detected properly, you may comment out the lines showing those screens in tracking.py if you want less clutter at startup.
* The joysticks are not working
  * Check if both joysticks are compatible with your system, you may need to install additional drivers.
* Error after using the button to teach new actions.
  * Check if your computer has Beep enabled, if not take steps to correct that.
* Error when the ball reaches the border of the arena.
  * When the ball reaches the border of the arena, a sound is supposed to be heard. If instead you get the error `ALSA lib pcm.c:7963:(snd_pcm_recover) underrun occurred`, you may need to adjust your computer's settings to be compatible with PyAudio.