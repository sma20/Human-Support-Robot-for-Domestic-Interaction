# HSR XXX


Change the plan how you see fit



## Presentation

## Prerequisite
You need to have Ros, Rviz, Gazebo installed on your laptop AND have the autorisation to install the HSR simulation.  

### State Machine Package PreRequisit
To use the state machine you need to have smach installed:
```bash
sudo apt-get install ros-kinetic-smach*
```

### Speech Recognition Package PreRequisits

*installing Speech Recognition*
```bash
*pip install SpeechRecognition or pip install speech-recognition or with pip3
```
*installing nltk*
```bash
*Install Setuptools: http://pypi.python.org/pypi/setuptools
*Install Pip: run sudo easy_install pip
*Install Numpy (optional): run sudo pip install -U numpy
*Install NLTK: run sudo pip install -U nltk
*Test installation: run python then type import nltk
```

*installing eSpeak*
```bash
*sudo apt-get install espeak
```

*Installing PyAudio*
```bash
*sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
*sudo apt-get install ffmpeg libav-tools
*sudo pip install pyaudio
```
### YOLO Package PreRequisits
In order to use YOLO with the hsr you first need to install this 3 package : 
```bash
git clone https://git.hsr.io/tmc/tmc_darknet.git
git clone https://git.hsr.io/tmc/tmc_darknet_ros.git
git clone https://git.hsr.io/tmc/hsrb_tk1_tutorials.git
```
do:
```bash
 cd ~/yolo_ws/src/hsrb_tk1_tutorials/hsrb_darknet_tutorials
  scripts/install_scripts/install_tk1_settings.sh
  ```
in order to have the weight do :
```bash
roscd hsrb_darknet_tutorials
scripts/install_scripts/get_yolo_data.sh
```
build them and after that install the package at this address : https://git.hsr.io/kazuto_murase/get_3d_position
and change the file by the one on our github
once this is done take the package on our github and build the workspace.
In order to try YOLO you need to do the following
-launch the gazebo simulator
-do : ```bash roslaunch hsrb_darknet_tutorials default_model_demo.launch  ```
- rosrun the get_3d_position
-rosrun our package

***To run(for now)
```bash
rosrun speech_reco test.py
rosrun speech_reco stop.py
```
## Packages descriptions
### Find_frontier

## Usage

```python
import etc
what to launch here
```
Some package will have to be adapted to your paths.
In XX.py
```python
what to change here
```

## Test

to check the state machine the viewer is launched with:
```bash
  rosrun smach_viewer smach_viewer.py
```
to launch the gazebo_world (HW lab) and rviz with hsr on it:
```bash
  roslaunch find_frontier find_front.launch 
```


## Acknowledgment 

Some packages have been adapted from other githubs:
* [Find_frontier]( https://github.com/bnurbekov/Turtlebot_Navigation) - Change made: it has been adapted to fit our robot, mean of motion, added a time limit, used hector mapping instead of gmapping and save the map autonomously. 
