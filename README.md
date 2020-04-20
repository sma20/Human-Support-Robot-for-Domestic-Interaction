# HSR Domestic Interaction

<div style="text-align:center">
<p align="center">
<img src="https://github.com/sma20/HSR/blob/master/image/1200px-Heriot-Watt_University_logo.svg.png " width="300" >
</p>
</div>



The aim of the project is to enable the HSR to complet basic domestic interaction. With the help of several open source packages and our State Machine the robot can perform mapping, navigation, object reognition and mapping (semantic) as well as speech recognition and synthesis for set situations. The project provide an achritecure for human interaction. The template can be used for further investigation.


The entire project is achieved using ROS Kinetic/Melodic. The following packages are implemented using Python, SMACH, YOLO, Gooogle API , Espeak and HSR tools.


<p align="center">
<img src="https://github.com/sma20/HSR/blob/master/image/hsr-photo1-full.jpg " width="400" >
</p>
## Hardware Description
"
Human Support Robot (HSR) developed by TOYOTA is a living support robot that assists handicapped and elderly people in everyday life. It has different advanced capabilities, such as: self-localization, recognition based failure avoidance, and motion planning. It can pick up objects placed on the oor and from shelves at different heights. HSR has four cameras and three sensors as shown in Fig. 3. The roles of cameras and sensors aredescribed as follows:
* RGB-D sensor: It obtains color image and depth image. It can be used to
recognize objects and environments.
* Wide-angle camera: It obtains a wide-angle view image of horizontal 135°.
* Stereo camera: It obtains the distance information of the object from the
parallax of the left and right camera images.
* IMU sensor: It obtains acceleration and angular velocity of HSR.
* Force-Torque sensor: It obtains the force and torque acting on the wrist.
* Laser range sensor: It obtains the distance information of ambient environ-
ment with a horizontal scan angle 250°.

"
HSR description made by DSPL Robocup@Home Team on https://emlab.jimdo.com/

## Presentation

Photo archi
(archi with 3 parts)

Photo rqt des topics links 

The architecture is composed of 3 actions : "Mapping", "Get" and "Welcome" each chosen by vocal command,  
eg. "Bring me a banana".  
The action = "Get"  
The object = "Banana"  
Where from ="Home"  
  
eg. "Go check the door"  
Action = "Welcome"  
Object = "_"  
Where from ="_"  

* "Mapping" does a mapping of the home (the doors must be open), object recognition and vocal command are on (to abort the mapping or even stop hsr completely). Once the process finished (success or not) the map is saved.

* "Get" searches in its semantic map if he knows this object and where it is. If yes, then it searches a free space close to it and move. If no, it searches the room concerned by the command and if the object is found the process end and the object position is returned, the closest accessible point found and the robot move.

* "Welcome" works when the user asks the robot to go and check the door. Through our Speech Recognition and Synthesis we enable the robot to welcome the visitor. #Brieuc please add the human recognition plan.



## Prerequisite
You need to have Ros, Rviz, Gazebo installed on your laptop AND have the autorisation to install the HSR simulation.  

### State Machine Package PreRequisit
To use the state machine you need to have smach installed:
```bash
sudo apt install ros-$ROS_DISTRO-smach*
```

### Mapping Package PreRequisit
To be able to map, you need Hector mapping:
```bash
sudo apt-get install ros-$ROS_DISTRO-hector*
```
### Speech Recognition Package PreRequisits

*installing Speech Recognition*
```bash
pip install SpeechRecognition
```
or 
```bash
pip install speech-recognition or with pip3
```
*installing nltk*

Install Setuptools: http://pypi.python.org/pypi/setuptools
Install Pip:
```bash
sudo easy_install pip
```
Install Numpy (optional): run 
```bash
sudo pip install -U numpy
```
Install NLTK: run 
```bash
sudo pip install -U nltk
```
Test installation: run 
```bash
python then type import nltk
```
To install the package like stopwords, tokenize verbs, nouns, we need nltk packages
Open python or python3 on terminal
write the following script
```
import nltk
nltk.download()
```
install all the corposes.


*installing eSpeak*
```bash
*sudo apt-get install espeak
```

*Installing PyAudio*
```bash
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
sudo apt-get install ffmpeg libav-tools
sudo pip install pyaudio
```

*for Text to Speech
```
sudo apt update
sudo apt install python-espeak
```
***To run(for now)
```bash
rosrun speech_reco thread.py
say hello and then give command.
///still have to put it in all.launch
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
and change the file ```bash get_3d_position.py ``` by the one on our github.

once this is done take the package semantic_hsr on our github and build the workspace.

In order to try YOLO you need to do the following:

-launch the gazebo simulator
- do : ```bash roslaunch hsrb_darknet_tutorials default_model_demo.launch  ```
- do : ```bash python get_3d_position.py ```
- do : ```bash python csv_writer.py ```



## Launch

to check the state machine the viewer is launched with:
```bash
  rosrun smach_viewer smach_viewer.py
```
to launch the gazebo_world (HW lab) and rviz with hsr on it:
```bash
  roslaunch hsr_world hsr_world.launch 
```
to launch the state machine, yolo and the speech recognition (to stop the action or the machine):
```bash
  roslaunch architecture all.launch 
```
Note: Some python files will have to be adapted to your paths as we use absolute paths. 

## List of Task

- [x] Hector Mapping 
- [x] Navigation
- [x] Object Recognition 
- [x] Semantic Mapping
- [x] Speech Recognition
- [x] Speech Synthesis
- [ ] Recognition of all the simulated objects
- [ ] Architeture fully implemented


## Acknowledgment 

Some packages have been adapted from other githubs:
* [Find_frontier]( https://github.com/bnurbekov/Turtlebot_Navigation) - Change made: it has been adapted to fit our robot, mean of motion, added a time limit, rearranged the goal setting, used Hector mapping (instead of gmapping), saved the map autonomously and arranged to call the whole process autonomously.
