# HSR XXX


Change the plan how you see fit



## Presentation

## Prerequisite
You need to have Ros, Rviz, Gazebo installed on your laptop AND have the autorisation to install the HSR simulation.  


To use the state machine you need to have smach installed:
```bash
sudo apt-get install ros-kinetic-smach*
```
## Built With
//examples, to change with what we actually use.

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Packages descriptions

### hsr_world




*installing Speech Recognition*
pip install SpeechRecognition or pip install speech-recognition or with pip3

*installing nltk*
Install Setuptools: http://pypi.python.org/pypi/setuptools
Install Pip: run sudo easy_install pip
Install Numpy (optional): run sudo pip install -U numpy
Install NLTK: run sudo pip install -U nltk
Test installation: run python then type import nltk

*installing eSpeak*
sudo apt-get install espeak

*Installing PyAudio*
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
sudo apt-get install ffmpeg libav-tools
sudo pip install pyaudio


***To run(for now)
rosrun speech_reco test.py
rosrun speech_reco stop.py
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
