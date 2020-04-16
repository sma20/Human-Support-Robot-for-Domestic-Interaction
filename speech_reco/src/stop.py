#!/usr/bin/env python3

import os
import sys
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import nltk
from nltk.corpus import stopwords
from nltk.corpus import wordnet
from std_msgs.msg import String
import os

#rospy.init_node('speech_rec', anonymous=True)

audio = record = aup = None



def killAll():
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n","")

    for node in nodes:
        os.system("rosnode kill "+ node)


def talker():
    pub = rospy.Publisher('chatter1', String, queue_size=10)

    rospy.init_node('stop', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        print('k')
        text=main()
        text= text.lower()
        print(text)
        if text is not None:
            if 'stop' in text:
                pub.publish(text)
            elif 'danger' in text:
                pub.publish(text)
                #os.system("espeak 'i am shutting down")
                rospy.on_shutdown(killAll)
                print('im shutting down')
                killAll()
    rate.sleep()



def main():
    global audio, record, aup
    # obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = r.listen(source)
        #rospy.spin(1)
    speechToText = ""
    # recognize speech using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        speechToText = r.recognize_google(audio)
        #print("Google Speech Recognition thinks you said " + speechToText)
        return speechToText

    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")

    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


if __name__ == '__main__':
    #while not rospy.is_shutdown():

    text=talker()
