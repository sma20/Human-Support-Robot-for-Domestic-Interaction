#!/usr/bin/env python3

import os
import sys
import rospy
import test
import time
import speech_recognition as sr
from threading import Thread


def start_test():

    while True:
        text=main()
        print(text)
        if text is not None:
            text= text.lower()
            if 'hello' in text:
                test.start()
            else:
                pass
        time.sleep(5)

def main():
    global audio, record, aup
    # obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = r.listen(source)
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


def execThread():
    thread1 = Thread(target=callback(),args=())
    #thread1.daemon = True
    thread1.start()

#if __name__ == '__main__':
    #while not rospy.is_shutdown():
    #execThread()
    #text=start_test()
    #print('im here')
