from enum import Enum
from utils.synchronizer import TaskSynchronizer
from api.tts_api import TtsApi
import os

import winsound






class Emotion(Enum):
    HAPPY = 1
    SAD = 2
    CONFUSED = 3
    NEUTRAL = 4

class UserInteractionManager:
    def __init__(self, platform):
        self.__platform = platform
        
        if platform == "ROBOT":
            #Imports for QTrobot
            import rospy
            from qt_robot_interface.srv import emotion_show, audio_play, behavior_talk_text
            from qt_gspeech_app.srv import speech_recognize
            from std_msgs.msg import String

            #Connect Services                
            self.__emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
            self.__behavior_talk = rospy.ServiceProxy('//qt_robot/behavior/talkText', behavior_talk_text)
            self.__recognize_speech = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
            self.__online_recognize_speech = rospy.ServiceProxy('/qt_robot/gspeech/recognize', speech_recognize)
            self.__audio_play = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
            self.__synchronizer = TaskSynchronizer()
        else:
            self.__tts_api = TtsApi()
          
    #Greeting when starting the programme
    def greeting(self):
        
        if self.__platform == "TERMINAL":
            print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.\n")
            #self.__tts_api.text_to_wav("en-US-Neural2-F", "You want to get new suggestions?")


            winsound.PlaySound('audio_files/greeting.wav', winsound.SND_FILENAME)

        else:
            print("Greeting")
            
            self.__synchronizer.sync([
                (0, lambda: self.__emotionShow("QT/happy")),
                (0, lambda: self.__behavior_talk("Hello! I'm Q T and I'm your personal nutrition coach. You can always leave, just say exit.")),
                (0, lambda: os.system("rosservice call /qt_robot/gesture/play 'name: 'QT/hi''"))
            ])

    #Output to the user including an emotion     
    def output_emotion(self, text, emotion):
        if self.__platform == "TERMINAL":
            if emotion.name == "CONFUSED":
                winsound.PlaySound('audio_files/wrong_input.wav', winsound.SND_FILENAME)
            else:
                self.__tts_api.text_to_wav("en-US-Neural2-F", text)
                winsound.PlaySound('audio_files/en-US-Neural2-F.wav', winsound.SND_FILENAME)
            print(text)
        else:
            if emotion.name == "HAPPY":
                emotion = "QT/happy"
                gesture = "QT/happy"
            elif emotion.name == "SAD":
                emotion = "QT/sad"
                gesture = "QT/sad"
            elif emotion.name == "CONFUSED":
                emotion = "QT/confused"
                gesture = "QT/surprise"
            else:
                emotion = "QT/neutral"
                gesture = "QT/neutral"
            print(text)
            
            self.__synchronizer.sync([
                (0, lambda: self.__behavior_talk(text)),
                (0, lambda: self.__emotionShow(emotion)),
                (0, lambda: os.system("rosservice call /qt_robot/gesture/play 'name: '"+gesture+"''"))
            ])

    #Normal output to user
    def output(self, text):
        if self.__platform == "TERMINAL":
            self.__tts_api.text_to_wav("en-US-Neural2-F", text)
            winsound.PlaySound('audio_files/en-US-Neural2-F.wav', winsound.SND_FILENAME)
            print(text)
        else:
            print(text)
            self.__behavior_talk(text)

    #User input with predefined input options, Return Value: User input in text form 
    def input_decision(self, text, opt1, opt2):
        if self.__platform == "TERMINAL":
            if opt1.upper() == "LOGIN":
                winsound.PlaySound('audio_files/login_register.wav', winsound.SND_FILENAME)
            elif opt1.upper() == "YES":
                winsound.PlaySound('audio_files/new_suggestion.wav', winsound.SND_FILENAME)
            else:
                winsound.PlaySound('audio_files/main_menu.wav', winsound.SND_FILENAME)
            return input(text)
        else:
            self.__behavior_talk(text)
            #Shows the user that they can now speak using a signal.
            self.__audio_play("infobleep", "")
            
            #STT of the user input
            answer = self.__recognize_speech("en_US",[opt1, opt2, "exit"], 10).transcript
            print("I understood: " + answer)
            return answer

    #User input without predefined input options, Return Value: User input in text form 
    def input(self, text):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            print("Robot Online Speech Recognition")
            self.__behavior_talk(text)
            #Shows the user that they can now speak using a signal.
            self.__audio_play("infobleep", "")
            #STT of the user input
            answer = self.__recognize_speech("en_US", [], 10).transcript
            print(answer)
            return input(text)

    #User input via the terminal, Return Value: User input in text form 
    def input_terminal(self, text):
        return input(text)
    
    #Goodbye if user want to close the program
    def goodbye(self):
        if self.__platform == "TERMINAL":
            winsound.PlaySound('audio_files/goodbye.wav', winsound.SND_FILENAME)
        else:
            print("text")
            self.__behavior_talk("text")
    
