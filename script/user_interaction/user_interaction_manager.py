from enum import Enum
from utils.synchronizer import TaskSynchronizer
import os


class Emotion(Enum):
    HAPPY = 1
    SAD = 2
    CONFUSED = 3
    NEUTRAL = 4

class UserInteractionManager:
    def __init__(self, platform):
        self.__platform = platform
        
        #Import libraries for the robot only if nessesary
        if platform == "ROBOT":
            #Imports
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
          

    def greeting(self):
        if self.__platform == "TERMINAL":
            print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.\n")
        else:
            print("Greeting")
            
            self.__synchronizer.sync([
                (0, lambda: self.__emotionShow("QT/happy")),
                (0, lambda: self.__behavior_talk("Hello! I'm Q T and I'm your personal nutrition coach. You can always leave, just say exit.")),
                (0, lambda: os.system("rosservice call /qt_robot/gesture/play 'name: 'QT/hi''"))
            ])
            
    def output_emotion(self, text, emotion):
        if self.__platform == "TERMINAL":
            print(text)
        else:
            if(emotion.name == "HAPPY"):
                emotion = "QT/happy"
                gesture = "QT/happy"
            elif(emotion.name == "SAD"):
                emotion = "QT/sad"
                gesture = "QT/sad"
            elif(emotion.name == "CONFUSED"):
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

    def output(self, text):
        if self.__platform == "TERMINAL":
            print(text)
        else:
            self.__behavior_talk(text)

    def input_decision(self, text, opt1, opt2):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            self.__behavior_talk(text)    
            self.__audio_play("infobleep", "")
            
            answer = self.__recognize_speech("en_US",[opt1, opt2, "exit"], 10).transcript
            print("I understood: " + answer)
            return answer
        
    def input(self, text):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            print("Robot Online Speech Recognition")
            self.__behavior_talk(text)    
            self.__audio_play("infobleep", "")
            answer = self.__recognize_speech("en_US", [], 10).transcript
            print(answer)
            return input(text)

    def input_terminal(self, text):
        return input(text)
