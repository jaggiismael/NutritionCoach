from enum import Enum
from utils.synchronizer import TaskSynchronizer


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
            from qt_vosk_app.srv import speech_recognize
            from std_msgs.msg import String

            #Connect Services                
            self.__emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
            #self.__gestureShow = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
            self.__behavior_talk = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
            self.__recognize_speech = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
            self.__audio_play = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
            self.__synchronizer = TaskSynchronizer()
          

    def greeting(self):
        if self.__platform == "TERMINAL":
            print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.\n")
        else:
            print("Greeting")
            #self.__gestureShow.publish("QT/happy")
            self.__synchronizer.sync([
                (0, lambda: self.__emotionShow("QT/happy")),
                (0, lambda: self.__behavior_talk("Hello!")) #Hello! I'm Q T and I'm your personal nutrition coach.
            ])
            
    def output(self, text, emotion):
        if self.__platform == "TERMINAL":
            print(text)
        else:
            if(emotion.name == "HAPPY"):
                emotion = "QT/happy"
            elif(emotion.name == "SAD"):
                emotion = "QT/sad"
            elif(emotion.name == "CONFUSED"):
                emotion = "QT/confused"
            else:
                emotion = "QT/neutral"
            print(text)
            
            self.__synchronizer.sync([
                (0, lambda: self.__behavior_talk(text)),
                (0, lambda: self.__emotionShow(emotion)) 
            ])

    def inputDecision(self, text, opt1, opt2):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            self.__behavior_talk(text)    
            self.__audio_play("infobleep", "")
            answer = self.__recognize_speech("en_US", [opt1, opt2, "exit"], 10).transcript
            print("I understood: %s", answer)
            return answer
        
    def input(self, text):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            print("Robot Online Speech Recognition")
            
            return input(text)
        
    def inputTerminal(self, text):
        return input(text)
