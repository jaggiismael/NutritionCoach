from enum import Enum

class Emotion(Enum):
    HAPPY = 1
    SAD = 2
    CONFUSED = 3
    NEUTRAL = 4

class UserInteractionManager:
    def __init__(self, platform):
        self.__platform = platform
        #Import libraries for the robot only if nessesary
        """if platform == "ROBOT":
            import rospy"""

    def greeting(self):
        if self.__platform == "TERMINAL":
            print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.\n")
        else:
            print("Robot Greeting")

    def output(self, text, emotion):
        if self.__platform == "TERMINAL":
            print(text)
        else:
            print("Robot Output with emotion: " + emotion.name)

    def inputDecision(self, text, opt1, opt2):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            print("Robot Offline Speech Recognition with " + opt1 + " or " + opt2)
            return ""
        
    def input(self, text):
        if self.__platform == "TERMINAL":
            return input(text)
        else:
            print("Robot Online Speech Recognition")
            return ""
        
    def inputRegister(self, text):
        return input(text)
