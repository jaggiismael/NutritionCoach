from enum import Enum
from api.stt_api import SttApi
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
            self.__stt_api = SttApi()
          
    #Greeting when starting the programme
    def greeting(self):
        print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.")
        if self.__platform == "TERMINAL":
            winsound.PlaySound("audio_files/greeting.wav", winsound.SND_FILENAME)
        else:
            self.__synchronizer.sync([
                (0, lambda: self.__emotionShow("QT/happy")),
                (0, lambda: self.__behavior_talk("Hello! I'm Q T and I'm your personal nutrition coach. You can always leave, just say exit.")),
                (0, lambda: os.system("rosservice call /qt_robot/gesture/play 'name: 'QT/hi''"))
            ])

    #Output to the user including an emotion     
    def output_emotion(self, text, emotion):
        print(text)
        if self.__platform == "TERMINAL":
            if emotion.name == "CONFUSED":
                winsound.PlaySound("audio_files/wrong_input.wav", winsound.SND_FILENAME)
            else:
                self.__tts_api.text_to_wav("en-US-Neural2-F", text)
                winsound.PlaySound("audio_files/en-US-Neural2-F.wav", winsound.SND_FILENAME)   
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
            
            self.__synchronizer.sync([
                (0, lambda: self.__behavior_talk(text)),
                (0, lambda: self.__emotionShow(emotion)),
                (0, lambda: os.system("rosservice call /qt_robot/gesture/play 'name: '"+gesture+"''"))
            ])

    #Normal output to user
    def output(self, text):
        print(text)
        if self.__platform == "TERMINAL":
            self.__tts_api.text_to_wav("en-US-Neural2-F", text)
            winsound.PlaySound("audio_files/en-US-Neural2-F.wav", winsound.SND_FILENAME)
        else:
            self.__behavior_talk(text)

    #User input with predefined input options, Return Value: User input in text form 
    def input_decision(self, text, opt1, opt2):
        print(text)
        if self.__platform == "TERMINAL":
            if opt1.upper() == "SIGN":
                winsound.PlaySound("audio_files/login_register.wav", winsound.SND_FILENAME)
            elif opt1.upper() == "YES":
                winsound.PlaySound("audio_files/new_suggestion.wav", winsound.SND_FILENAME)
            else:
                winsound.PlaySound("audio_files/main_menu.wav", winsound.SND_FILENAME)
            self.__record_audio(3)
            return self.__stt_api.transcribe_file("audio_files/recorded_audio.wav")
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
            print(text)
            winsound.PlaySound("audio_files/nutrition_question.wav", winsound.SND_FILENAME)
            winsound.PlaySound("audio_files/press_enter.wav", winsound.SND_FILENAME)
            input()
            self.__record_audio(6)
            return self.__stt_api.transcribe_file("audio_files/recorded_audio.wav")
        else:
            """
            print("Robot Online Speech Recognition")
            self.__behavior_talk(text)
            #Shows the user that they can now speak using a signal.
            self.__audio_play("infobleep", "")
            #STT of the user input
            answer = self.__recognize_speech("en_US", [], 10).transcript
            print(answer)
            """
            return input(text)

    #User input via the terminal, Return Value: User input in text form 
    def input_terminal(self, text):
        return input(text)
    
    #Goodbye if user want to close the program
    def goodbye(self):
        print("Ok bye, I hope to see you again soon.")
        if self.__platform == "TERMINAL":
            winsound.PlaySound("audio_files/goodbye.wav", winsound.SND_FILENAME)
        else:
            self.__behavior_talk("Ok bye, I hope to see you again soon.")

    #Function to record audio in terminal
    #Parts of the function taken from https://medium.com/@sarahisdevs/create-a-voice-recorder-using-python-daadd9523e98
    def __record_audio(self, seconds):
        import pyaudio
        import wave

        FORMAT = pyaudio.paInt16  # Format of audio samples (16-bit signed integers)
        CHANNELS = 2              # Number of audio channels (1 for mono, 2 for stereo)
        RATE = 44100              # Sample rate (samples per second)
        CHUNK = 1024              # Number of frames per buffer

        #Open the audio stream
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK)
        
        winsound.PlaySound("audio_files/infobleep.mp3", winsound.SND_FILENAME)
        print("Recording...")

        frames = []
        #Recording
        for _ in range(0, int(RATE / CHUNK * seconds)):
            data = stream.read(CHUNK)
            frames.append(data)

        print("Recording finished.")
        winsound.PlaySound("audio_files/infobleep.mp3", winsound.SND_FILENAME)

        # Stop and close the audio stream
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the recorded audio as a .wav file
        WAVE_OUTPUT_FILENAME = "audio_files/recorded_audio.wav"
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
    
