import random
import re
from api.emotion_api import EmotionApi
from api.llm_api import LlmApi
from model.user import User
from model.user_manager import UserManager
from user_interaction.user_interaction_manager import Emotion, UserInteractionManager
import logging
import os


class Controller:
    def __init__(self, platform):
        #Import API Keys as environment variables
        with open("keys.txt") as config_file:
            for line in config_file:
                key, value = line.strip().split("=")
                os.environ[key] = value
        llm_key = os.environ.get("LLM_KEY")

        self.__llm_api = LlmApi("")
        self.__user_manager = UserManager()
        self.__user_interaction_manager = UserInteractionManager(platform)
        self.__emotion_api = EmotionApi()
        #Configure logging
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s [%(levelname)s] - %(message)s',
            handlers=[
                logging.FileHandler('program.log')
            ]
        )

    #Main function. Controls the flow of the entire programme
    def start(self):
        logging.info("Application started")
        self.__user_interaction_manager.greeting()
        exit = False
        profile_ok = False
        #First decision: Login or create a new profile. Will be executed until you have successfully logged in or created a profile
        while not profile_ok:
            user_input = self.__user_interaction_manager.input_decision("Would you like to Login or Register?\n", "login", "register").upper()
            if user_input == "LOGIN":
                self.__user_interaction_manager.output("Please enter your next informations in the terminal")
                profile_ok = self.__login()
                if profile_ok:
                    self.__user_interaction_manager.output_emotion("Welcome back " + self.user.firstname, Emotion.HAPPY)
                    logging.info("Login successful")
            elif user_input == "REGISTER":
                self.__user_interaction_manager.output("Please enter the asked informations in the terminal")
                profile_ok= self.__register()
                if profile_ok:
                    self.__user_interaction_manager.output_emotion("Nice to meet you, " + self.user.firstname, Emotion.HAPPY)
                    logging.info("Profile Creation successful")
            elif user_input == "EXIT":
                exit = True
                profile_ok = True
            else:
                self.__user_interaction_manager.output_emotion("That is not a valid answer", Emotion.CONFUSED)

        #Main menu. Choice between nutritional coaching and meal suggestions
        while not exit:
            user_input = self.__user_interaction_manager.input_decision("Are you interested in asking me some nutrition-related questions or would you prefer meal suggestions for today?\n", "nutrition", "meal").upper()
            if user_input == "NUTRITION":
                logging.info("Start Nutrition Coaching")
                self.__nutritional_coaching()
            elif user_input == "MEAL":
                logging.info("Start Meal Suggestion")
                self.__meal_suggestion()
            elif user_input == "EXIT":
                logging.info("Application finished")
                exit = True
            else:
                self.__user_interaction_manager.output_emotion("That is not a valid answer", Emotion.CONFUSED)
        
        self.__user_interaction_manager.output("OK bye, I hope to see you again soon")
        
    #Function for logging in the user with first and last name
    def __login(self):
        logging.info("Start Login")
        firstname = self.__user_interaction_manager.input_terminal("To get your user profile, enter your Firstname and Lastname\nFirstname: ")
        logging.info("Firstname: " + firstname)
        lastname = self.__user_interaction_manager.input_terminal("Lastname: ")
        logging.info("Lastname: " + lastname)
        self.user = self.__user_manager.getUser(firstname, lastname)
        if self.user == None:
            self.__user_interaction_manager.output_emotion("I don't know this User", Emotion.CONFUSED)
            logging.info("User not found")
            return False
        return True


    #Function for creating a profile for a new user
    def __register(self):
        logging.info("Start Register")
        firstname = self.__user_interaction_manager.input_terminal("Enter your first name: ")
        logging.info("Firstname: " + firstname)
        lastname = self.__user_interaction_manager.input_terminal("Enter your last name: ")
        logging.info("Lastname: " + lastname)

        age = None
        while age == None:
            age = self.__user_interaction_manager.input_terminal("Enter your age: ")
            if not self.__check_number(age, 0, 120):
                age = None
        logging.info("Age: " + age)

        gender = None
        while gender == None:
            gender = self.__user_interaction_manager.input_terminal("Whats your gender. Enter the correct number\n1. Male\n2. Female\n")
            if not self.__check_number(gender, 1, 2):
                gender = None
        if int(gender) == 1:
            gender = "Male"
        else:
            gender = "Female"
        logging.info("Gender: " + gender)

        height = None
        while height == None:
            height = self.__user_interaction_manager.input_terminal("Enter your height (in cm): ")
            if not self.__check_number(height, 65, 220):
                height = None
        logging.info("Height: " + height)
        
        weight = None
        while weight == None:
            weight = self.__user_interaction_manager.input_terminal("Enter your weight (in kg): ")
            if not self.__check_number(weight, 0, 150):
                weight = None
        logging.info("Weight: " + weight)
        
        allergies = self.__user_interaction_manager.input_terminal("Do you have any allergies? If yes, please specify. If not, enter 'None': ")
        logging.info("Allergies: " + allergies)
        dietary_target = self.__user_interaction_manager.input_terminal("What is your dietary target (e.g., lose weight, maintain weight, gain muscle)? ")
        logging.info("Dietary Target: " + dietary_target)

        habit = None
        while habit == None:
            habit = self.__user_interaction_manager.input_terminal("What is your eating habit? Enter the correct number\n1. Omnivor (Eats everything)\n2. Vegetarian\n3. Vegan\n4. Pescetarian\n")
            if not self.__check_number(habit, 1, 4):
                habit = None
        if int(habit) == 1:
        
            habit = "Omnivor"
        if int(habit) == 2:
            habit = "Vegetarian"
        if int(habit) == 3:
            habit = "Vegan"
        if int(habit) == 4:
            habit = "Pescetarian"
        logging.info("Eating Habits: " + habit)

        self.user = User(firstname, lastname, age, gender, height, weight, allergies, dietary_target, habit)
        self.__user_manager.writeJSON(self.user)
        logging.info("User-Profile saved")
        return True

    #Function tests whether an input is a numerical value and whether it is between two values
    def __check_number(self, input, min, max):
        try:
            number = float(input)  
            if number >= min and number <= max:
                return True
            else:
                self.__user_interaction_manager.output_emotion("Please enter a valid number between "+str(min)+" and "+str(max)+"!", Emotion.CONFUSED)
                logging.info("Wrong Input")
                return False
        except ValueError:
            self.__user_interaction_manager.output_emotion("Please enter a valid number between "+str(min)+" and "+str(max)+"!", Emotion.CONFUSED)
            logging.info("Wrong Input")
            return False 
        
    #Main function for nutritional coaching. Questions are received from the user and answers are returned by the LLM
    def __nutritional_coaching(self):
        #Instructions on how the LLM should behave
        systemPrompt = "You are a nutrition coach and only answer my questions if they are related to nutrition. If they are about something else, ignore them. If its about nutrition give a short and helpful answer. Always answer in max 5 sentences and without a greeting. Answer as if it were a spoken ongoing conversation. Use this informations to provide personalised answers: " + self.user.__str__()
        messages = [{"role": "system", "content": ""}]
        while True:
            self.__user_interaction_manager.output("Please enter your next question")
            userRequest = self.__user_interaction_manager.input("Which question about nutrition you want to ask? \n")
            if(userRequest.lower() == "exit"):
                logging.info("Nutrition Coaching finished")
                break
            #Tells the user that answers are being searched for
            self.__user_interaction_manager.output(self.__get_thinking_phrase())
            logging.info("Question asked: " + userRequest)

            #Adds previous conversation to the context so that LLM can respond to previous answers
            context = messages
            messages.append({"role": "user", "content": systemPrompt + ". Question: " + userRequest})

            answerVerified = False
            attempts = 0
            while not answerVerified:

                #If the LLM response is not confirmed by the control layer for the third time, an error message is sent to the user
                if attempts > 2:
                    self.__user_interaction_manager.output_emotion("I'm sorry but I can't answer to this question, please ask something else", Emotion.SAD)
                    logging.info("Question answering not possible")
                    answerVerified = True
                    break

                #Send prompt to the LLM
                answer = self.__llm_api.send_prompt(messages, 0.2)
                logging.info("Answer: " + answer)

                #Response is checked by the control layer
                if self.__check_answer(context, userRequest, answer):
                    messages.append({"role": "assistant", "content": answer})

                    #Sends the first two sentences of the answer to the API to find the emotion in the answer
                    emotion = self.__emotion_api.get_emotion(self.__get_two_sentences(answer))
                    if 0.25 <= emotion <= 1.0:
                        self.__user_interaction_manager.output_emotion(answer, Emotion.HAPPY)
                    elif -0.25 <= emotion < 0.25:
                        self.__user_interaction_manager.output_emotion(answer, Emotion.NEUTRAL)
                    elif -1.0 <= emotion < -0.25:
                        self.__user_interaction_manager.output_emotion(answer, Emotion.SAD)
                    else:
                        logging.error("Problem with emotion detection")
                        self.__user_interaction_manager.output(answer)
                    
                    answerVerified = True
                else:
                    attempts += 1

    #Function works as a control layer. Answers are sent to the LLM again with the context and checked to see whether they match the question.
    def __check_answer(self, context, question, answer):
        contextStr = ','.join(str(v) for v in context)
        messages = [{"role": "system", "content": "Your job is to check whether the answer fits the question based on the context. If the answer and the question match, answer: True. If the last question and the answer do not match, answer: False. The answer must not be racist, sexist or offensive. Don't explain the decision, just reply with true or false."}, 
              {"role": "user", "content": "Context: " + contextStr + " Question: " + question + " Answer: " + answer}]
        
        verifyAnswer = self.__llm_api.send_prompt(messages, 0.2)

        if verifyAnswer.__contains__("True") or verifyAnswer.__contains__("true"):
            logging.info("Answer verified")
            return True
        
        logging.info("Answer not verified")
        return False
    
    #Function for creating meal suggestions.
    def __meal_suggestion(self):
        #Instructions on how the LLM should behave and what to answer
        systemPrompt = "I would like a new recommendation on what to eat for breakfast, lunch and dinner. I only want one recommendation per meal. I want it in 3-4 sentences and without a greeting. The recommendation must be adapted to me, here is my profile: "  + self.user.__str__()
        messages = [{"role": "system", "content": ""}, {"role": "user", "content": systemPrompt}]
        self.__user_interaction_manager.output("Ok I'm thinking about what you could eat today")
        suggestion_wanted = True
        while suggestion_wanted:
            #Send prompt to the LLM
            answer = self.__llm_api.send_prompt(messages, 0.2)
            logging.info("Meal Suggestion: " + answer)

            print(answer)
            self.__user_interaction_manager.output(answer)

            answer_ok = False
            #Ask if the user wants a new suggestion
            while not answer_ok:
                userInput = self.__user_interaction_manager.input_decision("You want to get new suggestions? (Yes / No)\n", "yes", "no").upper()
                if userInput == "NO":
                    logging.info("Meal Suggestion finished")
                    answer_ok = True
                    suggestion_wanted = False
                elif userInput == "YES":
                    #Add the question and answer to the context to avoid receiving the same suggestions again.
                    messages.append({"role": "assistant", "content": answer})
                    messages.append({"role": "user", "content": systemPrompt})
                    logging.info("User want new suggestions")
                    self.__user_interaction_manager.output(self.__get_thinking_phrase())
                    answer_ok = True
                else:
                    self.__user_interaction_manager.output_emotion("Thats not a valid answer", Emotion.CONFUSED)

    #Function that truncates and returns the first two sentences of a text
    def __get_two_sentences(self, answer):
        sentences = re.split(r'(?<=[.!?])\s', answer)
        if len(sentences) >= 2:
            newAnswer = ' '.join(sentences[:2])
        else:
            newAnswer = answer
        return newAnswer
    
    #Function that returns random sentences to bridge waiting times
    def __get_thinking_phrase(self):
        phrases = [
            "I'm thinking about a correct response",
            "Let me think for a moment",
            "Just give me some time to think about that",
            "I need some time to find the correct answer",
            "I'm taking a moment to think"
        ]
        return random.choice(phrases)
    