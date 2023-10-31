from API.LLM_API import LLM_API
from Model.User import User
from Model.User_Manager import UserManager
from UserInteraction.UserInteractionManager import Emotion, UserInteractionManager
import logging


class Controller:
    def __init__(self, platform):
        self.__api_service = LLM_API()
        self.__user_manager = UserManager()
        self.__user_interaction_manager = UserInteractionManager(platform)
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s [%(levelname)s] - %(message)s',
            handlers=[
                logging.FileHandler('program.log')
            ]
        )

    def start(self):
        logging.info("Application started")
        self.__user_interaction_manager.greeting()
        exit = False
        profile_ok = False
        while not profile_ok:
            user_input = self.__user_interaction_manager.inputDecision("Would you like to (L)ogin or (R)egister?\n", "Login", "Register").upper()
            match user_input:
                case "L":
                    profile_ok = self.__login()
                    if profile_ok:
                        self.__user_interaction_manager.output("Welcome back " + self.user.firstname, Emotion.HAPPY)
                        logging.info("Login successful")
                case "R":
                    profile_ok= self.__register()
                    if profile_ok:
                        self.__user_interaction_manager.output("Nice to meet you, " + self.user.firstname, Emotion.HAPPY)
                        logging.info("Profile Creation successful")
                case "EXIT":
                    exit = True
                    profile_ok = True
                case _:
                    self.__user_interaction_manager.output("Wrong Input", Emotion.CONFUSED)

        while not exit:
            user_input = self.__user_interaction_manager.inputDecision("Are you interested in asking me some (n)utrition-related questions or would you prefer (m)eal suggestions for today?\n", "Nutrition", "Meal").upper()
            match user_input:
                case "N":
                    logging.info("Start Nutrition Coaching")
                    self.__nutritionCoaching()
                case "M":
                    logging.info("Start Meal Suggestion")
                    self.__mealSuggestion()
                case "EXIT":
                    logging.info("Application finished")
                    exit = True
                case _:
                    self.__user_interaction_manager.output("Wrong Input", Emotion.CONFUSED)
        
    def __login(self):
        logging.info("Start Login")
        firstname = self.__user_interaction_manager.input("To get your user profile, enter your Firstname and Lastname\nFirstname: ")
        logging.info("Firstname: " + firstname)
        lastname = self.__user_interaction_manager.input("Lastname: ")
        logging.info("Lastname: " + lastname)
        self.user = self.__user_manager.getUser(firstname, lastname)
        if self.user == None:
            self.__user_interaction_manager.output("User not found", Emotion.CONFUSED)
            logging.info("User not found")
            return False
        return True


    def __register(self):
        logging.info("Start Register")
        firstname = self.__user_interaction_manager.inputRegister("Enter your first name: ")
        logging.info("Firstname: " + firstname)
        lastname = self.__user_interaction_manager.inputRegister("Enter your last name: ")
        logging.info("Lastname: " + lastname)

        age = None
        while age == None:
            age = self.__user_interaction_manager.inputRegister("Enter your age: ")
            if not self.__checkNumber(age, 0, 120):
                age = None
        logging.info("Age: " + age)

        gender = None
        while gender == None:
            gender = self.__user_interaction_manager.inputRegister("Whats your gender. Enter the correct number\n1. Male\n2. Female\n")
            if not self.__checkNumber(gender, 1, 2):
                gender = None
        match int(gender):
            case 1:
                gender = "Male"
            case 2:
                gender = "Female"
        logging.info("Gender: " + gender)

        height = None
        while height == None:
            height = self.__user_interaction_manager.inputRegister("Enter your height (in cm): ")
            if not self.__checkNumber(height, 65, 220):
                height = None
        logging.info("Height: " + height)
        
        weight = None
        while weight == None:
            weight = self.__user_interaction_manager.inputRegister("Enter your weight (in kg): ")
            if not self.__checkNumber(weight, 0, 150):
                weight = None
        logging.info("Weight: " + weight)
        
        allergies = self.__user_interaction_manager.inputRegister("Do you have any allergies? If yes, please specify. If not, enter 'None': ")
        logging.info("Allergies: " + allergies)
        dietary_target = self.__user_interaction_manager.inputRegister("What is your dietary target (e.g., lose weight, maintain weight, gain muscle)? ")
        logging.info("Dietary Target: " + dietary_target)

        habit = None
        while habit == None:
            habit = self.__user_interaction_manager.inputRegister("What is your eating habit? Enter the correct number\n1. Omnivor (Eats everything)\n2. Vegetarian\n3. Vegan\n4. Pescetarian\n")
            if not self.__checkNumber(habit, 1, 4):
                habit = None
        match int(habit):
            case 1:
                habit = "Omnivor"
            case 2:
                habit = "Vegetarian"
            case 3:
                habit = "Vegan"
            case 4:
                habit = "Pescetarian"
        logging.info("Eating Habits: " + habit)

        self.user = User.User(firstname, lastname, age, gender, height, weight, allergies, dietary_target, habit)
        self.__user_manager.writeJSON(self.user)
        logging.info("User-Profile saved")
        return True

    def __checkNumber(self, input, min, max):
        try:
            number = float(input)  
            if number >= min and number <= max:
                return True
            else:
                self.__user_interaction_manager.output("Please enter a valid number between "+str(min)+" and "+str(max)+"!", Emotion.CONFUSED)
                logging.info("Wrong Input")
                return False
        except ValueError:
            self.__user_interaction_manager.output("Please enter a valid number between "+str(min)+" and "+str(max)+"!", Emotion.CONFUSED)
            logging.info("Wrong Input")
            return False 
        
    def __nutritionCoaching(self):
        systemPrompt = "You are a nutrition coach and only answer my questions if they are related to nutrition. If they are about something else, ignore them. If its about nutrition give a short and helpful answer. Always answer in max 5 sentences and without a greeting. Answer as if it were a spoken conversation. Use this informations to provide personalised answers: " + self.user.__str__()
        messages = [{"role": "system", "content": ""}]
        while True:
            userRequest = self.__user_interaction_manager.input("Which question about nutrition you want to ask? \n")

            if(userRequest.lower() == "exit"):
                logging.info("Nutrition Coaching finished")
                break

            logging.info("Question asked: " + userRequest)

            context = messages
            messages.append({"role": "user", "content": systemPrompt + ". Question: " + userRequest})

            answerVerified = False
            attempts = 0
            while not answerVerified:

                if attempts > 2:
                    self.__user_interaction_manager.output("I'm sorry but I can't answer to this question, please ask something else", Emotion.SAD)
                    logging.info("Question answering not possible")
                    answerVerified = True
                    break

                answer = self.__api_service.sendPrompt(messages, 0.2)
                logging.info("Answer: " + answer)

                if self.__controlLlmAnswer(context, userRequest, answer):
                    messages.append({"role": "assistant", "content": answer})
                    #Emotion change
                    self.__user_interaction_manager.output(answer, Emotion.HAPPY)
                    answerVerified = True
                else:
                    attempts += 1

    def __controlLlmAnswer(self, context, question, answer):
        contextStr = ','.join(str(v) for v in context)
        messages = [{"role": "system", "content": "Your job is to check whether the answer fits the question based on the context. If the answer and the question match, answer: True. If the last question and the answer do not match, answer: False. The answer must not be racist, sexist or offensive. Don't explain the decision, just reply with true or false."}, 
              {"role": "user", "content": "Context: " + contextStr + " Question: " + question + " Answer: " + answer}]
        
        verifyAnswer = self.__api_service.sendPrompt(messages, 0.2)

        if verifyAnswer.__contains__("True") or verifyAnswer.__contains__("true"):
            logging.info("Answer verified")
            return True
        
        logging.info("Answer not verified")
        return False
    
    def __mealSuggestion(self):
        systemPrompt = "I would like a new recommendation on what to eat for breakfast, lunch and dinner. I only want one recommendation per meal. I want it in 3-4 sentences and without a greeting. The recommendation must be adapted to me, here is my profile: "  + self.user.__str__()
        messages = [{"role": "system", "content": ""}, {"role": "user", "content": systemPrompt}]

        while True:
            answer = self.__api_service.sendPrompt(messages, 0.2)
            logging.info("Meal Suggestion: " + answer)

            self.__user_interaction_manager.output(answer, Emotion.HAPPY)

            userInput = self.__user_interaction_manager.inputDecision("You want to get new suggestions? (Yes / No)\n", "Yes", "No").upper()
            if userInput == "NO":
                logging.info("Meal Suggestion finished")
                break

            messages.append({"role": "assistant", "content": answer})
            messages.append({"role": "user", "content": systemPrompt})
            logging.info("User want new suggestions")
                



