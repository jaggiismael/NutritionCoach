from LLM import API_Service
from Model import User
from Model import User_Manager
import logging

class Controller:
    def __init__(self):
        self.__api_service = API_Service.LlmService()
        self.__user_manager = User_Manager.UserManager()
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s [%(levelname)s] - %(message)s',
            handlers=[
                logging.FileHandler('program.log')
            ]
        )

    def start(self):
        logging.info("Application started")
        print("Hi I'm your personal nutrition coach, I will be happy to help you. You can always leave, just enter exit.\n")
        exit = False
        profile_ok = False
        while not profile_ok:
            user_input = input("Would you like to (L)og in or (R)egister?\n").upper()
            match user_input:
                case "L":
                    profile_ok = self.__login()
                    if profile_ok:
                        print("Welcome back " + self.user.firstname)
                        logging.info("Login successful")
                case "R":
                    profile_ok= self.__register()
                    if profile_ok:
                        print("Nice to meet you, " + self.user.firstname)
                        logging.info("Profile Creation successful")
                case "EXIT":
                    exit = True
                    profile_ok = True
                case _:
                    print("Wrong Input")

        while not exit:
            user_input = input("Are you interested in asking me some (n)utrition-related questions or would you prefer (m)eal suggestions for today?\n").upper()
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
                    print("Wrong Input")
        
    def __login(self):
        logging.info("Start Login")
        firstname = input("To get your user profile, enter your Firstname and Lastname\nFirstname: ")
        logging.info("Firstname: " + firstname)
        lastname = input("Lastname: ")
        logging.info("Lastname: " + lastname)
        self.user = self.__user_manager.getUser(firstname, lastname)
        if self.user == None:
            print("User not found")
            logging.info("User not found")
            return False
        return True


    def __register(self):
        logging.info("Start Register")
        firstname = input("Enter your first name: ")
        logging.info("Firstname: " + firstname)
        lastname = input("Enter your last name: ")
        logging.info("Lastname: " + lastname)

        age = None
        while age == None:
            age = input("Enter your age: ")
            if not self.__checkNumber(age, 0, 120):
                age = None
        logging.info("Age: " + age)

        gender = None
        while gender == None:
            gender = input("Whats your gender. Enter the correct number\n1. Male\n2. Female\n")
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
            height = input("Enter your height (in cm): ")
            if not self.__checkNumber(height, 65, 220):
                height = None
        logging.info("Height: " + height)
        
        weight = None
        while weight == None:
            weight = input("Enter your weight (in kg): ")
            if not self.__checkNumber(weight, 0, 150):
                weight = None
        logging.info("Weight: " + weight)
        
        allergies = input("Do you have any allergies? If yes, please specify. If not, enter 'None': ")
        logging.info("Allergies: " + allergies)
        dietary_target = input("What is your dietary target (e.g., lose weight, maintain weight, gain muscle)? ")
        logging.info("Dietary Target: " + dietary_target)

        habit = None
        while habit == None:
            habit = input("What is your eating habit? Enter the correct number\n1. Omnivor (Eats everything)\n2. Vegetarian\n3. Vegan\n4. Pescetarian\n")
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
                print("Please enter a valid number between "+str(min)+" and "+str(max)+"!")
                logging.info("Wrong Input")
                return False
        except ValueError:
            print("Please enter a valid number between "+str(min)+" and "+str(max)+"!")
            logging.info("Wrong Input")
            return False 
        
    def __nutritionCoaching(self):
        systemPrompt = "You are a nutrition coach and only answer my questions if they are related to nutrition. If they are about something else, ignore them. If its about nutrition give a short and helpful answer. Always answer in max 5 sentences and without a greeting. Answer as if it were a spoken conversation. Use this informations to provide personalised answers: " + self.user.__str__()
        messages = [{"role": "system", "content": ""}]
        while True:
            userRequest = input("Which question about nutrition you want to ask? \n")

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
                    print("I'm sorry but I can't answer to this question, please ask something else")
                    logging.info("Question answering not possible")
                    answerVerified = True
                    break

                answer = self.__api_service.sendPrompt(messages, 0.2)
                logging.info("Answer: " + answer)

                if self.__controlLlmAnswer(context, userRequest, answer):
                    messages.append({"role": "assistant", "content": answer})
                    print(answer)
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

            print(answer)

            userInput = input("You want to get new suggestions? (Yes / No)\n").upper()
            if userInput == "NO":
                logging.info("Meal Suggestion finished")
                break

            messages.append({"role": "assistant", "content": answer})
            messages.append({"role": "user", "content": systemPrompt})
            logging.info("User want new suggestions")
                



