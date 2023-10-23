from LLM import API_Service
from Model import User
from Model import User_Manager

class Controller:
    def __init__(self):
        self.__api_service = API_Service.LlmService()
        self.__user_manager = User_Manager.UserManager()

    def start(self):
        print("Hi I'm your personal nutrition coach, I will be happy to help you with questions about nutrition.You can always leave, just enter exit.\n")
        exit = False
        profile_ok = False
        while not profile_ok:
            user_input = input("Would you like to (L)og in or (R)egister?\n").upper()
            match user_input:
                case "L":
                    profile_ok = self.__login()
                    if profile_ok:
                        print("Welcome back " + self.user.firstname)
                case "R":
                    profile_ok= self.__register()
                    if profile_ok:
                        print("Nice to meet you, " + self.user.firstname)
                case "EXIT":
                    exit = True
                    profile_ok = True
                case _:
                    print("Wrong Input")

        while not exit:
            user_input = input("I'm happy to help you. Are you interested in asking me some (n)utrition-related questions or would you prefer a (m)eal suggestion?\n").upper()
            match user_input:
                case "N":
                    self.__nutritionCoaching()
                case "M":
                    print("I can't do that at the moment, " + self.user.firstname)
                case "EXIT":
                    exit = True
                case _:
                    print("Wrong Input")
        
    def __login(self):
        firstame = input("To get your user profile, enter your Firstname and Lastname\nFirstname: ")
        lastname = input("Lastname: ")
        self.user = self.__user_manager.getUser(firstame, lastname)
        if self.user == None:
            print("User not found")
            return False
        return True


    def __register(self):
        firstname = input("Enter your first name: ")
        lastname = input("Enter your last name: ")

        age = None
        while age == None:
            age = input("Enter your age: ")
            if not self.__checkNumber(age, 0, 120):
                age = None

        gender = None
        while gender == None:
            gender = input("Whats your gender. Enter the correct number\n1. Male\n2. Female")
            if not self.__checkNumber(gender, 1, 2):
                gender = None

        height = None
        while height == None:
            height = input("Enter your height (in cm): ")
            if not self.__checkNumber(height, 65, 220):
                height = None
        
        weight = None
        while weight == None:
            weight = input("Enter your weight (in kg): ")
            if not self.__checkNumber(weight, 0, 150):
                weight = None
        
        allergies = input("Do you have any allergies? If yes, please specify. If not, enter 'None': ")
        dietary_target = input("What is your dietary target (e.g., lose weight, maintain weight, gain muscle)? ")

        habit = None
        while habit == None:
            habit = input("What is your eating habit? Enter the correct number\n1. Omnivor (Eats everything)\n2. Vegetarian\n3. Vegan\n4. Pescetarian ")
            if not self.__checkNumber(habit, 1, 4):
                habit = None

        self.user = User.User(firstname, lastname, age, gender, height, weight, allergies, dietary_target, habit)
        self.__user_manager.writeJSON(self.user)
        return True

    def __checkNumber(self, input, min, max):
        try:
            number = float(input)  
            if number >= min and number <= max:
                return True
            else:
                print("Please enter a valid number between "+str(min)+" and "+str(max)+"!")
                return False
        except ValueError:
            print("Please enter a valid number between "+str(min)+" and "+str(max)+"!")
            return False 
        
    def __nutritionCoaching(self):
        systemPrompt = "You are a nutrition coach and only answer my questions if they are related to nutrition. If they are about something else, ignore them. If its about nutrition give a short and helpful answer. Always answer in max 5 sentences and without a greeting. Use this informations to provide personalised answers: " + self.user.__str__()
        messages = [{"role": "system", "content": ""}]
        while True:
            userRequest = input("Which question about nutrition you want to ask? \n")

            if(userRequest.lower() == "exit"):
                break

            context = messages
            messages.append({"role": "user", "content": systemPrompt + ". Question: " + userRequest})

            answer = self.__api_service.sendPrompt(messages, 0.2)

            if self.__controlLlmAnswer(context, userRequest, answer):
                messages.append({"role": "assistant", "content": answer})
                print(answer)
            else:
                messages.pop()
                print("I'm sorry but I can't answer to this question, please ask something else")

    def __controlLlmAnswer(self, context, question, answer):
        contextStr = ','.join(str(v) for v in context)
        messages = [{"role": "system", "content": "Your job is to check whether the answer fits the question based on the context. If the answer and the question match, answer: True. If the last question and the answer do not match, answer: False. Don't explain the decision, just reply with true or false."}, 
              {"role": "user", "content": "Context: " + contextStr + " Question: " + question + " Answer: " + answer}]
        
        answer = self.__api_service.sendPrompt(messages, 0.2)

        if answer.find("True") or answer.find("true"):
            return True
        
        return False
