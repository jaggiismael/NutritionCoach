import json
from Model import User

class UserManager:

    def __init__(self):
        self.__filename = "user_data.json"
    
    def writeJSON(self, user):
        print("Firstname:", user.firstname)
        print("Lastname:", user.lastname)
        print("Age:", user.age)
        print("Gender:", user.gender)
        print("Height:", user.height)
        print("Weight:", user.weight)
        print("Allergies:", user.allergies)
        print("Dietary Target:", user.target)
        print("Eating Habit:", user.habit)

        with open(self.filename, "r") as file:
            data = json.load(file)

        data.append(user.__dict__)

        with open(self.__filename, "w") as json_file:
            json.dump(data, json_file)

    def getUser(self, firstname, lastname):

        with open(self.__filename, 'r') as file:
            data = json.load(file)

        user_data = None
        for userJSON in data:
            if userJSON['firstname'] == firstname and userJSON['lastname'] == lastname:
                user_data = userJSON
                break

        if user_data:
            user = User.User(
                user_data['firstname'],
                user_data['lastname'],
                user_data['age'],
                user_data['gender'],
                user_data['height'],
                user_data['weight'],
                user_data['allergies'],
                user_data['target'],
                user_data['habit']
            )
            return user

        else:
            return None