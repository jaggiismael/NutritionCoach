class User:
    def __init__(self, firstname, lastname, age, gender, height, weight, allergies, target, habit):
        self.firstname = firstname
        self.lastname = lastname
        self.age = age
        self.gender = gender
        self.height = height
        self.weight = weight
        self.allergies= allergies
        self.target = target
        self.habit = habit

    def __str__(self):
        user_info = f"Firstname: {self.firstname}\n"
        user_info += f"Lastname: {self.lastname}\n"
        user_info += f"Age: {self.age}\n"
        user_info += f"Gender: {self.gender}\n"
        user_info += f"Height: {self.height}\n"
        user_info += f"Weight: {self.weight}\n"
        user_info += f"Allergies: {self.allergies}\n"
        user_info += f"Dietary Target: {self.target}\n"
        user_info += f"Eating Habits: {self.habit}\n"
        return user_info