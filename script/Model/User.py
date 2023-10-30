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
        user_info = f"Firstname: {self.firstname} "
        user_info += f"Lastname: {self.lastname} "
        user_info += f"Age: {self.age} "
        user_info += f"Gender: {self.gender} "
        user_info += f"Height: {self.height} "
        user_info += f"Weight: {self.weight} "
        user_info += f"Allergies: {self.allergies} "
        user_info += f"Dietary Target: {self.target} "
        user_info += f"Eating Habits: {self.habit} "
        return user_info