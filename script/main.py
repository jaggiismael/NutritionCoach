import sys
from controller import Controller

if __name__ == "__main__":

    #Select Platform
    if len(sys.argv) > 1:
        platform = sys.argv[1].upper()
        match platform:
            case "TERMINAL":
                print("Terminal selected")
            case "ROBOT":
                print("Robot selected")
            case _:
                print("This platform is not supported. The Terminal will be used.")
                platform = "TERMINAL"
    else:
        print("No platform provided. The Terminal will be used.")
        platform = "TERMINAL"

    controller = Controller(platform)
    controller.start()