import sys
from controller import Controller

if __name__ == "__main__":

    #Select Platform
    if len(sys.argv) > 1:
        platform = sys.argv[1].upper()
        if platform == "TERMINAL":
                print("Terminal selected")
        elif platform ==  "ROBOT":
            print("Robot selected")
            import rospy
            rospy.init_node('nutrtion_node')
            rospy.loginfo("Process starting!")
        else:
            print("This platform is not supported. The Terminal will be used.")
            platform = "TERMINAL"
    else:
        print("No platform provided. The Terminal will be used.")
        platform = "TERMINAL"

    controller = Controller(platform)
    controller.start()