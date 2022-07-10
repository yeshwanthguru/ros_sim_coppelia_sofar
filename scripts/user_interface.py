import rospy
import time
from sofar_assignment.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    print("\n User interface to control the Robot")
    x = int(input("\nPress 1 to start the robot from the current position "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot  at the last position"))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot from the current position "))
            
if __name__ == '__main__':
    main()
