import rospy
import numpy as np
import argparse
import matplotlib.pyplot as plt

from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from waypoint_list import WayPoints
from util import euler_to_quaternion, quaternion_to_euler

def run_model():

    x_list = []
    y_list = []
    wx_list = []
    wy_list = []
    rospy.init_node("model_dynamics")
    controller = vehicleController()

    waypoints = WayPoints()
    pos_list = waypoints.getWayPoints()
    pos_idx = 1

    target_x = pos_list[pos_idx][0]
    target_y = pos_list[pos_idx][1]
    prev_target_x = pos_list[pos_idx-1][0]
    prev_target_y = pos_list[pos_idx-1][1]
    target_orientation = np.arctan2(target_y-prev_target_y, target_x-prev_target_x)
    target_v = pos_list[pos_idx][2]
    ref_x = prev_target_x
    ref_y = prev_target_y
    ref_vx = target_v*np.cos(target_orientation)*0.01
    ref_vy = target_v*np.sin(target_orientation)*0.01

    
    
    # for i in range(len(pos_list)):
    #     wx_list.append(pos_list[i][0])
    #     wy_list.append(pos_list[i][1])

    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller.stop()
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  controller.getModelState()

        if not currState.success:
            continue

        # Compute relative position between vehicle and waypoints
        distToTargetX = abs(target_x - currState.pose.position.x)
        distToTargetY = abs(target_y - currState.pose.position.y)

        x_list.append(currState.pose.position.x)
        y_list.append(currState.pose.position.y)
        print(currState.pose.position.x,"wwwwwwwwwwwww")
        if (distToTargetX < 0.5 and distToTargetY < 0.5):
            # If the vehicle is close to the waypoint, move to the next waypoint
            prev_pos_idx = pos_idx
            pos_idx = pos_idx+1
            pos_idx = pos_idx % len(pos_list)
            target_x = pos_list[pos_idx][0]
            target_y = pos_list[pos_idx][1]
            wx_list.append(target_x)
            wy_list.append(target_y)
            prev_target_x = pos_list[pos_idx-1][0]
            prev_target_y = pos_list[pos_idx-1][1]
            target_orientation = np.arctan2(target_y-prev_target_y, target_x-prev_target_x)
            target_v = pos_list[pos_idx][2]
            ref_x = prev_target_x
            ref_y = prev_target_y
            ref_vx = target_v*np.cos(target_orientation)*0.01
            ref_vy = target_v*np.sin(target_orientation)*0.01

            print("reached",pos_list[prev_pos_idx][0],pos_list[prev_pos_idx][1],"next",pos_list[pos_idx][0],pos_list[pos_idx][1])

        # Perform interpolation to get the reference position of the vehicle
        ref_x = ref_x + ref_vx
        ref_y = ref_y + ref_vy

        
        controller.execute(currState, [ref_x, ref_y, target_orientation, target_v])
        #plot
    # x_list.append(currState.pose.position.x)
    # y_list.append(currState.pose.position.y)
    # print(x_list, y_list)
    plt.plot(x_list, y_list)
    plt.scatter(wx_list, wy_list, s=300,color='red')
    plt.savefig("scatter_points_order_01.png", bbox_inches='tight')
    plt.show()
        

if __name__ == "__main__":
    try:
        run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
