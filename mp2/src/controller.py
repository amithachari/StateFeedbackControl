import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
from util import euler_to_quaternion, quaternion_to_euler

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def execute(self, currentPose, referencePose):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   referencePose: list, the reference state of the vehicle,
        #       the element in the list are [ref_x, ref_y, ref_theta, ref_v]
        # Output: None

        # TODO: Implement this function

        # define referencePose list variables
        ref_x = referencePose[0]
        ref_y = referencePose[1]
        ref_theta = referencePose[2]
        ref_v = referencePose[3]

        # define currentPose variables
        x_B = currentPose.pose.position.x
        y_B = currentPose.pose.position.y
        currentEuler = quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)
        theta_B = currentEuler[2]
        v_B_x = currentPose.twist.linear.x  # not sure how to find v_B
        v_B_y = currentPose.twist.linear.y
        v_B = np.sqrt(v_B_x**2 + v_B_y**2)
        #v_B = np.array([v_B_x, v_B_y])
        #v_B.transpose()

        # define delta matrix
        delta_x = (np.cos(ref_theta) * (ref_x - x_B)) + (np.sin(ref_theta) * (ref_y - y_B))
        delta_y = (-1 * np.sin(ref_theta) * (ref_x - x_B)) + (np.cos(ref_theta) * (ref_y - y_B))
        delta_theta = ref_theta - theta_B
        delta_v = ref_v - v_B  # not sure how to find v_B

        error = np.array([delta_x, delta_y, delta_theta, delta_v])
        error.transpose()

        # define k matrix (gains)
        k_x = 0.3  # [0.1, 0.5, 1.0]
        k_y = 0.2  # [0.05, 0.1, 0.5]
        k_v = 0.8  # [0.5, 0.1, 1.5]
        k_theta = 1.0  # [0.8, 1.0, 2.0]
        zeros = 0  # [0, 0, 0]

        k = np.array([[k_x, zeros, zeros, k_v],
                      [zeros, k_y, k_theta, zeros]])

        # calculate u matrix
        u = np.matmul(k, error)  # u: control input to vehicle

        # compute velocity and steering angle
        velocity = u[0]
        steering_angle = u[1]

        # Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = velocity
        newAckermannCmd.steering_angle = steering_angle

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)


    def setModelState(self, currState, targetState, vehicle_state = "run"):
        control = self.rearWheelFeedback(currState, targetState)
        self.controlPub.publish(control)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
