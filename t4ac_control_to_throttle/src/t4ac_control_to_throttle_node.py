#!/usr/bin/env python
import rospy

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from t4ac_msgs.msg import CarControl


Kp = 0.175
Ki = 0.002

class Control_object:

    def __init__(self):
        self.actual_speed = 0
        self.cmd_vel = CarlaEgoVehicleControl()
        rospy.init_node('control_to_throttle_node',anonymous=True)
        rospy.Subscriber("/t4ac/control/cmd_vel", CarControl, self.callback_control)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.callback_status)
        self.carla_control_publisher = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd",CarlaEgoVehicleControl, queue_size=1)
        self.errorSum = 0

    def callback_control(self, data):
        errorSpeed = data.velocity-self.actual_speed #distance away from setpoint
        self.errorSum += (errorSpeed*Ki)
        if (self.errorSum > 0.5):
            self.errorSum = 0.5
        if (self.errorSum < -0.5):
            self.errorSum = -0.5

        throttle = ((errorSpeed*Kp) + self.errorSum)
        brake = 0.0
        if (data.velocity==0):
            self.errorSum = 0 #Reset PI

        if (throttle < 0):
            brake = -throttle 
            throttle = 0 
        if (throttle > 1):
            throttle = 1
        print(brake)

        self.cmd_vel.throttle = throttle
        self.cmd_vel.brake = brake
        self.cmd_vel.steer = -data.steer
        self.carla_control_publisher.publish(self.cmd_vel)


    def callback_status(self, data):
        self.actual_speed = data.velocity

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    Object = Control_object()
    Object.main()


    