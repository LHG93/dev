 #!/usr/bin/env python

import json


 #ros
import rospy
from geometry_msgs.msg import Twist, Transform, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry

#carla rosbridge
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.ego_vehicle import EgoVehicle
import carla_common.transforms as transforms
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel,\
    CarlaEgoVehicleControl, CarlaEgoVehicleStatus

from rospy_message_converter import json_message_converter
from datetime import datetime

def ROScall(data):      
    #self.json_str = json_message_converter.convert_ros_messaSge_to_json(data)
    gdata = json_message_converter.convert_ros_message_to_json(data)
    js = json.loads(gdata)
    #api_list = [ x for x in gdata['pose']['pose']['position'].keys()]
    a=js['pose']['pose']['position']['x']
    b=js['pose']['pose']['position']['y']
    print('x',a)
    print('y',b)
    #print( (api_list))

def RosSubCallbak_DataCarla(data):      
    #self.json_str = json_message_converter.convert_ros_message_to_json(data)
    rosdata = json_message_converter.convert_ros_message_to_json(data)
    js = json.loads(rosdata)

    vel_data=js['velocity']
    vel_data = float(vel_data * 3.6)
    throttle = js['control']['throttle']
    brake =  js['control']['brake']
    print('speed',vel_data)
    print('throttle',throttle)
    print('brake',brake)
    date_s = (datetime.now().strftime('%M:%S.%f'))
    print('time',date_s)

def main():
    rospy.init_node("carla_webdata", anonymous=False)

    rospy.Subscriber("/carla/ego_vehicle/odometry", 
    Odometry, ROScall, queue_size = 1)

    rospy.Subscriber("/carla/ego_vehicle/vehicle_status", 
    CarlaEgoVehicleStatus,RosSubCallbak_DataCarla, queue_size = 1)

    rospy.spin()
       



if __name__ == "__main__":
    #while True:
    try :
        main()
    
    except Exception as e:
        print( 'exception' , e )