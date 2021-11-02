
from dateutil.relativedelta import TH
from jedi.inference.utils import reraise_uncaught
from prompt_toolkit.layout.dimension import D
import carla

from global_route_planner import  GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO 

import time
import math
import numpy as np
import controller
import rospy
from decimal import Decimal
from std_msgs.msg import Float64,Float32MultiArray, Float64MultiArray

from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel,\
    CarlaEgoVehicleControl, CarlaEgoVehicleStatus 

vehicle_cmd = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1)
pub = rospy.Publisher('/cmd_vel', Float64MultiArray, queue_size=1)
vehicle_ctrl_info = CarlaEgoVehicleControl()
#vehicle_num = 259
#spawn_points[346].location
client = carla.Client("localhost", 2000)
client.set_timeout(10)

#world = client.load_world('Town04')
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

vehicle = world.get_actors().filter('vehicle.toyota.prius')
vehicle_num = vehicle[0].id
vehicle = world.get_actors().find(vehicle_num)
spawnPoint=carla.Transform(carla.Location(spawn_points[120].location),carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.000000)) #120
vehicle = carla.Actor.set_transform(vehicle,spawnPoint)
vehicle = world.get_actors().find(vehicle_num)

def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


#Location(x=-410.0, y=16.0, z=0.275307 )carla.Location(spawn_points[123].location //carla.Location(x=-400.0, y=30.0, z=0.275307 )
def spawn_vehicle(spawnPoint=carla.Transform(carla.Location(spawn_points[120].location),carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.000000))):
    
    """
    
    This function spawn vehicles in the given spawn points. If no spawn 
    point is provided it spawns vehicle in this 
    position x=27.607,y=3.68402,z=0.02
    """
    
    spawnPoint=spawnPoint
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.find('vehicle.toyota.prius')

    vehicle = world.spawn_actor(bp, spawnPoint)
    return vehicle

#i=0
#j=0
#control=0
#distance_v =0
#vehicle_loc=0
#target=0

def drive_through_plan(planned_route,vehicle,speed,PID):
    """
    This function drives throught the planned_route with the speed passed in the argument
    
    """
    #global i,j,control,distance_v,target
    #if j ==0 :
    target=planned_route[0]
    #j = 1
    i=0
    end = 0
    position = Float64MultiArray()
    veldata = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    while not rospy.is_shutdown():
        start = time.time()
        #veldata =[1,2,3,4,6]
        current_speed = get_speed(vehicle)
        vehicle_loc= vehicle.get_location()
        #vehicle_loc= spawnPoint.get_location()
        distance_v =find_dist_veh(vehicle_loc,target)
        control = PID.run_step(speed,target)
        #vehicle.apply_control(control)
  
        '''
        data_total = sublodata.GetRosdata()
        if data_total.__len__() > 0:
            print(sublodata.GetRosdata())
            vehicle_ctrl_info.steer=    float(data_total[0])
            vehicle_ctrl_info.throttle=    float(data_total[1])
            vehicle_ctrl_info.hand_brake = float(data_total[2])
            vehicle_cmd.publish(vehicle_ctrl_info)
        '''
        veldata[0] = (round(target.transform.location.x, 7))
        veldata[1] = (round(target.transform.location.y, 7))
        veldata[2] = (round(vehicle_loc.x, 7))
        veldata[3] = (round(vehicle_loc.y, 7))
        veldata[4] = (round(target.transform.location.x, 7)) - (round(vehicle_loc.x, 7))
        veldata[5] = (round(target.transform.location.y, 7)) - (round(vehicle_loc.y, 7))
        veldata[6] = (round(vehicle_loc.z, 7))

        veldata[7] = (round(current_speed, 7))
        veldata[8] = 40.00#(40.0)
        veldata[9] = (round(vehicle_loc.z, 7))
        
        veldata[10] = (round(current_speed, 7))
        veldata[11] = (round(distance_v, 7))
        veldata[12] = 40.00#(40.0)
        
        #veldata[8] = (round(target.transform.location.x, 7))
        #veldata[9] = (round(target.transform.location.y, 7))
        #veldata[10] = (round(vehicle_loc.x, 7))
        #veldata[11] = (round(vehicle_loc.y, 7))

        position.data = veldata
        #print(position.data)
        #print(position.data)
        pub.publish(position)
        #vehicle.apply_control(control)
        if i==(len(planned_route)-1):
            print("last waypoint reached")
            control = PID.run_step(0,planned_route[len(planned_route)-1])
            #vehicle.apply_control(control)
            i=0
            j = 0
            #time.sleep(2)
            break

        if (distance_v<3.5):
            control = PID.run_step(speed,target)
            #vehicle.apply_control(control)
            i=i+1
            target=planned_route[i]
            
            print("vehicle_Debugdistance_v<3.5")

        if (distance_v>8):
            print("vasdasfsafsdf_asdsadasdfasfdsf")
            break
        
#-00000000000000000000000000000000000000000000000000000000000
#-00000000000000000000000000000000000000000000000000000000000
#-00000000000000000000000000000000000000000000000000000000000
#-00000000000000000000000000000000000000000000000000000000000
#-00000000000000000000000000000000000000000000000000000000000
        #print('vehicle_loc',vehicle_loc.x,vehicle_loc.y)
        '''
        if((vehicle_loc.x<20 or vehicle_loc.x<= -2) and (vehicle_loc.y<6 or vehicle_loc.y<= -2)):#and result_final[1])  and vehicle_loc.y<-1
        #if((vehicle_loc.x>=380.0 and vehicle_loc.x<= 397.0) and ( vehicle_loc.y<= 331.0 and vehicle_loc.y>= 318.0)):#and result_final[1])  and vehicle_loc.y<-1
            if (distance_v<3.5):
                control = PID.run_step(speed,target)
                #vehicle.apply_control(control)
                i=i+1
                target=planned_route[i]
                print(i)
                print("ML<3.5")

        else:
            vehicle.apply_control(control)
            if i==(len(planned_route)-1):
                print("last waypoint reached")
                control = PID.run_step(0,planned_route[len(planned_route)-1])
                #vehicle.apply_control(control)
                i=0
                j = 0
                #time.sleep(2)
                break

            
            if (distance_v<3.5):
    
                print("pid_<3.5")
                control = PID.run_step(speed,target)
                #print('control',control)
                vehicle.apply_control(control)
                i=i+1
                target=planned_route[i]
                print(i,'',start-end)
                #print("PID_vehicle_Debugdistance_v<3.5")
                end = start 
        
        if (distance_v>8):
            print("vasdasfsafsdf_asdsadasdfasfdsf")
            break
        '''

        """veldata[0] = target.transform.location.x#(round(target.transform.location.x, 3))
        veldata[1] = target.transform.location.y#(round(target.transform.location.y, 3))
        veldata[2] = vehicle_loc.x#(round(vehicle_loc.x, 3))
        veldata[3] = vehicle_loc.y#(round(vehicle_loc.y, 3))
        veldata[4] = current_speed#(round(current_speed, 3))
        veldata[5] = distance_v#(round(distance_v, 3))
        veldata[6] = 40#(40.0)"""
        veldata[0] = (round(target.transform.location.x, 7))
        veldata[1] = (round(target.transform.location.y, 7))
        veldata[2] = (round(vehicle_loc.x, 7))
        veldata[3] = (round(vehicle_loc.y, 7))
        veldata[4] = (round(target.transform.location.x, 7)) - (round(vehicle_loc.x, 7))
        veldata[5] = (round(target.transform.location.y, 7)) - (round(vehicle_loc.y, 7))
        veldata[6] = (round(vehicle_loc.z, 7))

        veldata[7] = (round(current_speed, 7))
        veldata[8] = 40.00#(40.0)
        veldata[9] = (round(vehicle_loc.z, 7))
        
        veldata[10] = (round(current_speed, 7))
        veldata[11] = (round(distance_v, 7))
        veldata[12] = 40.00#(40.0)
        
        #veldata[8] = (round(target.transform.location.x, 7))
        #veldata[9] = (round(target.transform.location.y, 7))
        #veldata[10] = (round(vehicle_loc.x, 7))
        #veldata[11] = (round(vehicle_loc.y, 7))

        position.data = veldata
        #print(position.data)
        #print(position.data)
        pub.publish(position)
        

def find_dist(start ,end ):
    dist = math.sqrt( (start.transform.location.x - end.transform.location.x)**2 + (start.transform.location.y - end.transform.location.y)**2 )

    return dist



def find_dist_veh(vehicle_loc,target):
    dist = math.sqrt( (target.transform.location.x - vehicle_loc.x)**2 + (target.transform.location.y - vehicle_loc.y)**2 )
    
    return dist
    

    
    

        


def setup_PID(vehicle):
    
    
    """
    This function creates a PID controller for the vehicle passed to it 
    """
    
    
    args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07

            ,'dt': 1.0 / 10.0
            }

    args_long_dict = {
            'K_P': 1,
            'K_D': 0.0,
            'K_I': 0.75
            ,'dt': 1.0 / 10.0
            }

    PID=controller.VehiclePIDController(vehicle,args_lateral=args_lateral_dict,args_longitudinal=args_long_dict)
    
    return PID



#rospy.init_node("pub_hglee_pid", anonymous=False)



def find_waypoind(w1,w2) :
    amap = world.get_map() 
    sampling_resolution = 2
    dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()
    spawn_points = world.get_map().get_spawn_points()
    #a = carla.Location(spawn_points[a].location)#351 120
    a = w1.get_location()
    b = carla.Location(spawn_points[w2].location)#367 123
    w_route = grp.trace_route(a, b) 

	#world.debug.draw_point(a,color=carla.Color(r=0, g=0, b=255),size=0.4 ,life_time=12.0)
	#world.debug.draw_point(b,color=carla.Color(r=0, g=0, b=255),size=0.4 ,life_time=12.0)
    wps=[]

    for i in range(len(w_route)):
        wps.append(w_route[i][0])
		#world.debug.draw_point(w_route[i][0].transform.location,color=carla.Color(r=255, g=255, b=0),size=0.2 ,life_time=300.0)
    
    return wps
'''
spawnPoint=carla.Transform(carla.Location(x=-400.0, y=30.0, z=0.275307 ),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))
vehicles = world.get_actors().find(vehicle_num)
vehicles = carla.Actor.set_transform(vehicles,spawnPoint)
vehicles = world.get_actors().find(vehicle_num)
'''

#spawnPoint=carla.Transform(carla.Location(x=-400.0, y=30.0, z=0.275307 ),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

#vehicle=spawn_vehicle()
class Subxy:
    # gdata =list()
    def __init__(self):
        self.json_str =""
        self.gdata = []

    def GetRosdata(self):
        #return self.json_str
        return self.gdata

    def ROSxy(self,data):   
        #self.json_str = json_message_converter.convert_ros_messaSge_to_json(data)
        self.gdata = data.data
 

sublodata=Subxy()

def main():
    #PID=setup_PID(vehicles)
    rospy.init_node("pub_hglee_tbqkf1", anonymous=False)
    z=1
    speed=40
    time.sleep(4)
    x72 = Decimal (carla.Location(spawn_points[120].location).x)
    y72  = Decimal (carla.Location(spawn_points[120].location).y)
    x56 = Decimal (carla.Location(spawn_points[53].location).x)
    y56 = Decimal (carla.Location(spawn_points[53].location).y)
    rospy.Subscriber("/cmd_vel1",Float64MultiArray,sublodata.ROSxy, queue_size = 10)
    while not rospy.is_shutdown():
        vx = Decimal(vehicle.get_location().x)
        vy = Decimal(vehicle.get_location().y)
        
        if( z== 1 ):
            j=0
            i=0
            z=0
            wps = find_waypoind(vehicle,53)
            print('debug72')
        #elif(  round(vx, 2) == round(x56, 2) and  round(vx, 2) == round(y56, 2) and z==0):
        elif( z== 0 ):
            j=0
            i=0
            z=1
            wps = find_waypoind(vehicle,120)
            print('debug56')
        
        PID=setup_PID(vehicle)
        drive_through_plan(wps,vehicle,speed,PID)
 

if __name__ == "__main__":
    try :
        main()

    except Exception as e:
        #vbreak
        print( 'exception' , e )
'''
client = carla.Client("localhost", 2000)
world = client.get_world()
vehicles = world.get_actors().filter('vehicle.toyota.prius')
print(vehicles)'''

