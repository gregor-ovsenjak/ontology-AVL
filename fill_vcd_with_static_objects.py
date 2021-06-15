import unittest
import os
import vcd.core as core
import vcd.schema as schema
import vcd.types as types
import vcd.utils as utils
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
import math
vcd_version_name = "vcd" + schema.vcd_schema_version.replace(".", "")




def check_vehicle_acc(a):
        a_max = np.sqrt(np.max(a*a))
        b=np.zeros(a.shape)
        N = 10
        for i in range(1,N):
            A = a[(i-1)*10:i*10]
            if np.sum(A)/10 >= 0.35*a_max :
                b[(i-1)*10:i*10] = 0 # left
            elif np.sum(A)/10 < 0.35*a_max and np.sum(A)/10 > -0.35*a_max:
                b[(i-1)*10:i*10] = 1 # straight
            elif np.sum(A)/10 <= -0.35*a_max:
                b[(i-1)*10:i*10] = 2 # right
        return b




def list_of_points_for_crosswalk(Polygon):
    points = []
    for x,y in Polygon.exterior.coords:
        points.append(x)
        points.append(y)
    return points


def check_vehicle_activity(a,v):
    a = np.where(a[:-1]>0,1,-1)
    b=np.zeros(a.shape)
    N = 11
    for i in range(1,N):
        A = a[(i-1)*10:i*10]
        if np.sum(A) >= 3 :
            b[(i-1)*10:i*10] = 1 #accelleration
        elif np.sum(A) < 3 and np.sum(A) > -3:
            if sum(v[(i-1)*10:i*10])/10 < 0.8:
                b[(i-1)*10:i*10] = 0 #stoped
            else:
                b[(i-1)*10:i*10] = 2 # cruising
        elif np.sum(A) <= -3:
            
            if sum(v[(i-1)*10:i*10])/10 < 0.8:
                b[(i-1)*10:i*10] = 0 #stoped
            else:
                b[(i-1)*10:i*10] = 3 # deacellerating
    return b



def yaw_and_yaw_rate(scene,objects_of_interest,get_car=True):
    from derivative import  dxdt
    if get_car:
            object_type = 1
    else: 
            object_type = 2

    heading = []
    time = np.arange(0.0,9.1,0.1)
    for j,seconds in enumerate(scene.timestamps_seconds):
        for object_of_interest in objects_of_interest:
            for i,tracks in enumerate(scene.tracks) :
                if object_of_interest == tracks.id :
                    if tracks.object_type == object_type:
                        heading.append(tracks.states[j].heading)

    #yaw_rate
    omega = dxdt(heading, time, kind="finite_difference", k=1)
    return omega



def vehicle_speed(scene,objects_of_interest,get_car=True):

            if get_car:
                object_type = 1
            else: 
                object_type = 2
            v = []
            time = np.arange(0.0,9.1,0.1)
            for j,seconds in enumerate(scene.timestamps_seconds):
                for object_of_interest in objects_of_interest:
                    for i,tracks in enumerate(scene.tracks) :
                        if object_of_interest == tracks.id :
                            if tracks.object_type == object_type:
                                v.append(math.sqrt(tracks.states[j].velocity_x**2 + tracks.states[j].velocity_y**2))

       
            # reduce noise of speed
            v = np.convolve(v, np.ones(6)/6,mode='same')
            # acceleration calculator
            a = (np.append(v[1:],0) - v[0:])/(1/91)
            actions = check_vehicle_activity(a,v)
            return actions



vcd = core.VCD()

ont_uid_0 = vcd.add_ontology("http://vcd.vicomtech.org/ontology/automotive#")

class Filler:


    def __init__(self):
        self.VCD = vcd
        self.uid_car1 = self.VCD.add_object(name="CAr", semantic_type="Car", frame_value=None, ont_uid=ont_uid_0)
        self.uid_road = self.VCD.add_object(name="Road1",
                	                       semantic_type="Road",
                                           frame_value=(0,90),
                                           ont_uid=ont_uid_0)
        self.vehicle_longitudinal_actions = []
        self.vehicle_turning_actions = []
        self.frames_when_Vehicle_crosses_junction = []
        self.frames_when_Vehicle_crosses_crosswalks = {}
        self.frames_when_Vehicle_approaches_junction = {"exists":False, "frames":[]}


    def fill(self,static,EgoCar,scenario_number):
        print("----------start--fill--------------------")
        print("...")
        
        self.VCD.add_metadata_properties({"cnl_text": "Generating data from proto file : 00052-of-00150 --> scenario number : {a}".format(a=scenario_number)})
        which_intersection = ["zero","one","two","Three","Four","Five","Six"]
        # add road object --> always true
        print("-----------------------------------------")
        print("Adding road...")
        print("-----------------------------------------")

        # add an intersection if it exists
        print("-----------------------------------------")
        print("Adding junction...")
        print("-----------------------------------------")
        if static.junction["exists"] :
            uid_junction = self.VCD.add_object(name="{a}WayIntersection".format(a=which_intersection[static.junction["type"]]),
                                         semantic_type="{a}-way intersection".format(a=which_intersection[static.junction["type"]]),
                                         frame_value=(0,90),
                                         ont_uid=ont_uid_0)
            # add intersection exterior boundary points
            self.VCD.add_object_data(uid=uid_junction,
                                    object_data=types.poly2d(
                                        name="Exterior-points",
                                        val=list_of_points_for_crosswalk(static.junction["Polygon"]),
                                        mode=types.Poly2DType.MODE_POLY2D_ABSOLUTE,
                                        closed=True))
            
            print("-----------------------------------------")
            print("Adding crosswalks...")
            print("-----------------------------------------")
            crosswalks_part_of_junction = {}
            #add crosswalks that are part of the junction/intersection
            for crosswalks in static.junction["Junction_crosswalks"]:
                crosswalks_part_of_junction["{a}".format(a=crosswalks)]=self.VCD.add_object(name="Crosswalk{a}".format(a=crosswalks),
                                                    semantic_type="Crosswalk",
                                                    frame_value=(0,90),
                                                    ont_uid=ont_uid_0)


            # add crosswalk exterior points
            for i,crosswalk in enumerate(static.junction["Junction_crosswalks"]):
                
                self.VCD.add_object_data(uid=crosswalks_part_of_junction["{a}".format(a=crosswalk)],
                                    object_data=types.poly2d(
                                        name="Exterior-points",
                                        val=static.junction["Junction_crosswalks"][crosswalk],
                                        mode=types.Poly2DType.MODE_POLY2D_ABSOLUTE,
                                        closed=True))
            print("-----------------------------------------")
            print("Adding junction relations...")
            print("-----------------------------------------")
            # add relation isPartOf with subject as Intersection and object as Crosswalk
            for i,crosswalk in enumerate(crosswalks_part_of_junction):
                self.VCD.add_relation_object_object(name="isPartOf{a}".format(a = i),
                                                    semantic_type="isPartOf", 
                                                    object_uid_1=crosswalks_part_of_junction[crosswalk], 
                                                    object_uid_2=uid_junction,
                                                    ont_uid=ont_uid_0)

            print("-----------------------------------------")
            print("Adding dynamic actions...")
            print("-----------------------------------------")
            # Adding dynamic actions based on interaction with static objects
            distance_0 = (EgoCar.states[0].center_x -static.junction["Center_point"].x)**2 + (EgoCar.states[0].center_y -static.junction["Center_point"].y)**2
            for i,state in enumerate(EgoCar.states):
                car_position = Point(state.center_x,state.center_y)
                center_of_junction = static.junction["Center_point"]
                distance_1 = (car_position.x -center_of_junction.x)**2 + (car_position.y -center_of_junction.y)**2
                # if car approaching the junction but not yet crossing
                if distance_1 <= distance_0 and not static.junction["Polygon"].contains(car_position):
                    self.frames_when_Vehicle_approaches_junction["exists"] = True
                    self.frames_when_Vehicle_approaches_junction["frames"].append(i)
                # if point inside a junction
                elif static.junction["Polygon"].contains(car_position) : 
                    self.frames_when_Vehicle_crosses_junction.append(i)
                distance_0 = distance_1
            

            # Crossing action
            uid_action_crossing = vcd.add_action(name="CrossingAction",
                                                semantic_type="Crossing",
                                                frame_value=[(self.frames_when_Vehicle_crosses_junction[0],
                                                             self.frames_when_Vehicle_crosses_junction[-1])],
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name="Crosses",
                                                semantic_type="isSubjectOfAction",  
                                                object_uid=self.uid_car1,
                                                action_uid=uid_action_crossing,
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name="Crosses",
                                                semantic_type="isObjectOfAction", 
                                                object_uid=uid_junction,
                                                action_uid=uid_action_crossing,
                                                ont_uid=ont_uid_0)



            # if vehicle approaches the intersection addd approaching action and 
            # also add a crossing event
            if self.frames_when_Vehicle_approaches_junction["exists"] :
                uid_action_approaching = vcd.add_action(name="ApproachingAction",
                                                semantic_type="Approaching",
                                                frame_value=[(self.frames_when_Vehicle_approaches_junction["frames"][0],
                                                             self.frames_when_Vehicle_approaches_junction["frames"][-2])],
                                                ont_uid=ont_uid_0)
                self.VCD.add_relation_object_action(name="Approaching",
                                                    semantic_type="isSubjectOfAction",  
                                                    object_uid=self.uid_car1,
                                                    action_uid=uid_action_approaching,
                                                    ont_uid=ont_uid_0)
                self.VCD.add_relation_object_action(name="Approaching",
                                                    semantic_type="isObjectOfAction", 
                                                    object_uid=uid_junction,
                                                    action_uid=uid_action_approaching,
                                                    ont_uid=ont_uid_0)
                #cross event
                uid_event_cross = self.VCD.add_event(name="CrossEvent",
                                                    semantic_type="Cross",
                                                    frame_value=self.frames_when_Vehicle_approaches_junction["frames"][-1])
                self.VCD.add_relation_subject_object(name="", 
                                                    semantic_type="isSubjectOfEvent", 
                                                    subject_type=core.ElementType.object, 
                                                    subject_uid=self.uid_car1,
                                                    object_type=core.ElementType.event, 
                                                    object_uid=uid_event_cross)
                self.VCD.add_relation_subject_object(name="", 
                                                    semantic_type="isObjectOfEvent", 
                                                    subject_type=core.ElementType.object,
                                                    subject_uid=uid_junction,
                                                    object_type=core.ElementType.event, 
                                                    object_uid=uid_event_cross)
                vcd.add_relation_subject_object(name="", 
                                                semantic_type="causes", 
                                                subject_type=core.ElementType.event,
                                                subject_uid=uid_event_cross,
                                                object_type=core.ElementType.action, 
                                                object_uid=uid_action_crossing)
            # adding crossing the crosswalk action
            for id in static.interacting_crosswalks:
                self.frames_when_Vehicle_crosses_crosswalks["{a}".format(a=id)] = []
                for i,state in enumerate(EgoCar.states):
                    car_position = Point(state.center_x,state.center_y)

                    if static.interacting_crosswalks_polygons["{a}".format(a=id)].contains(car_position):
                        self.frames_when_Vehicle_crosses_crosswalks["{a}".format(a=id)].append(i)
            for crosswalk_id in self.frames_when_Vehicle_crosses_crosswalks:
                for crosswalk in crosswalks_part_of_junction:
                    if crosswalk_id == crosswalk:
                        uid_action_crossing1 = vcd.add_action(name="CrossingAction{a}".format(a=crosswalk_id),
                                                semantic_type="Crossing",
                                                frame_value=[(self.frames_when_Vehicle_crosses_crosswalks["{a}".format(a=crosswalk_id)][0],
                                                             self.frames_when_Vehicle_crosses_crosswalks["{a}".format(a=crosswalk_id)][-1])],
                                                ont_uid=ont_uid_0)
                        self.VCD.add_relation_object_action(name="Crosses{a}".format(a=crosswalk_id),
                                                            semantic_type="isSubjectOfAction",  
                                                            object_uid=self.uid_car1,
                                                            action_uid=uid_action_crossing1,
                                                            ont_uid=ont_uid_0)
                        self.VCD.add_relation_object_action(name="Crosses{a}".format(a=crosswalk_id),
                                                            semantic_type="isObjectOfAction", 
                                                            object_uid=crosswalks_part_of_junction[crosswalk],
                                                            action_uid=uid_action_crossing1,
                                                            ont_uid=ont_uid_0)




        print("...")
        print("------------end--fill--------------------")
        if not os.path.isfile('./VCD_files/WAYMO/' + vcd_version_name + '_scenario_{a}_proto_{b}.json'.format(a=scenario_number,b=52)):
            self.VCD.save("./VCD_files/WAYMO/"+vcd_version_name + '_scenario_{a}_proto_{b}.json'.format(a=scenario_number,b=52), validate=True)



    def fill_with_moving_actions(self,scene,objects_of_interest):
        actions = {}
        self.vehicle_longitudinal_actions = vehicle_speed(scene,objects_of_interest,get_car=True)
        previous_action = 10
        new_action = True
        numb_of_new_action = 0
        for i,action in enumerate(self.vehicle_longitudinal_actions):
            if action != previous_action:
                numb_of_new_action +=1
                actions["{a}".format(a= numb_of_new_action)] = {"which":action,"frames":[]}
            elif action == previous_action:
                actions["{a}".format(a= numb_of_new_action)]["frames"].append(i)
            previous_action = action
        
        name_of_action = ["Stopped","Moving maneuver","Driving straight","Decelerating"]
        print(actions)
        for i,action in enumerate(actions):
            uid_action = vcd.add_action(name=name_of_action[int(actions[action]["which"])]+"{a}".format(a=i),
                                                semantic_type=name_of_action[int(actions[action]["which"])],
                                                frame_value=[(actions[action]["frames"][0],
                                                             actions[action]["frames"][-1])],
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name=name_of_action[int(actions[action]["which"])],
                                                semantic_type="isSubjectOfAction",  
                                                object_uid=self.uid_car1,
                                                action_uid=uid_action,
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name=name_of_action[int(actions[action]["which"])],
                                                semantic_type="isObjectOfAction", 
                                                object_uid=self.uid_road,
                                                action_uid=uid_action,
                                                ont_uid=ont_uid_0)

    
    def fill_with_turning_actions(self,scene,objects_of_interest):
        actions = {}
        omega = yaw_and_yaw_rate(scene,objects_of_interest)
        self.vehicle_turning_actions = check_vehicle_acc(omega)
        self.vehicle_turning_actions[-1] = self.vehicle_turning_actions[-2]
        previous_action = 10
        numb_of_new_action = 0
        for i,action in enumerate(self.vehicle_turning_actions):
            if action != previous_action:
                numb_of_new_action +=1
                actions["{a}".format(a= numb_of_new_action)] = {"which":action,"frames":[]}
            elif action == previous_action:
                actions["{a}".format(a= numb_of_new_action)]["frames"].append(i)
            previous_action = action
        
        
        name_of_action = ["Turning left","Driving straight","Turning right"]
        for action in actions:
            uid_action = vcd.add_action(name=name_of_action[int(actions[action]["which"])],
                                                semantic_type=name_of_action[int(actions[action]["which"])],
                                                frame_value=[(actions[action]["frames"][0],
                                                             actions[action]["frames"][-1])],
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name=name_of_action[int(actions[action]["which"])],
                                                semantic_type="isSubjectOfAction",  
                                                object_uid=self.uid_car1,
                                                action_uid=uid_action,
                                                ont_uid=ont_uid_0)
            self.VCD.add_relation_object_action(name=name_of_action[int(actions[action]["which"])],
                                                semantic_type="isObjectOfAction", 
                                                object_uid=self.uid_road,
                                                action_uid=uid_action,
                                                ont_uid=ont_uid_0)


            





#print(self.VCD.stringify())











    