import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import math
from derivative import  dxdt



def check_vehicle_acc(a):
        a_max = np.sqrt(np.max(a*a))
        b=np.zeros(a.shape)
        N = 10
        for i in range(1,N):
            A = a[(i-1)*10:i*10]
            if np.sum(A)/10 >= 0.35*a_max :
                b[(i-1)*10:i*10] = 1
            elif np.sum(A)/10 < 0.35*a_max and np.sum(A)/10 > -0.35*a_max:
                b[(i-1)*10:i*10] = 2
            elif np.sum(A)/10 <= -0.35*a_max:
                b[(i-1)*10:i*10] = 3
        return b

def check_vehicle_activity(a,v):
    a = np.where(a[:-1]>0,1,-1)
    b=np.zeros(a.shape)
    N = 11
    for i in range(1,N):
        A = a[(i-1)*10:i*10]
        if np.sum(A) >= 3 :
            b[(i-1)*10:i*10] = 1
        elif np.sum(A) < 3 and np.sum(A) > -3:
            if sum(v[(i-1)*10:i*10])/10 < 0.5:
                b[(i-1)*10:i*10] = 0
            else:
                b[(i-1)*10:i*10] = 2
        elif np.sum(A) <= -3:
            if sum(v[(i-1)*10:i*10])/10 < 0.8:
                b[(i-1)*10:i*10] = 0 #stoped
            else:
                b[(i-1)*10:i*10] = 3 # deacellerating
    return b




class Draw():

    def __init__(self):
        self.figsize_speed = (5,5)
        self.figsize_yaw = (5,5)
        self.fig = None

    

    


    def vehicle_speed(self,scene,objects_of_interest,include_action=False,get_car=True):

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
        self.fig = plt.figure(figsize=self.figsize_speed)
        v = np.convolve(v, np.ones(6)/6,mode='same')
        # acceleration calculator
        a = (np.append(v[1:],0) - v[0:])/(1/91)
        b = check_vehicle_activity(a,v)
        print(len(b),len(time))
        #draw plot
        plt.plot(time, v)
        plt.ylim([0,max(v)+3])
        plt.xlabel(r'time $[s]$')
        plt.ylabel(r'Velocity $[\frac{m}{s}]$')
        # include actions if set to True
        # by default set to False
        if include_action:
            plt.scatter(time[1:], b,marker='.')
            breaking = np.where(b==3)
            accelerating = np.where(b==1)
            cruising = np.where(b==2)
            plt.scatter(time[breaking], b[breaking],marker='.',label="Breaking")
            plt.scatter(time[accelerating], b[accelerating],marker='.',label="Accelerating")
            plt.scatter(time[cruising], b[cruising],marker='.',label="Cruising")
            plt.legend()
        self.fig.tight_layout()

        plt.show()
        plt.close()


    def yaw_and_yaw_rate(self,scene,objects_of_interest,include_action=False,get_car=True):

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

        self.fig = plt.figure(figsize=self.figsize_yaw)
        #yaw_rate
        omega = dxdt(heading, time, kind="finite_difference", k=1)

        c = check_vehicle_acc(omega)
        #draw plot
        plt.plot(time,heading)
        plt.plot(time, omega)
        plt.ylim([-2*np.pi,2*np.pi])
        plt.xlabel(r'time $[s]$')
        plt.ylabel(r'Yaw and Yaw-rate $[\frac{m}{s}]$')
        # include actions if set to True
        # by default set to False
        if include_action:
            right = np.where(c==3)
            left = np.where(c==1)
            straight = np.where(c==2)
            plt.scatter(time[right], c[right],marker='.',label="Right")
            plt.scatter(time[left], c[left],marker='.',label="Left")
            plt.scatter(time[straight], c[straight],marker='.',label="Straight")
            plt.legend()

        self.fig.tight_layout()
        plt.show()
        plt.close()

    # draw the tracks of the specified dynamic object in scenario
    def draw_vehicle_tracks(self,object_states):
        object_points = {"x":[],"y":[]}
        for state in object_states:
            object_points["x"].append(state.center_x)
            object_points["y"].append(state.center_y)
        plt.scatter(object_points['x'],object_points['y'],color=np.random.rand(3,),marker=".",alpha=0.8)

    # draw crosswalks present in scenario
    def draw_scenario_crosswalks(self,Map,crosswalks):

        for id in crosswalks:
            for road_construct in Map:
                if road_construct.id == id:
                    crosswalk_points = {"x":[],"y":[]}
                    for crosswalk in road_construct.crosswalk.polygon:
                        
                        crosswalk_points['x'].append(crosswalk.x)
                        crosswalk_points['y'].append(crosswalk.y)
                    plt.fill(crosswalk_points['x'],crosswalk_points['y'],color='green',alpha=0.2)
    
    def draw_map(self,Map,lane_id):
    
        for id in lane_id:
            map_points = {"x":[],"y":[]}
            for road_construct in Map:
                if road_construct.id == id:
                    for polyline in road_construct.lane.polyline:
                        map_points["x"].append(polyline.x)
                        map_points["y"].append(polyline.y)
            plt.plot(map_points['x'],map_points['y'],color=np.random.rand(3,),alpha=0.8)

    def distance_traveled(self,object):
        from numpy import diff
        
        S = 0
        s = []
        t = []
        ds1 = []
        for i,states in enumerate(object.states):
            dx = object.states[i].center_x - object.states[i-1].center_x
            dy = object.states[i].center_y - object.states[i-1].center_y
            ds = dx*dx + dy*dy
            ds1.append(ds)
            S += ds
            s.append(S)
            t.append(i)
        print(len(s),len(ds1))

        actions=[]

        for i in range(0,len(s)):
            if ds1[i] - ds1[i-1] > 0.1*ds1[i-1]:
                actions.append(2)

            elif ds1[i] - ds1[i-1] < 0:
                actions.append(-2)
            
            elif ds1[i] - ds1[i-1] < 0.1*ds1[i-1] and s[i]-s[i-1] < 0.1*ds1[i-1]:
                actions.append(0)
            elif ds1[i] - ds1[i-1] < 0.1*ds1[i-1] and s[i]-s[i-1] > 0.1*ds1[i-1]:
                actions.append(1)
        print(len(t),len(actions))
        plt.scatter(t[:],actions)
        plt.plot(t,s)