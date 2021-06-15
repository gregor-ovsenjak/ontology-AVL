 # library for detecting if Point is in polygon
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def crosswalk_points(map, id):
    polygon_1 = []
    for road_part in map:
        if road_part.id == id : 
            for polygon in road_part.crosswalk.polygon:
                        polygon_1.append(polygon.x)
                        polygon_1.append(polygon.y)
    return polygon_1


# this class detects all static objects that the Actors interact with.
class Static():

    def __init__(self,Vehicle, Map):
        self.Vehicle = Vehicle
        self.Map = Map
        self.interacting_crosswalks = []
        self.interacting_crosswalks_polygons = {}
        self.junction = {"exists":False,"type":0}
        self.stop_signs = False
        self.road = True



    def fill_up_objects(self):
        self.find_id_of_scenario_crosswalks()
        if self.interacting_crosswalks:
            self.find_junction()
        else: 
            pass
        print("-----------------------------------------")
        print("All static objects recognized...")
        print("-----------------------------------------")
        
        


    def find_id_of_scenario_crosswalks(self):
   
        # list of crosswalk ID'S present in scenario
        crosswalk_ids = []
        # loop through the set of map features
        for road_part in self.Map:

            # calculate only for crosswalks
            if len(road_part.crosswalk.polygon) != 0 : 
                polygon_1 = []
                for polygon in road_part.crosswalk.polygon:
                    polygon_1.append((polygon.x,polygon.y))
                # loop through the states of the EgoVehicle
                for i,states in enumerate(self.Vehicle.states):
                    # Point of Vehicle center
                    point = Point(states.center_x,states.center_y)
                    # Detect whether the point is inside a Crosswalk Polygon
                    if Polygon(polygon_1).contains(point):
                        crosswalk_ids.append(road_part.id)
        
        # algorithm appends many instances of the same crosswalk
        # so we have to extract only unique ID'S.
        crosswalk_ids = list(set(crosswalk_ids))
        self.interacting_crosswalks = crosswalk_ids
        
        if self.interacting_crosswalks:
            message = "Found {a} crosswalks that Vehicle crosses, with ID's: {b}".format(a=len(self.interacting_crosswalks),b = self.interacting_crosswalks)
            print("-----------------------------------------")
            print(message)
            print("-----------------------------------------")
            
        else:
            print("-----------------------------------------")
            print("Found no crosswalks")
            print("-----------------------------------------")


    def find_center_of_junction(self,crosswalks):
    
        x = []
        y = []
        for id in crosswalks:
            for road_part in self.Map:
                if road_part.id == id:
                    for polygon in road_part.crosswalk.polygon:
                        x.append(polygon.x)
                        y.append(polygon.y)
        x_center = (max(x)+min(x)) /2
        y_center = (max(y)+min(y)) /2

        return Point(x_center,y_center),Polygon([(min(x),min(y)),(max(x),min(y)),(max(x),max(y)),(min(x),max(y))])


    def find_junction(self,):
        from shapely.geometry import Point
        from shapely.geometry.polygon import Polygon
        
        # adding Polygon object of interacting crosswalks to Static object
        for id in self.interacting_crosswalks:
            for road_part in self.Map:
                if road_part.id == id:
                    polygon_of_interacting_crosswalk = []
                    for polygon in road_part.crosswalk.polygon:
                        polygon_of_interacting_crosswalk.append((polygon.x,polygon.y))
                    Polygon_of_interacting_crosswalk = Polygon(polygon_of_interacting_crosswalk)
                    self.interacting_crosswalks_polygons["{a}".format(a=id)] = Polygon_of_interacting_crosswalk
        
        
        #lane ID's that run through crosswalks



        lanes = []
        for id in self.interacting_crosswalks:
            for road_part in self.Map:

                if road_part.id == id: 
                    # remember crosswalk points
                    polygon_1 = []
                    for polygon in road_part.crosswalk.polygon:
                        polygon_1.append((polygon.x,polygon.y))
                    Polygon_1 = Polygon(polygon_1)
                    for road_construct in self.Map:
                        if road_construct.lane.type == 2 or road_construct.lane.type == 1:
                            for polygon in road_construct.lane.polyline:
                                point = Point(polygon.x,polygon.y)
                                if Polygon_1.contains(point):
                                    lanes.append(road_construct.id) 
        # only unique lane ID's that run through crosswalks
        lanes = list(set(lanes))
        # reverse enginering of crosswalks that contain calculated lanes
        set_of_crosswalks = []
        
        for road_part in self.Map:

                if len(road_part.crosswalk.polygon) != 0: 
                    # remember crosswalk points
                    polygon_1 = []
                    for polygon in road_part.crosswalk.polygon:
                        polygon_1.append((polygon.x,polygon.y))
                    Polygon_1 = Polygon(polygon_1)
                    for id in lanes:
                        for road_construct in self.Map:
                            if road_construct.id ==id :
                                for polygon in road_construct.lane.polyline:
                                    point = Point(polygon.x,polygon.y)
                                    if Polygon_1.contains(point):
                                        set_of_crosswalks.append(road_part.id)
        set_of_crosswalks = list(set(set_of_crosswalks))

        if len(set_of_crosswalks) == 0 :
            message = "Found no junction"
            print("-----------------------------------------")
            print(message)
            print("-----------------------------------------")
            self.junction["exists"] = False
        else:
            message = "Found a {a}-way junction that the vehicle crosses over".format(a = len(set_of_crosswalks))
            print("-----------------------------------------")
            print(message)
            print("-----------------------------------------")
            self.junction["exists"] = True
            self.junction["type"] = len(set_of_crosswalks)
            Point,Polygon = self.find_center_of_junction(set_of_crosswalks)
            self.junction["Center_point"] = Point
            self.junction["Polygon"] = Polygon
            self.junction["Junction_crosswalks"] = {}
            for id in set_of_crosswalks:
                self.junction["Junction_crosswalks"][id] = crosswalk_points(self.Map,id)
    

        
                    

    
            



        
        
        
        