from trafficSimulator import *
import numpy as np

class ExIntersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 24
        island_width = 2
        length = 100

#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.vehicle_rate = 10
        self.v = 17
        self.speed_variance = 0
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#

        # 0
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), 
                                (lane_space/2 + island_width/2, intersection_size/2))
        # 1
        self.sim.create_segment((lane_space/2 + island_width/2, intersection_size/2), 
                                (lane_space/2 + island_width/2, -intersection_size/2))
        # 2
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2),
                                (lane_space/2 + island_width/2, -length - intersection_size/2))
        # 3
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), 
                                                (lane_space/2, lane_space/2 + island_width/2), 
                                                (intersection_size/2, lane_space/2 + island_width/2))
        # 4
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), 
                                                (lane_space/2, -lane_space/2 - island_width/2), 
                                                (-intersection_size/2, -lane_space/2 - island_width/2))
        # 5
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2),
                                (length + intersection_size/2, lane_space/2 + island_width/2))
        # 6
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2),
                                (-length - intersection_size/2, -lane_space/2 - island_width/2))

        self.vg = VehicleGenerator({
            'vehicles': [
                (1, {'path': [0, 1, 2]}),
                (1, {'path': [0, 3, 5]}),
                (1, {'path': [0, 4, 6]}),
            ],
            'vehicle_rate': self.vehicle_rate
        })
        self.sim.add_vehicle_generator(self.vg)

    def get_sim(self):
        return self.sim