from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49
        island_width = 2
        length = 43.75
        radius = 18

        self.v = 8.5

        #entrance 0-7
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        self.sim.create_segment((lane_space*3/2 + island_width/2, length + intersection_size/2), (lane_space*3/2 + island_width/2, intersection_size/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, -lane_space*3/2 - island_width/2)) 
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2)) 
        #exit 8-15
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_segment((intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((lane_space*3/2 + island_width/2, -intersection_size/2), (lane_space*3/2 + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length-intersection_size/2, -lane_space*3/2 - island_width/2))
        #corners 16-19
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))
        #connectors 20-23
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))
        #turn into corners 24-31
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2, intersection_size/2),(lane_space*3/2 + island_width/2, radius),(lane_space*3/2 + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2),(radius, -lane_space*3/2 - island_width/2),(radius,-lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, - intersection_size/2),(-lane_space*3/2 - island_width/2, -radius),(-lane_space*3/2 - island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2),(-radius,lane_space*3/2 + island_width/2),(-radius,lane_space*3/2 + island_width/2))
        #turn to exit 32-39
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space*3/2 + island_width/2),(intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space*3/2 - island_width/2),(-intersection_size/2, -lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space*3/2 - island_width/2,radius),(-lane_space*3/2 - island_width/2, intersection_size/2))
    
        self.vg = VehicleGenerator({
            'vehicles': [
                (1, {'path': [0, 24, 16,32,8],'v_max':self.v}),
                (1, {'path': [0, 24, 16,20,17,33,9],'v_max':self.v}),
                (1, {'path': [0, 24, 16,20,17,21,18,34,10],'v_max':self.v}),
                (1, {'path': [0, 24, 16,20,17,21,18,22,19,35,11],'v_max':self.v}),

                (1, {'path': [1, 25, 17,33,9],'v_max':self.v}),
                (1, {'path': [1, 25, 17,21,18,34,10],'v_max':self.v}),
                (1, {'path': [1, 25, 17,21,18,22,19,35,11],'v_max':self.v}),
                (1, {'path': [1, 25, 17,21,18,22,19,15,16,32,8],'v_max':self.v}),

                (1, {'path': [2, 26, 18,34,10],'v_max':self.v}),
                (1, {'path': [2, 26, 18,22,19,35,11],'v_max':self.v}),
                (1, {'path': [2, 26, 18,22,19,15,16,32,8],'v_max':self.v}),
                (1, {'path': [2, 26, 18,22,19,15,16,20,17,33,9],'v_max':self.v}),
                
                (1, {'path': [3, 27, 19,35,11],'v_max':self.v}),
                (1, {'path': [3, 27, 19,15,16,32,8],'v_max':self.v}),
                (1, {'path': [3, 27, 19,15,16,20,17,33,9],'v_max':self.v}),
                (1, {'path': [3, 27, 19,15,16,20,17,21,18,34,10],'v_max':self.v}),
            ], 'vehicle_rate': 30
        })
        
        self.sim.define_interfearing_paths([0,24],[15,16],turn=True)
        self.sim.define_interfearing_paths([1,25],[20,17],turn=True)
        self.sim.define_interfearing_paths([2,26],[21,18],turn=True)
        self.sim.define_interfearing_paths([3,27],[22,19],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim