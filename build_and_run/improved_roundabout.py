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

        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length - intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length - intersection_size/2, -lane_space*3/2 - island_width/2))

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
    
        #inside corners 40-43
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius - lane_space),(radius - lane_space,radius - lane_space),(radius - lane_space,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius - lane_space,-lane_space - island_width/2),(radius - lane_space,-radius + lane_space),(lane_space + island_width/2,-radius + lane_space))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius + lane_space),(-radius + lane_space,-radius + lane_space),(-radius + lane_space,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius + lane_space,lane_space + island_width/2),(-radius + lane_space,radius - lane_space),(-lane_space - island_width/2, radius - lane_space))

        #inside connectors 44-47
        self.sim.create_segment((radius - lane_space,lane_space + island_width/2),(radius - lane_space,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius + lane_space),(-lane_space - island_width/2,-radius + lane_space))
        self.sim.create_segment((-radius + lane_space,-lane_space - island_width/2),(-radius + lane_space,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius - lane_space),(lane_space + island_width/2, radius - lane_space))

        #turn into inside corners 48-55
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius - lane_space),(lane_space + island_width/2, radius - lane_space))
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2, intersection_size/2),(lane_space*3/2 + island_width/2, radius - lane_space),(lane_space*3/2 + island_width/2, radius - lane_space))

        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius - lane_space, -lane_space/2 - island_width/2),(radius - lane_space,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2),(radius - lane_space, -lane_space*3/2 - island_width/2),(radius - lane_space,-lane_space*3/2 - island_width/2))

        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius + lane_space),(-lane_space - island_width/2,-radius + lane_space))
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, - intersection_size/2),(-lane_space*3/2 - island_width/2, -radius + lane_space),(-lane_space*3/2 - island_width/2,-radius + lane_space))

        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius + lane_space,lane_space/2 + island_width/2),(-radius + lane_space,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2),(-radius + lane_space,lane_space*3/2 + island_width/2),(-radius + lane_space,lane_space*3/2 + island_width/2))

        #turn to exit from inside 56-63
        self.sim.create_quadratic_bezier_curve((radius - lane_space,lane_space + island_width/2),(radius - lane_space,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius - lane_space,lane_space + island_width/2),(radius - lane_space,lane_space*3/2 + island_width/2),(intersection_size/2, lane_space*3/2 + island_width/2))

        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius + lane_space),(lane_space/2 + island_width/2,-radius + lane_space),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius + lane_space),(lane_space*3/2 + island_width/2,-radius + lane_space),(lane_space*3/2 + island_width/2, -intersection_size/2))

        self.sim.create_quadratic_bezier_curve((-radius + lane_space,-lane_space - island_width/2),(-radius + lane_space,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius + lane_space,-lane_space - island_width/2),(-radius + lane_space,-lane_space*3/2 - island_width/2),(-intersection_size/2, -lane_space*3/2 - island_width/2))

        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius - lane_space),(-lane_space/2 - island_width/2,radius - lane_space),(-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius - lane_space),(-lane_space*3/2 - island_width/2,radius - lane_space),(-lane_space*3/2 - island_width/2, intersection_size/2))

        # TODO: FIX THE PATHS OF ALL THE VHEILCES AND ADD NEW ONES FOR THE NEW INNER ROUNDABOUT + ADD COLORS
        self.vg = VehicleGenerator({
            'vehicles': [
                # these dont propperly work TODO someone fix
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

                # Testing Paths
                # delete for final this just to test the inner roundabout
                # (1, {'path': [44, 45, 46, 47],'v_max':self.v}),
                # (1, {'path': [48, 49, 50, 51, 52, 53, 54, 55],'v_max':self.v}), # test all turn into corners
                # (1, {'path': [56, 57, 58, 59, 60, 61, 62, 63], 'v_max':self.v}), # test all turn to exit from inside
            ], 'vehicle_rate': 20
        })
        
        # TODO: WE NEED TO FIX THIS AND DETERMINE THE INTERFERING PATHS (lowkey we dont need to do this)
        # self.sim.define_interfearing_paths([0,24],[15,16],turn=True)
        # self.sim.define_interfearing_paths([1,25],[20,17],turn=True)
        # self.sim.define_interfearing_paths([2,26],[21,18],turn=True)
        # self.sim.define_interfearing_paths([3,27],[22,19],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim