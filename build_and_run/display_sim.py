# from base_intersection import *
# from example_intersection import *
from improved_roundabout import *

#defining the intersection
intersection = Intersection()
# intersection = ExIntersection()

#assigning the simulation
sim = intersection.get_sim()
#defining the window that displays the simulation
win = Window(sim)

#starts running the simulation then displays it
win.run()
win.show()