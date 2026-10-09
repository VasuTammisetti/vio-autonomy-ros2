"""
factory_layout.py -- one place for the layout of the factory hall.

Used by make_factory_world.py (builds the world), sim_agents.py (moves people and the cart),
mission_runner.py (patrol goals) and auto_mapper_v2.py (parks the agents while mapping).
Change a number here, not in four files.

World frame: x points east, y points north, origin = robot start (hall centre). Units: metres, radians.
"""
import math

HALL_X = 20.0          # east-west inner length
HALL_Y = 15.0          # north-south inner length
WALL_H = 2.5

MAIN_AISLE_HALF = 1.8  # main aisle runs east-west through the centre, y in [-1.8, 1.8]
CROSSING_X = 2.5       # zebra crossing across the main aisle (people walk north-south here)
CART_LANE_Y = -3.2     # tugger cart lane, east-west
CART_LANE_HALF = 0.6

# Patrol stations for the robot: name -> (x, y, yaw)
STATIONS = [
    ("assembly", -6.5, 4.4, math.pi / 2),
    ("loading_bay_1", 4.0, -6.0, -math.pi / 2),
    ("charging", 8.8, 4.5, 0.0),
    ("qa_cell", -5.8, -4.9, math.pi),
]

# Moving agents: name -> kind, speed (m/s), waypoints [(x, y, dwell_s)]. Paths loop back to the first point.
# 'person_standing' is the person model that was already in the world; ped_2, ped_3 and cart are added.
AGENTS = [
    ("person_standing", "person", 0.7, [(CROSSING_X, 5.6, 2.0), (CROSSING_X, -5.4, 2.0)]),   # uses the zebra crossing
    ("ped_2", "person", 0.6, [(-2.8, 4.6, 3.0), (-2.8, -5.4, 2.5)]),                       # crosses without a crossing
    ("ped_3", "person", 0.5, [(-8.3, 5.7, 4.0), (-4.8, 5.7, 3.0)]),                        # worker at the assembly benches
    ("cart", "cart", 0.5, [(-7.8, CART_LANE_Y, 3.0), (8.6, CART_LANE_Y, 3.0)]),            # tugger cart in its lane
]

# Where agents go while the room is being mapped (outside the hall, out of LiDAR sight)
PARK = {"person_standing": (30.0, 30.0), "ped_2": (32.0, 30.0), "ped_3": (34.0, 30.0), "cart": (36.0, 30.0)}

PILLARS = [(-4.5, 2.6, "C2"), (5.5, 2.6, "C5"), (0.0, -5.6, "D3")]