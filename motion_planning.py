import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils_graph import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import matplotlib.pyplot as plt
from sampling import Sampler
from graph_building import create_graph


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        data = np.loadtxt(filename, delimiter=';,', dtype='str')[0].split(", ")
        lat0, lon0 = [float(d.split(" ")[1]) for d in data]

        print("lat0, lon0", lat0, lon0)

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        print("self.local_position ", self.local_position)

        # TODO: retrieve current global position
        current_glbl_pos = [self._longitude, self._latitude,self._altitude]
        print(current_glbl_pos)
        print(self.global_position)

        # TODO: convert to current local position using global_to_local()
        current_lcl_pos = global_to_local(current_glbl_pos, self.global_home)
        print("current_lcl_pos ", len(current_lcl_pos))
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(current_lcl_pos[0]), int(current_lcl_pos[1]))
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)

        offsets = (north_offset, east_offset)

        # Example: sampling 1000 points and removing
        # ones conflicting with obstacles.
        # print("Start of sampling...")
        sampler = Sampler(data)
        polygons = sampler.polygons
        nodes = sampler.sample(300)
        nodes.insert(0, (grid_start[0], grid_start[1], 5.0))
        print("nodes", nodes)
        # print("len_nodes", len(nodes))
        # print("End of sampling")

        # Build a graph using the sampled nodes
        # and connect each nodes to its 10th closest nodes
        print("Start graph building...")
        t0 = time.time()
        graph = create_graph(polygons, nodes, 11)
        print('graph took {0} seconds to build'.format(time.time() - t0))

        start = list(graph.nodes)[0]

        len_path = 0
        while len_path == 0:
            k = np.random.randint(len(graph.nodes))
            print(k, len(graph.nodes))
            goal = list(graph.nodes)[k]

            # goal = (709.6953556162655, 480.1928341700754, 18.724961802433807)

            print("start,goal", start, goal)

            # print("grid_start, grid_goal", grid_start, grid_goal)
            
            # plt.imshow(grid, cmap='Greys', origin='lower')
            # plt.plot(grid_start[1], grid_start[0], 'x')
            # plt.plot(grid_goal[1], grid_goal[0], 'x')
            # plt.show()

            path, _ = a_star(graph, heuristic, start, goal)
            len_path = len(path)
        pruned_path = prune_path(path, polygons)

        # plt.imshow(grid, cmap='Greys', origin='lower')

        # # draw edges
        # for (n1, n2) in graph.edges:
        #     plt.plot([n1[1] - east_offset, n2[1] - east_offset],
        #              [n1[0] - north_offset, n2[0] - north_offset], 'blue', alpha=0.5)

        # # draw connected nodes
        # for n1 in graph.nodes:
        #     plt.scatter(n1[1] - east_offset, n1[0] - north_offset, c='red')
        # # For the purposes of the visual the east coordinate lay along
        # # the x-axis and the north coordinates long the y-axis.
        # plt.plot(start[1] - east_offset, start[0] - north_offset, 'x')
        # plt.plot(goal[1] - east_offset, goal[0] - north_offset, 'x')
        # # plt.show()

        # # print("start ", start[1], start[0])
        # # print("start ", goal[1], goal[0])

        # # print(len(path))
        # # print("pp[:, 1] ", pp1[:, 1][0], pp1[:, 0][0])
        # # print("pp[:, 1] ", pp1[:, 1][-1], pp1[:, 0][-1])
        # pp1 = np.array(path)
        # plt.plot(pp1[:, 1] - east_offset, pp1[:, 0] - north_offset, 'g')

        # # plt.xlabel('EAST')
        # # plt.ylabel('NORTH')
        # # print(path, path)
        # # print("pruned_path", pruned_path)
        # # print(len(path))
        # # print(len(pruned_path))
        # pp2 = np.array(pruned_path)
        # plt.plot(pp2[:, 1] - east_offset, pp2[:, 0] - north_offset, 'y')
        # plt.show()
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        path = pruned_path

        # path = [[0, 0, 5.0, 0], [66.9370981974588, 30.19269583034736, 14.234298552833767, 0], [70.34303013581928, 63.54105965355211, 7.893215810248362, 0], [102.95893012555308, 94.58053333408247, 10.888389506580907, 0], [123.96481111175109, 133.5637465482348, 2.059356357794573, 0], [156.70567566891026, 166.8098450360302, 19.816858952638338, 0], [189.11392716337156, 218.42843894658802, 11.349741536617795, 0], [253.9550130598646, 249.01700706657562, 13.28287430801504, 0],
                # [301.27643026492586, 278.56438300889533, 17.865244568663073, 0], [332.2266519006742, 300.4081532588564, 17.328211894650973, 0], [369.9179765205873, 321.41099762523913, 11.549857703012549, 0], [414.03506621165496, 348.3528546751338, 0.3863419218183406, 0], [470.9470870017146, 358.9010704471268, 6.461690931861986, 0], [523.7197772627942, 351.37048057430854, 18.441224421777157, 0], [591.5067409141013, 387.1421064414079, 8.001992953583708, 0]]

        print(path)
        # path.pop(0)

        for p in path:
            print([int(p[0]), int(p[1]), int(p[2]), 0])

        # Convert path to waypoints
        # waypoints = [[0, 0, 5, 0], [8, 32, 10, 0], [104, 82, 11, 0], [65, 136, 7, 0], [3, 169, 17, 0],
                    #  [-66, 245, 18, 0], [-142, 234, 13, 0], [-185, 187, 7, 0], [-224, 256, 6, 0]]
        # waypoints = [[0, 0, 5, 0], [-232, 245, 12, 0], [-244, 351, 15, 0]]
        # print("waypoints 1", waypoints)
        waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path]
        # waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, TARGET_ALTITUDE, 0] for p in path]
        print("waypoints 2", waypoints)
        # waypoints = [[316 + north_offset, 445 + east_offset, TARGET_ALTITUDE, 0], [317 + north_offset, 446 + east_offset, TARGET_ALTITUDE, 0], [318 + north_offset, 447 + east_offset, TARGET_ALTITUDE, 0], [319 + north_offset, 448 + east_offset, TARGET_ALTITUDE, 0], [320 + north_offset, 449 + east_offset, TARGET_ALTITUDE, 0],
        #  [321 + north_offset, 450 + east_offset, TARGET_ALTITUDE, 0], [322 + north_offset, 451 + east_offset, TARGET_ALTITUDE, 0], [323 + north_offset, 452 + east_offset, TARGET_ALTITUDE, 0], [324 + north_offset, 453 + east_offset, TARGET_ALTITUDE, 0], [325 + north_offset, 454 + east_offset, TARGET_ALTITUDE, 0]]
        # print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
