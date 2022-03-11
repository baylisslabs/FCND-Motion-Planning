import argparse
import time
import msgpack
import re
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path, local_to_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_lat, goal_lon, target_altitude, safety_distance):
        super().__init__(connection)

        self.target_position = np.array((0.0, 0.0, 0.0))
        self.all_waypoints = []
        self.in_mission = True
        self.goal = (goal_lon, goal_lat, 0)
        self.target_altitude = target_altitude
        self.safety_distance = safety_distance

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.TAKEOFF:
            if self.altitude_reached(-self.target_position[2]):
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
             if self.ground_position_reached(self.target_position) and self.speed_reached(0.0):
                 if self.all_waypoints:
                    self.waypoint_transition()
                 else:
                    self.landing_transition()
        elif self.flight_state == States.LANDING:
            if self.altitude_reached(0.0) and self.speed_reached(0.0):
                self.disarming_transition()


    def velocity_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.WAYPOINT:
            if self.ground_position_reached(self.target_position) and self.speed_reached(0.0):
                if self.all_waypoints:
                    self.waypoint_transition()
                else:
                    self.landing_transition()
        elif self.flight_state == States.LANDING:
            if self.altitude_reached(0) and self.speed_reached(0.0):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            if self.is_global_position_set():
                self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed and self.guided:
                self.plan_path_transition()
        elif self.flight_state == States.PLANNING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def ground_position_reached(self,position,tolerance=0.5):
        return np.linalg.norm(self.local_position[0:2]-position[0:2]) <= tolerance

    def altitude_reached(self,altitude,tolerance=0.5):
        return abs(self.local_position[2]+altitude) <= tolerance

    def speed_reached(self,speed,tolerance=0.2):
        return abs(np.linalg.norm(self.local_velocity)-speed) <= tolerance

    def is_global_position_set(self):
        return self.global_position[0] or self.global_position[1] or self.global_position[2]

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print(f"takeoff transition({-self.target_position[2]})")
        self.takeoff(-self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints[0]
        self.all_waypoints = self.all_waypoints[1:]
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            -self.target_position[2],
            0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.target_position = np.array((0.0,0.0,0.0))
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def send_waypoints(self):
        print(f"Sending waypoints to simulator ...")
        data = msgpack.dumps([(int(p[0]),int(p[1]),int(p[2])) for p in self.all_waypoints])
        self.connection._master.write(data)

    def load_home_position(self,file):
        """
        Retrieve global home position from first line of colliders file (lon, lat, up)
        """
        with open(file) as f:
            line0 = f.readline()
            match = re.search(r"lat0 ([0-9-.]+), lon0 ([0-9-.]+)", line0)
            return np.array((float(match.group(2)),float(match.group(1)),0))

    def plan_path_transition(self):
        print("Searching for a path ...")

        home_position = self.load_home_position('colliders.csv')
        print(f"set_home_position({home_position})")
        self.set_home_position(home_position[0],home_position[1],home_position[2])

        local_position = global_to_local(self.global_position, self.global_home)
        print(f"calculated local position: {local_position}")

        print(f"global home {self.global_home}, position {self.global_position}, local position {self.local_position}")

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, self.target_altitude, self.safety_distance)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        grid_start = local_to_grid(local_position,north_offset,east_offset)

        grid_goal = local_to_grid(
            global_to_local(self.goal, self.global_home),
            north_offset,
            east_offset)

        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print(f"Found path: {path}")
        path, _ = prune_path(grid, path[:1], path[1:])
        print(f"Pruned path: {path}")

        # Convert path to waypoints
        self.all_waypoints = [(p[0] + north_offset, p[1] + east_offset, -self.target_altitude, 0) for p in path]
        self.target_position = np.array((0.0, 0.0, -self.target_altitude))
        self.flight_state = States.PLANNING
        self.send_waypoints()

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    parser.add_argument('--goal-lat', type=float, default=37.792123, help='Goal latitude')
    parser.add_argument('--goal-lon', type=float, default=-122.398844, help='Goal longitude')
    parser.add_argument('--target-altitude', type=int, default=5, help='Target altitude')
    parser.add_argument('--safety-distance', type=int, default=5, help='Safety distance')

    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60, threaded=False, PX4=False)
    drone = MotionPlanning(conn, args.goal_lat, args.goal_lon, args.target_altitude, args.safety_distance)
    time.sleep(2)
    drone.start()
