import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *
from types import SimpleNamespace
from TimeHelper import TimeHelper # TODO add to files downloaded
Hz = 30

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        assert(isinstance(id, int))
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.timer = self.create_timer(1/Hz, self.timer_callback)
        self.timeHelper = TimeHelper(self)
        self.running = False
        self.done = False
    
    def compute_trajectories(self):
        """
        Inject Trajectory computation code here...
        """
        trajectories = []
        ### -----Insert Trajectories Here-------

        return trajectories

    def upload_trajectories(crazyflies, trajectories):
        '''
            Upload trajectories to crazyflies one by one
        '''

        for i, traj in enumerate(trajectories):
            for cf in crazyflies:
                cf.uploadTrajectory(traj, i, 0)

    def start(self):
        """
            Start execution of blocks
        """
        trajectories = self.compute_trajectories()
        self.upload_trajectories(trajectories)
        self.execute_blocks()

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def execute_blocks(self):
        """
        Must be injected into...

        Typical format should be:
        
        start_time = 0.0
        self.timeHelper.sleepUntil(start_time)
        takeoff(crazyflies, height=1.0, duration=2.0)

        start_time = 5.0
        self.wait_until(start_time)
        land(crazyflies, height=0.0, duration=2.0)
        
        ...

        Where a new start time is added for each block.
        """
        groupState = SimpleNamespace(crazyflies=self.crazyflies, timeHelper=self.timeHelper)
        ### ---------Insert Execution Code Here------------
        # Block Name: takeoffhigh
        start_time = 0.23999999999999977
        self.timeHelper.sleepUntil(start_time)
        takeoff(groupState, 1.5, 3)
        goto_velocity_relative_position(groupState, -0.5,0,0,0.25)
        # Block Name: 0.1
        start_time = 4.4899999999999975
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.1,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.15,0,0,0.0625))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.1,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.15,0,0,0.0625))
        # Block Name: 0.2
        start_time = 8.500000000000009
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.1))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.1))
        # Block Name: 0.3
        start_time = 12.509999999999994
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.3,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.125))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.3,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.125))
        # Block Name: 0.4
        start_time = 16.524999999999995
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.4,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.125))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.4,0.25), lambda groupState: goto_velocity_relative_position(groupState, 0.2,0,0,0.125))
        # Block Name: -0.1
        start_time = 21.135
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.1,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.15,0,0,0.0625))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.1,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.15,0,0,0.0625))
        # Block Name: -0.2
        start_time = 25.94
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.1))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.1))
        
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.1))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.2,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.1))
        # Block Name: -0.3
        start_time = 33.95
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.3,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.125))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.3,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.125))
        # Block Name: -0.4
        start_time = 37.17
        self.timeHelper.sleepUntil(start_time)
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,-0.4,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.125))
        addTrajectories(groupState, lambda groupState: goto_velocity_relative_position(groupState, 0,0,0.4,0.25), lambda groupState: goto_velocity_relative_position(groupState, -0.2,0,0,0.125))
        # Block Name: prelanding
        start_time = 42.03
        self.timeHelper.sleepUntil(start_time)
        goto_velocity_relative_position(groupState, -0.5,0,0,0.25)

        
        self.done = True
    
    def timer_callback(self):
        if not self.running:
            self.start()
            self.running = True
