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
        # Block Name: firstDroneTakeoff
        start_time = 0.07999999999999965
        self.timeHelper.sleepUntil(start_time)
        # takeoff(groupState, 1.7, 3)
        # stop_and_hover(groupState)
        # # Block Name: firstDroneLand
        # start_time = 19.002929687499986
        # self.timeHelper.sleepUntil(start_time)
        # land(groupState, 0,3)
        takeoff(groupState, 0.5, 3)
        goto_duration(groupState, 0,0.5,0.5,3)
        rotate(groupState, lambda groupState: circle(groupState, 1, 0.5, 6.283185307179586,'null'), 0, 1.5707963267948966, 0)
        land(groupState, 0,3)

        
        self.done = True
    
    def timer_callback(self):
        if not self.running:
            self.start()
            self.running = True