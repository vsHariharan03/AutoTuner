import rclpy
from rclpy.node import Node
import time

import json
import random

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Empty
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import ClearEntireCostmap
from robot_interface.srv import Score
from geometry_msgs.msg import Twist

class Nav2_switch(Node):

    def __init__(self):
        super().__init__('nav2_switch')
 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters'
        )

        while not self.param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for controller_server...")


        # service to trigger parameter update
        self.srv = self.create_service(
            Score,
            'update_mppi_params',
            self.update_params_callback
        )

        self.sim_client = self.create_client(Empty, '/reset_world')

        while not self.sim_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for gazebo server...")

        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes'
        )

        while not self.lifecycle_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for lifecycle service...")


        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        while not self.clear_global_costmap.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for global costmap clear service...")

        while not self.clear_local_costmap.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for local costmap clear service...")

        self.score_client = self.create_client(
            Score,
            'send_score'
        )


        while not self.score_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for score accumulator service...")


        self.get_logger().info("MPPI parameter service ready")
        self.get_logger().info("Gazebo service ready")


        l = [
            ("FollowPath.time_steps", 28),
            ("FollowPath.model_dt", 0.05),
            ("FollowPath.batch_size", 2000),
            ("FollowPath.vx_std", 0.45),
            ("FollowPath.wz_std", 0.4),
            ("FollowPath.vx_max", 2.5),
            ("FollowPath.vx_min", -0.25),
            ("FollowPath.wz_max", 3.0),
            ("FollowPath.temperature", 0.25),
            ("FollowPath.gamma", 0.015),
            ("FollowPath.prune_distance", 1.7),
            
            # ConstraintCritic
            ("FollowPath.ConstraintCritic.cost_weight", 4.0),

            # GoalCritic
            ("FollowPath.GoalCritic.cost_weight", 5.0),
            ("FollowPath.GoalCritic.threshold_to_consider", 1.4),

            # GoalAngleCritic
            ("FollowPath.GoalAngleCritic.cost_weight", 3.0),
            ("FollowPath.GoalAngleCritic.threshold_to_consider", 0.5),

            # PreferForwardCritic
            ("FollowPath.PreferForwardCritic.cost_weight", 5.0),
            ("FollowPath.PreferForwardCritic.threshold_to_consider", 0.5),

            # ObstaclesCritic
            ("FollowPath.ObstaclesCritic.repulsion_weight", 4.0),
            ("FollowPath.ObstaclesCritic.critical_weight", 20.0),
            ("FollowPath.ObstaclesCritic.collision_margin_distance", 0.15),
            ("FollowPath.ObstaclesCritic.near_goal_distance", 0.1),

            # CostCritic
            ("FollowPath.CostCritic.cost_weight", 3.81),
            ("FollowPath.CostCritic.critical_cost", 300.0),
            ("FollowPath.CostCritic.near_goal_distance", 1.0),

            # PathAlignCritic
            ("FollowPath.PathAlignCritic.cost_weight", 20.0),
            ("FollowPath.PathAlignCritic.max_path_occupancy_ratio", 0.05),
            ("FollowPath.PathAlignCritic.threshold_to_consider", 0.5),
            ("FollowPath.PathAlignCritic.offset_from_furthest", 20)
        ]
        self.curr_params=dict(l)
        self.score=0

        self.is_processing = False

  
        
    def stop_robot(self):
            
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info("Sent zero velocity command (STOP)")
            print("published stop")

    def make_double(self, name, value):
        p = Parameter()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=value
        )
        return p


    def make_int(self, name, value):
        p = Parameter()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER,
            integer_value=value
        )
        return p
    def param_response(self, future):

        result = future.result()

        if result is not None:
            self.get_logger().info("MPPI parameters updated")
        else:
            self.get_logger().error("Parameter update failed")

        for r in result.results:
            print(r.successful, r.reason)

        self.send_request()
        print('-------------------')

    def update_params_callback(self, request, response):
        if self.is_processing:
            print("Reset underway .. cancelling")
            response.success = False
            return response

        self.is_processing = True

        print("Pausing all nav2 nodes")

        

        self.score=request.score
        self.pause_future = self.send_lifecycle_request(1)
        self.pause_future.add_done_callback(self.send_score_request)

        response.success = True
        response.message = "Updating MPPI parameters"

        return response
    

    def send_score_request(self,future):

        self.stop_robot()

        print("Nav2 paused")

        req = Score.Request()
        req.score=self.score
        req.params=json.dumps(self.curr_params) 

        future = self.score_client.call_async(req)
        future.add_done_callback(self.after_pause)
        self.get_logger().info("Sent the score")

        return req
        
    def after_pause(self, future):

        print("Sending parameters") 

        result = future.result()
        print(result)
        self.curr_params=json.loads(result.message)
        future = self.param_client.call_async(self.create_param_request())
        future.add_done_callback(self.after_param_update)     


    def create_param_request(self):

        req = SetParameters.Request()

        req.parameters = [
            self.make_int("FollowPath.time_steps", int(self.curr_params["FollowPath.time_steps"])),
            self.make_double("FollowPath.model_dt", self.curr_params["FollowPath.model_dt"]),
            self.make_int("FollowPath.batch_size", int(self.curr_params["FollowPath.batch_size"])),
            self.make_double("FollowPath.vx_std", self.curr_params["FollowPath.vx_std"]),
            self.make_double("FollowPath.wz_std", self.curr_params["FollowPath.wz_std"]),
            self.make_double("FollowPath.vx_max", self.curr_params["FollowPath.vx_max"]),
            self.make_double("FollowPath.vx_min", self.curr_params["FollowPath.vx_min"]),
            self.make_double("FollowPath.wz_max", self.curr_params["FollowPath.wz_max"]),
            self.make_double("FollowPath.temperature", self.curr_params["FollowPath.temperature"]),
            self.make_double("FollowPath.gamma", self.curr_params["FollowPath.gamma"]),
            self.make_double("FollowPath.prune_distance", self.curr_params["FollowPath.prune_distance"]),

            # ConstraintCritic
            self.make_double("FollowPath.ConstraintCritic.cost_weight", self.curr_params["FollowPath.ConstraintCritic.cost_weight"]),

            # GoalCritic
            self.make_double("FollowPath.GoalCritic.cost_weight", self.curr_params["FollowPath.GoalCritic.cost_weight"]),
            self.make_double("FollowPath.GoalCritic.threshold_to_consider", self.curr_params["FollowPath.GoalCritic.threshold_to_consider"]),

            # GoalAngleCritic
            self.make_double("FollowPath.GoalAngleCritic.cost_weight", self.curr_params["FollowPath.GoalAngleCritic.cost_weight"]),
            self.make_double("FollowPath.GoalAngleCritic.threshold_to_consider", self.curr_params["FollowPath.GoalAngleCritic.threshold_to_consider"]),

            # PreferForwardCritic
            self.make_double("FollowPath.PreferForwardCritic.cost_weight", self.curr_params["FollowPath.PreferForwardCritic.cost_weight"]),
            self.make_double("FollowPath.PreferForwardCritic.threshold_to_consider", self.curr_params["FollowPath.PreferForwardCritic.threshold_to_consider"]),

            # ObstaclesCritic
            self.make_double("FollowPath.ObstaclesCritic.repulsion_weight", self.curr_params["FollowPath.ObstaclesCritic.repulsion_weight"]),
            self.make_double("FollowPath.ObstaclesCritic.critical_weight", self.curr_params["FollowPath.ObstaclesCritic.critical_weight"]),
            self.make_double("FollowPath.ObstaclesCritic.collision_margin_distance", self.curr_params["FollowPath.ObstaclesCritic.collision_margin_distance"]),
            self.make_double("FollowPath.ObstaclesCritic.near_goal_distance", self.curr_params["FollowPath.ObstaclesCritic.near_goal_distance"]),

            # CostCritic
            self.make_double("FollowPath.CostCritic.cost_weight", self.curr_params["FollowPath.CostCritic.cost_weight"]),
            self.make_double("FollowPath.CostCritic.critical_cost", self.curr_params["FollowPath.CostCritic.critical_cost"]),
            self.make_double("FollowPath.CostCritic.near_goal_distance", self.curr_params["FollowPath.CostCritic.near_goal_distance"]),

            # PathAlignCritic
            self.make_double("FollowPath.PathAlignCritic.cost_weight", self.curr_params["FollowPath.PathAlignCritic.cost_weight"]),
            self.make_double("FollowPath.PathAlignCritic.max_path_occupancy_ratio", self.curr_params["FollowPath.PathAlignCritic.max_path_occupancy_ratio"]),
            self.make_double("FollowPath.PathAlignCritic.threshold_to_consider", self.curr_params["FollowPath.PathAlignCritic.threshold_to_consider"]),
            self.make_int("FollowPath.PathAlignCritic.offset_from_furthest", int(self.curr_params["FollowPath.PathAlignCritic.offset_from_furthest"]))
        ]

        return req
    


    def after_param_update(self, future):

        result = future.result()
        self.get_logger().info("MPPI parameters updated")

        for r in result.results:
            print(r.successful, r.reason)

        req = Empty.Request()
        future = self.sim_client.call_async(req)
        future.add_done_callback(self.after_reset)
        
    def after_reset(self, future):

        self.get_logger().info("Simulation reset complete")
        self.clear_costmaps()
        print("Turning on all nav2 nodes")
        self.send_lifecycle_request(2)
        self.is_processing=False

    def send_request(self):

        req = Empty.Request()
        future = self.sim_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Simulation reset complete")
        self.stop_robot()

    def send_lifecycle_request(self,cmd):

        def lifecycle_done():
            cmd_dict = {
                0: "STARTUP",
                1: "PAUSE",
                2: "RESUME",
                3: "RESET",
                4: "SHUTDOWN",
                5: "CONFIGURE",
                6: "CLEANUP"
            }
            
            self.get_logger().info(f"Completed command { cmd_dict[cmd] }")

        req = ManageLifecycleNodes.Request()
        req.command = cmd

        future = self.lifecycle_client.call_async(req)
        lifecycle_done()

        return future


    def clear_costmaps(self):

        req = ClearEntireCostmap.Request()

        future1 = self.clear_global_costmap.call_async(req)
        future2 = self.clear_local_costmap.call_async(req)

        self.is_processing=False

        self.get_logger().info("Costmaps cleared")
        
def main():
    rclpy.init()
    node=Nav2_switch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()