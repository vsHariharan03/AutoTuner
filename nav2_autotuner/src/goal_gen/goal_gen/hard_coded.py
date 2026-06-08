import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid , Odometry
import numpy as np

import random
from scipy.signal import savgol_filter
import time

from robot_interface.srv import Score

class goalPublisher(Node):

    def __init__(self):

        super().__init__('get_goal')

        self.reset_client = self.create_client(
            Score,
            'update_mppi_params'
        )

        self.publisher=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.create_subscription(Odometry,'/odom',self.update_odom,5)

        self.timer = self.create_timer(0.3,self.publish_goal)

        self.odom_data=None

        self.xy_goals1=np.load("goals.npy")
        self.xy_goals=np.array(self.xy_goals1)

        def find_outliers(data, window=11, poly=3, threshold=3):
            smooth = savgol_filter(data, window_length=window, polyorder=poly)
            residual = np.abs(data - smooth)
            std = np.std(residual)
            return residual > threshold * std

        outlier_mask = np.zeros(len(self.xy_goals), dtype=bool)

        for col in range(2, 5):
            outlier_mask |= find_outliers(self.xy_goals[:, col])

        self.xy_goals = self.xy_goals[~outlier_mask]

        self.start=0
        self.end=0

        self.num_resets=-1
        self.curr_id=0

        self.is_resetting=0

        self.full_course_time_thresh=50
        self.dist=0

        self.distance_thresh=100.0
        
        dist_arr=[((self.xy_goals[i+1][2]-self.xy_goals[i][2])**2+(self.xy_goals[i+1][3]-self.xy_goals[i][3])**2)**0.5 for i in range(len(self.xy_goals)-1)]
        dist_arr=dist_arr+[dist_arr[-1]]
        self.cumm_dist_arr=[sum(dist_arr[0:i]) for i in range(len(self.xy_goals))]

        for i in enumerate(self.cumm_dist_arr):
            pass
            print(i[0],i[1])


 


    def after_pause(self,future):
        self.curr_id=0
        self.odom_data=None
        self.start=0
        self.is_resetting=0
        time.sleep(3)
        self.get_logger().info("Simulation reset complete")
    
    def send_request(self):

        if(self.is_resetting==1):
            print("reseting the simulation ...")
            return
        

        
        x,y=self.odom_data.position.x,self.odom_data.position.y
        dist=((x-self.xy_goals[self.curr_id][0])**2+(y-self.xy_goals[self.curr_id][1])**2)**0.5
        


        req = Score.Request()
        req.score=(self.cumm_dist_arr[self.curr_id]+dist)/float(((time.time()-self.start)+0.01))

        self.is_resetting=1
        self.curr_id=0
        self.odom_data=None
        self.start=0


        if(self.num_resets==-1):
            req.score=-1.0
        self.num_resets+=1

        future = self.reset_client.call_async(req)
        future.add_done_callback(self.after_pause)


        
        
        

    def update_odom(self , msg : Odometry):

        self.odom_data=msg.pose.pose

    def publish_goal(self):

        if(self.is_resetting==1):
            print('is resetting ...')
            return

        if(self.odom_data==None):
            print("waiting for odom data")
            return 
        
        if((self.start > 0) and (float(((time.time()-self.start)))>self.full_course_time_thresh)):
            self.send_request()
            print(float(((time.time()-self.start))))
            print("timeout reset the simulation")
            return
        

        if(self.cumm_dist_arr[self.curr_id]>=self.distance_thresh):
            self.send_request()
            print(self.curr_id)
            print("reset the simulation")
            return

        
        if(self.num_resets==-1):
            print('1st request ...')
            self.send_request()
            return

        if(self.odom_data==None):
            print("odom data reset waiting ...")
            return
        
        def move(x,y,xy_goals,curr):

            dist=(((x-xy_goals[curr][0])**2+(y-xy_goals[curr][1])**2))**0.5
            dist1=(((x-xy_goals[min(curr+random.randint(5,14),len(xy_goals)-1)][0])**2+(y-xy_goals[min(curr+random.randint(5,14),len(xy_goals)-1)][1])**2))**0.5

            while(dist1<dist):
                if((curr+1)==len(xy_goals)):
                    return curr
                dist=(((x-xy_goals[curr][0])**2+(y-xy_goals[curr][1])**2))**0.5
                dist1=(((x-xy_goals[min(curr+random.randint(5,14),len(xy_goals)-1)][0])**2+(y-xy_goals[min(curr+random.randint(5,14),len(xy_goals)-1)][1])**2))**0.5

                curr+=1
            
            return curr

        self.curr_id=move(self.odom_data.position.x,self.odom_data.position.y,self.xy_goals,self.curr_id)

        goal=PoseStamped()
        goal.header.frame_id='map'
        goal.header.stamp=self.get_clock().now().to_msg()

        goal.pose.position.x=self.xy_goals[self.curr_id][2]
        goal.pose.position.y=self.xy_goals[self.curr_id][3]
        goal.pose.orientation.z = float(self.xy_goals[self.curr_id][4])
        goal.pose.orientation.w = float(self.xy_goals[self.curr_id][5])

        self.publisher.publish(goal)

        if(self.curr_id>1 and self.start==0):
            self.start=time.time()

        print(self.curr_id)


def main():
    rclpy.init()
    node=goalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


