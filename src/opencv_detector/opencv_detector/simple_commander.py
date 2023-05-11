#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32

"""
Basic navigation demo to go to pose.
"""


class NavigationClient(Node):
    # rclpy.init()

    def __init__(self):
        super().__init__('navigation_client')
        #self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigator = BasicNavigator()

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0
        # initial_pose.pose.position.y = 0
        # initial_pose.pose.orientation.z = 0
        # initial_pose.pose.orientation.w = 0
        # self.navigator.setInitialPose(initial_pose)

        # self.navigator.lifecycleStartup()
        self.navigator.waitUntilNav2Active()

        self._goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal',
            self.send_goal,
            10
        )
        self.feedback_publisher = self.create_publisher(
            Int32,
            '/navigation_feedback',
            10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # self.goal_reached = True
        # self.count = 0
        self.goal_recieved = False
        self.goal_in_progress = False
        
       
    

        # # # Set our demo's initial pose
       

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()


    def send_goal(self, pose):
        self.goal_recieved = True
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose = pose.pose
      
        # goal_pose.pose.position.x = -2.0
        # goal_pose.pose.position.y = -0.5
        # goal_pose.pose.orientation.w = 1.0

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)
        self.goal_in_progress = True
        self.navigator.goToPose(goal_pose)

    def timer_callback(self):
        i = 0
        if not self.navigator.isTaskComplete():
     
            feedback = self.navigator.getFeedback()
            
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #     self.goal_pose.pose.position.x = -3.0
                #     self.navigator.goToPose(self.goal_pose)
        elif self.goal_recieved:     

            # Do something depending on the return code
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                i = Int32()
                i.data = 1
                self.feedback_publisher.publish(i)
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            self.goal_recieved = False

        #navigator.lifecycleShutdown()




def main(args=None):
    rclpy.init(args=args)
    navigation = NavigationClient()
    while rclpy.ok():
        rclpy.spin_once(navigation)
    #print("here")
    # while not navigation._feedback_received and rospy_ok():
    #     pass

    navigation.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
