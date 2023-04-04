

# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped
# # from nav2_msgs.action import NavigateToPose
# # # from nav2_msgs.action import NavigateToPoseGoal
# # # from nav2_msgs.action import NavigateToPoseFeedback
# # # from nav2_msgs.action import NavigateToPoseResult
# # from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
# # from nav2_msgs.action._navigate_to_pose import NavigateToPose_Result
# # from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
# # import action_msgs.msg
# # from rclpy.action import ActionClient



# class NavigationActionClient(Node):
#     def __init__(self):
#         super().__init__('navigation_action_client')
#         self._action_client = ActionClient(
#             self,
#             NavigateToPose,
#             'navigate_to_pose'
#         )
#         self._goal_subscriber = self.create_subscription(
#             PoseStamped,
#             '/goal',
#             self._on_new_goal_received,
#             10
#         )
#         self._feedback_received = False

#     def _on_new_goal_received(self, pose):
#         self.get_logger().info('New navigation goal received')
#         #goal = NavigateToPoseGoal()
#         goal = NavigateToPose.Goal()
#         goal.pose = pose
#         self._action_client.wait_for_server()
#         self._send_goal(goal)

#     def _send_goal(self, pose):
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = pose
#         self._feedback_received = False
#         self._action_client.wait_for_server()
#         self.get_logger().info('Sending navigation goal')
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)
        


#     def feedback_callback(self, feedback_future):
#         feedback_msg = feedback_future.result()
#         if feedback_msg is None:
#             return
#         feedback = NavigateToPose_Feedback.from_msg(feedback_msg.feedback)
#         self.get_logger().info(
#             f'Navigation feedback received: {feedback.distance_remaining}'
#         )
#         if feedback.distance_remaining <= 0.1:
#             self.get_logger().info('Robot reached the goal')
#             self._feedback_received = True


#     # def _on_goal_sent(self, future):
#         #     goal_handle = future.result()
#         #     if not goal_handle.accepted:
#         #         self.get_logger().info('Navigation goal rejected')
#         #         return
#         #     self.get_logger().info('Navigation goal accepted')
#         #     feedback_future = goal_handle.feedback_async()
#         #     feedback_future.add_done_callback(self._on_feedback_received)
#         #     result_future = goal_handle.result_async()
#         #     result_future.add_done_callback(self._on_goal_completed)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Navigation goal rejected')
#             return
#         self.get_logger().info('Navigation goal accepted')


#     def _on_goal_completed(self, future):
#         result_msg = future.result()
#         if result_msg is None:
#             self.get_logger().info('Navigation goal failed')
#             return
#         result = NavigateToPose_Result.from_msg(result_msg.result)
#         if result.result == 1:
#             self.get_logger().info('Navigation goal succeeded')
#         else:
#             self.get_logger().info('Navigation goal failed')
#         self._feedback_received = True


   


# def main(args=None):
#     rclpy.init(args=args)
#     navigation_action_client = NavigationActionClient()
#     rclpy.spin(navigation_action_client)
#     while not navigation_action_client._feedback_received:
#         pass
#     navigation_action_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





















# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped


# class NavigationActionClient(Node):

#     def __init__(self):
#         super().__init__('navigation_action_client')
#         self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self._goal_subscriber = self.create_subscription(
#             PoseStamped,
#             '/goal',
#             self._on_new_goal_received,
#             10
#         )
#         self._feedback_received = False
#         self._processing_goal = False



#     # def _on_new_goal_received(self, pose):
#     #     self.get_logger().info('New navigation goal received')
#     #     #goal = NavigateToPoseGoal()
        
        
#     def _on_new_goal_received(self, pose):
#         if self._processing_goal:
#             self.get_logger().info('Already processing a goal, ignoring new goal')
#             return
#         self.get_logger().info('New navigation goal received')
#         goal = NavigateToPose.Goal()
#         goal.pose = pose
#         self._action_client.wait_for_server()
#         self._send_goal(goal)

#     def _send_goal(self, pose):
#         if self._processing_goal:
#             self.get_logger().info('Already processing a goal, ignoring new goal')
#             return
#         self._processing_goal = True
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = pose
#         self._feedback_received = False
#         self._action_client.wait_for_server()
#         self.get_logger().info('Sending navigation goal')
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         #self.get_logger().info(f'Navigation feedback received: {feedback.distance_remaining}')
#         if feedback.distance_remaining <= 0.25:
#             #self.get_logger().info('Robot reached the goal')
#             self._feedback_received = True
        

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Navigation goal rejected')
#             return
#         self.get_logger().info('Navigation goal accepted')

#     def check_goal_status(self):
#         return self._feedback_received


# def main(args=None):
#     rclpy.init(args=args)
#     navigation_action_client = NavigationActionClient()
#     while True:
#         goal = PoseStamped()
#         # Fill the goal message
#         goal.pose.position.x = 4.0
#         goal.pose.position.y = 1.0
#         goal.pose.orientation.z = 0.1
#         navigation_action_client._send_goal(goal)
#         while rclpy.ok() and not navigation_action_client._feedback_received:
#             rclpy.spin_once(navigation_action_client)
#         if navigation_action_client.check_goal_status():
#             navigation_action_client.get_logger().info('Navigation goal succeeded')
#         else:
#             navigation_action_client.get_logger().info('Navigation goal failed')
#         # sleep for some time before sending the next goal
#         rclpy.spin_once(navigation_action_client, timeout_sec=0.5)
#     navigation_action_client.destroy_node()
#     rclpy.shutdown()


# # def main(args=None):
# #     rclpy.init(args=args)
# #     navigation_action_client = NavigationActionClient()
# #     rclpy.spin(navigation_action_client)
# #     print("here")
# #     while not navigation_action_client._feedback_received:
# #         pass
# #     navigation_action_client.destroy_node()
# #     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
#from nav2_msgs.action._navigate_to_pose import NavigateToPose_Result
#from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal

# from nav2_msgs.action._navigate_to_pose import N


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal',
            self._on_new_goal_received,
            10
        )
        self.goal_reached = True
        self.count = 0
        self.goal_in_progress = False


    def _on_new_goal_received(self, pose):
        if self.goal_in_progress:
            self.get_logger().info('Already processing a goal, ignoring new goal')
            return
        elif not self.goal_reached:
            self.get_logger().info('goal not reached')
            return
        self.get_logger().info('New navigation goal received')
        self._action_client.wait_for_server()
        self.send_goal(pose)

    def send_goal(self, pose):
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.goal_reached = False
        self._action_client.wait_for_server()
        #self.get_logger().info('Sending navigation goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.path_check_callback)


    def path_check_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return
        self.get_logger().info('Navigation goal accepted')
        self.goal_in_progress = True



    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(f'Navigation feedback received: {feedback.distance_remaining}')
        # print(type(feedback.distance_remaining)) #float


        if feedback.distance_remaining <= 0.25:
            self.count += 1
            if self.count >= 400:
                self.get_logger().info('Robot reached the goal')
                self.goal_reached = True
                self.goal_in_progress = False
    
    # def _on_goal_completed(self, future):
    #     goal_handle = future.result()
    #     result_msg = goal_handle.get_result()
    #     if result_msg is None:
    #         self.get_logger().info('Navigation goal failed')
    #         return
    #     result = NavigateToPose.Result()
    #     result.result = result_msg.result
    #     if result.result == 1:
    #         self.get_logger().info('Navigation goal succeeded')
    #     else:
    #         self.get_logger().info('Navigation goal failed')
    #     self._feedback_received = True


    # def goal_reached(self):
    #     return self.goal_reached

def main(args=None):
    rclpy.init(args=args)
    navigation_action_client = NavigationActionClient()
    while True:

        # if not navigation_action_client.goal_in_progress and navigation_action_client.goal_reached():
        #     print("send goalie")
        #     goal = PoseStamped()
        #     goal.header.frame_id = "map"
        #     goal.pose.position.x = 4.0
        #     goal.pose.position.y = 1.0
        #     goal.pose.orientation.z = 0.1
        #     navigation_action_client.send_goal(goal)


        while rclpy.ok() and not navigation_action_client.goal_reached:
            rclpy.spin_once(navigation_action_client)

        rclpy.spin_once(navigation_action_client, timeout_sec=0.5)
    navigation_action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
