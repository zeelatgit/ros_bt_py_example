#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_actions.action import RobotCommand
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState

class RobotActionServer(Node):
    def __init__(self):
        super().__init__('robot_action_server')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            RobotCommand,
            'robot_action',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Create location publisher with timestamp
        self.location_pub = self.create_publisher(String, '/location', 10)
        
        # Create charging service client
        self.charging_client = self.create_client(Trigger, 'start_charging')

        self.get_logger().info('Robot action server is ready')

    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        result = RobotCommand.Result()
        
        self.get_logger().info(f'Executing command: {command}')
        
        # Create stamped location message
        location_msg = String()
        location_msg.data = 'home' if command in ['go_to_home', 'go_to_home_n_charge'] else 'random_place'
        self.location_pub.publish(location_msg)
        self.get_logger().info(f'Published location: {location_msg.data}')
        
        if command == 'go_to_home':
            
            result.success = True
            result.message = "Executed go_to_home."
            
        elif command == 'go_to_home_n_charge':
            self.get_logger().info('Calling charging service...')
            # Call charging service
            await self.charging_client.call_async(Trigger.Request())
            self.get_logger().info('Charging service called successfully')
            
            result.success = True
            result.message = "Executed go_to_home_n_charge."
            
        elif command == 'execute_dummy_action':
            result.success = True
            result.message = "Executed dummy action."
            
        else:
            self.get_logger().error(f"Received unknown command: {command}")
            result.success = False
            result.message = f"Unknown command: {command}"
        
        self.get_logger().info(f'Action completed with result: {result.message}')    
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = RobotActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
