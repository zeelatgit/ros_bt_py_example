#!/usr/bin/env python3

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.node_data import NodeData
from rclpy.action import ActionClient
from robot_actions.action import RobotCommand
from std_msgs.msg import String
from ros_bt_py_interfaces.msg import Node as NodeMsg

ACTION_NAME = 'robot_action'  # Fixed action server name

@define_bt_node(
    NodeConfig(
        version='0.1.0',
        options={
            'command_topic': str,         # Topic to receive commands
        },
        inputs={
            'is_ready': bool,             # Ready flag from battery monitor
        },
        outputs={
            'success': bool,              # Action execution success
        },
        max_children=0
    )
)
class ActionClientNode(Leaf):
    """Action client that executes latest received command if ready."""
    
    def _do_setup(self):
        """Initialize command subscriber and action client."""
        try:
            # Initialize state
            self.goal_handle = None
            self.action_client = None
            self.latest_command = None
            
            # Subscribe to command topic
            self.command_sub = self.node.create_subscription(
                String,
                self.options['command_topic'],
                self.command_callback,
                1  # Keep only latest message
            )
            
            # Create action client 
            self.action_client = ActionClient(
                self.node,
                RobotCommand,
                ACTION_NAME
            )
            
            # Initialize outputs
            self.outputs['success'] = NodeData(False)
            
            self.get_logger().info(f"ActionClientNode setup with command topic {self.options['command_topic']}")
            return NodeMsg.IDLE

        except Exception as e:
            self.get_logger().error(f"Error in setup: {str(e)}")
            return NodeMsg.FAILED

    def _do_tick(self):
        """Execute the current command if not in warning state."""
        try:
            # Only process if we have a command and are ready
            if self.latest_command and self.inputs['is_ready'].data:
                command = self.latest_command
                self.latest_command = None  # Clear command after using it
                self._send_goal(command)
                return NodeMsg.RUNNING
            return NodeMsg.IDLE

        except Exception as e:
            self.get_logger().error(f"Error in tick: {str(e)}")
            return NodeMsg.FAILED

    def command_callback(self, msg):
        """Store only the latest command."""
        valid_commands = ['go_to_home', 'go_to_home_n_charge', 'execute_dummy_action']
        if msg.data in valid_commands:
            self.latest_command = msg.data
            self.get_logger().debug(f'Received new command: {msg.data}')

    def _send_goal(self, command):
        """Helper to send a new action goal."""
        goal = RobotCommand.Goal()
        goal.command = command

        self.get_logger().info(f'Sending goal: {command}')
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            self.outputs['success'] = NodeData(False)
            return

        self.get_logger().info('Goal accepted!')
        self.goal_handle = goal_handle

        # Get the result 
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle action result."""
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result.success}')
        
        self.outputs['success'] = NodeData(result.success)
        self.goal_handle = None

    def _do_untick(self):
        """Clean up when unticking."""
        if self.goal_handle and not self.goal_handle.done():
            self.goal_handle.cancel_goal_async()
        return NodeMsg.IDLE

    def _do_reset(self):
        """Reset the node state."""
        self.goal_handle = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        """Clean up subscribers and action client."""
        if self.action_client:
            if self.goal_handle and not self.goal_handle.done():
                self.goal_handle.cancel_goal_async()
            self.action_client.destroy()
            self.action_client = None
        
        if hasattr(self, 'command_sub'):
            self.node.destroy_subscription(self.command_sub)
            self.command_sub = None