#!/usr/bin/env python3

from typing import Dict, Optional
from rclpy.node import Node
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from std_msgs.msg import Float32
from ros_bt_py_interfaces.msg import Node as NodeMsg

@define_bt_node(
    NodeConfig(
        version='0.1.0',
        options={
            'battery_level_topic': str,        # Topic to publish battery level
            'charging_cmd_topic': str,
            'discharge_rate': float,   # Battery discharge rate per tick
            'min_warning_level': float,  # Level for low battery warning
            'recovery_level': float,   # Level to clear low battery state 
        },
        optional_options=['discharge_rate', 'min_warning_level', 'recovery_level'],
        inputs={},  # No inputs needed
        outputs={
            'battery_level': float,    # Current battery level
            'error_msg': str,          # Error message if something goes wrong
            'is_warning': bool,        # True when battery is in warning state
            'is_critical': bool,       # True when battery is critically low
        },
        max_children=0  # This is a leaf node
    )
)
class BatteryMonitorNode(Leaf):
    """Battery simulator node that publishes decreasing battery levels."""
    
    def _do_setup(self):
        """Initialize battery level and publisher."""
        try:
            # Set default values 
            self.options['discharge_rate'] = self.options.get('discharge_rate', 0.1)
            self.options['min_warning_level'] = self.options.get('min_warning_level', 15.0)
            self.options['recovery_level'] = self.options.get('recovery_level', 20.0)

            # Initialize state
            self.battery_level = 100.0
            
            # Create publisher and subscriber
            self.publisher = self.ros_node.create_publisher(
                Float32,
                self.options['topic_name'],
                10
            )
            


            # Initialize outputs
            self.outputs['battery_level'] = 100.0
            self.outputs['error_msg'] = ""
            self.outputs['is_warning'] = False
            self.outputs['is_critical'] = False
            
            return NodeMsg.IDLE
        except Exception as e:
            self.outputs['error_msg'] = str(e)
            return NodeMsg.FAILED
    
    def battery_callback(self, msg):
        """Handle battery level updates, including resets."""
        if msg.data == 100.0:  # This is a reset signal
            self.get_logger().info('Received battery reset signal')
            self._do_reset()
        # Ignore other messages as we manage discharge internally

    def _do_tick(self):
        """Update battery level and publish."""
        try:
            # Update battery level
            self.battery_level = max(0.0, self.battery_level - self.options['discharge_rate'])
            
            # Update states
            # If already in warning state, need to go above recovery level to clear it
            if self.outputs['is_warning']:
                is_warning = self.battery_level < self.options['recovery_level']
            else:
                is_warning = self.battery_level < self.options['min_warning_level']
            is_critical = self.battery_level < 5.0

            # Publish battery level
            msg = Float32()
            msg.data = self.battery_level
            self.publisher.publish(msg)

            # Update outputs
            self.outputs['battery_level'] = self.battery_level
            self.outputs['error_msg'] = ""
            self.outputs['is_warning'] = is_warning
            self.outputs['is_critical'] = is_critical

            return NodeMsg.SUCCEEDED
        except Exception as e:
            self.outputs['error_msg'] = str(e)
            return NodeMsg.FAILED
    
    def _do_reset(self):
        """Reset battery level to 100%."""
        try:
            self.battery_level = 100.0
            self.outputs['battery_level'] = 100.0
            self.outputs['error_msg'] = ""
            self.outputs['is_warning'] = False
            self.outputs['is_critical'] = False
            return NodeMsg.IDLE
        except Exception as e:
            self.outputs['error_msg'] = str(e)
            return NodeMsg.FAILED
        
    def _do_shutdown(self):
        """Clean up publisher and subscriber."""
        if hasattr(self, 'publisher') and self.publisher:
            self.ros_node.destroy_publisher(self.publisher)
            self.publisher = None
            
        if hasattr(self, 'subscriber') and self.subscriber:
            self.ros_node.destroy_subscription(self.subscriber)
            self.subscriber = None
        
    def _do_untick(self):
        """Nothing special needed for untick."""
        return NodeMsg.IDLE