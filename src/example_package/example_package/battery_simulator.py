import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState 
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger
import random

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.get_logger().info('Battery Simulator initialized')
        self.battery_level = 100.0
        self.is_charging = False

        # Create a timer that fires every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create publishers for battery state and percentage
        self.pub = self.create_publisher(BatteryState, 'battery_level', 10)
        self.percent_pub = self.create_publisher(Float32, 'battery_percent', 10)
        
        # Initialize battery state message
        self.battery_msg = BatteryState()
        self.battery_msg.percentage = self.battery_level
        self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.battery_msg)

        # Publish initial percentage
        self.percent_msg = Float32()
        self.percent_msg.data = self.battery_level
        self.percent_pub.publish(self.percent_msg)

        # Create srevices to implement charging command
        self.start_chrg = self.create_service(Trigger, 'start_charging', self.start_chrg_callback)
        self.stop_chrg = self.create_service(Trigger, 'stop_charging', self.stop_chrg_callback)
        self.reset_batt = self.create_service(Trigger, 'reset_battery', self.reset_battery_callback)
        
    def timer_callback(self):
        if not self.is_charging:
            i = random.randint(1, 50)
            if i <= 5:
                self.battery_level = max(0.0, self.battery_level-random.randint(2, 5))
                self.publish_battery_state()
            else:
                if self.battery_level > 0.0:
                    self.battery_level = self.battery_level-1.0
                    self.publish_battery_state()
                elif self.battery_level == 0.0:
                    self.get_logger().info("Critical: Battery at 0%. Please charge!")
                    self.publish_battery_state()
        else:
            if self.battery_level < 100.0:
                if self.battery_level < 90.0:
                    self.battery_level = min(90.0, self.battery_level + 5)
                else:
                    self.battery_level = self.battery_level + (100.0 - self.battery_level)
                self.get_logger().info(f"Battery now at {self.battery_level:.1f}%")
                self.publish_battery_state()
            
            elif self.battery_level == 100.0:
                self.is_charging = False
                self.get_logger().info("Battery fully charged. Charging has now stopped.")

    def start_chrg_callback(self, request: Trigger.Request, response: Trigger.Response):

        self.is_charging = True
        response.success = True
        response.message = "Started Charging."

        self.get_logger().info("Started Charging.")

        return response
        

    def stop_chrg_callback(self, request: Trigger.Request, response: Trigger.Response):

        if self.is_charging:
            self.is_charging = False
            self.publish_battery_state()

            self.get_logger().info(f"Stopped charging. Battery level: {self.battery_level:.1f}%")
            response.success = True
            response.message = f"Stopped charging. Battery level: {self.battery_level:.1f}%"
        else:
            response.success = False
            response.message = "Cannot stop charging: device is not in charging mode"
        return response

    def reset_battery_callback(self, request: Trigger.Request, response: Trigger.Response):

        if self.is_charging:
            self.is_charging = False
            self.get_logger().info("Resetting battery, charging has now stopped")
        self.battery_level = 100.0
        self.get_logger().info("Battery level reset to 100%")
        self.publish_battery_state()
        response.success = True
        response.message = "Battery level reset to 100%"
        return response
    
    def publish_battery_state(self):
        # Update power supply status
        if self.is_charging:
            self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.battery_level >= 100.0:
            self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Publish BatteryState message
        self.battery_msg.percentage = self.battery_level
        self.battery_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.battery_msg)

        # Publish Float32 percentage message
        self.percent_msg.data = float(self.battery_level)
        self.percent_pub.publish(self.percent_msg)


def main(args=None):
    rclpy.init(args=args)
    battery_simulator = BatterySimulator()
    
    try:
        rclpy.spin(battery_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        battery_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()







