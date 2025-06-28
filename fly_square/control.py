import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission_node')

        # Service clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')

        # Publisher for velocity
        self.vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

    def wait_for_service(self, client, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec):
            self.get_logger().error(f'Service {client.srv_name()} not available')
            return False
        return True

    def call_mode_switch(self, mode_id: int):
        if not self.wait_for_service(self.mode_client):
            return
        req = ModeSwitch.Request()
        req.mode = mode_id
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().status:
            self.get_logger().info(f"Flight mode set to mode_id={mode_id}")
        else:
            self.get_logger().error("Failed to set mode")

    def call_arm(self):
        if not self.wait_for_service(self.arm_client):
            return
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().result:
            self.get_logger().info("Drone armed successfully")
        else:
            self.get_logger().error("Failed to arm")

    def call_takeoff(self, altitude=5.0):
        if not self.wait_for_service(self.takeoff_client):
            return
        req = Takeoff.Request()
        req.alt = float(altitude)
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Takeoff to {altitude} meters initiated")
        else:
            self.get_logger().error("Takeoff failed")

    def send_velocity(self, x=0.0, y=0.0, z=0.0, yaw_rate=0.0, duration=3):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.twist.angular.z = yaw_rate
        for _ in range(duration * 10):
            self.vel_pub.publish(msg)
            time.sleep(0.1)

    def fly_square(self):
        self.get_logger().info("Flying in square...")
        side_time = 25  # seconds per side

        self.send_velocity(x=1.0, duration=side_time)  # forward
        self.send_velocity(y=1.0, duration=side_time)  # right
        self.send_velocity(x=-1.0, duration=side_time) # backward
        self.send_velocity(y=-1.0, duration=side_time) # left

        self.send_velocity(0.0, 0.0, 0.0, 0.0, 2)  # stop

    def run_mission(self):
        self.get_logger().info("Starting mission...")

        self.call_mode_switch(4)  # GUIDED
        time.sleep(2)

        self.call_arm()
        time.sleep(2)

        self.call_takeoff(altitude=5.0)
        time.sleep(10)

        self.fly_square()
        time.sleep(10)
        

        self.get_logger().info("Switching to RTL...")
        self.call_mode_switch(6)  # RTL

def main(args=None):
    rclpy.init(args=args)
    mission = DroneMission()
    mission.run_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
