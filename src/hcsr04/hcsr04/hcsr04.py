#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import lgpio

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_range_publisher')

        # Declare parameters
        self.declare_parameter('trig', 17)
        self.declare_parameter('echo', 18)
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('frame_id', 'ultrasonic_sensor')

        # Get parameter values
        self.trig = self.get_parameter('trig').get_parameter_value().integer_value
        self.echo = self.get_parameter('echo').get_parameter_value().integer_value
        publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publisher and timer
        self.publisher_ = self.create_publisher(Range, 'range', 10)
        timer_period = 1.0 / publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # GPIO setup
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.trig)
        lgpio.gpio_claim_input(self.h, self.echo)
        lgpio.gpio_write(self.h, self.trig, 0)
        time.sleep(2)

    def timer_callback(self):
        # Trigger ultrasonic pulse
        lgpio.gpio_write(self.h, self.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, self.trig, 0)

        # Measure echo pulse width
        timeout = time.time() + 1
        while lgpio.gpio_read(self.h, self.echo) == 0:
            if time.time() > timeout:
                self.get_logger().warn('Timeout waiting for echo HIGH')
                return
        pulse_start = time.time()

        timeout = time.time() + 1
        while lgpio.gpio_read(self.h, self.echo) == 1:
            if time.time() > timeout:
                self.get_logger().warn('Timeout waiting for echo LOW')
                return
        pulse_end = time.time()

        # Calculate distance in meters
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150 / 100  # cm to meters

        # Publish Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = 0.02
        msg.max_range = 1.0

        if (distance>msg.max_range): distance = msg.max_range
        msg.range = distance

        self.publisher_.publish(msg)

    def destroy_node(self):
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
