"""
Testbed for LiDAR, solenoid plunger and servo:
  1. Read LiDAR
  2. If measured distrance is less than or equal to specified minimum distance of 1m,
  3. Turn servo 45 degrees and trigger solenoid plunger once
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import time
import numpy as np

from .servo_device import Servo_device
from .solenoidsw_device import SolenoidSwitch_device

servo_pin = 12
servo = Servo_device(servo_pin)

solenoidsw_pin = 21
solenoidsw = SolenoidSwitch_device(solenoidsw_pin)


class Scanner(Node):
  def __init__(self):
    super().__init__('scanner')
    self.subscription = self.create_subscription(
      LaserScan,
      'scan',
      self.listener_callback,
      qos_profile_sensor_data)
    self.subscription  # prevent unused variable warning
    
    # Variables/flag to detect minimum distance
    self.mindist = 0.3
    self.mindist_detected = False

  def listener_callback(self, msg):
    # create numpy array
    laser_range = np.array(msg.ranges)
    # replace 0's with nan
    laser_range[laser_range == 0] = np.nan

    measured_mindist = np.nanmin(laser_range)
    print("Measured minimum distance is:", measured_mindist)

    if measured_mindist <= self.mindist and not self.mindist_detected:
      self.mindist_detected = True
      servo.set_angle(45)
      solenoidsw.on()
      time.sleep(1)
    
    if measured_mindist > self.mindist and self.mindist_detected:
      self.mindist_detected = False
      servo.reset()
      solenoidsw.off()
      time.sleep(1)

def main(args=None):
  try:
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    servo.close()
    solenoidsw.close()

if __name__ == '__main__':
  main()
