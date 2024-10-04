#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import serial

ser = serial.Serial('/dev/ttyACM0', 57600) # Thay đổi /dev/ttyACM0 thành cổng serial của Arduino

def cmd_vel_callback(msg):
  # Nhận lệnh điều khiển động cơ từ ROS
  speed_left = msg.data[0]
  speed_right = msg.data[1]
  command = "<%d,%d>" % (speed_left, speed_right)
  ser.write(command.encode() + b'\n') # Gửi lệnh đến Arduino

def main():
  rospy.init_node('diffbot_serial_node')
  rospy.Subscriber('/cmd_vel', Int32MultiArray, cmd_vel_callback)
  pub_encoder = rospy.Publisher('/encoder', Int32MultiArray, queue_size=10)

  while not rospy.is_shutdown():
    # Đọc dữ liệu từ serial
    if ser.in_waiting > 0:
      data = ser.readline().decode().strip()
      # Parse dữ liệu encoder
      commaIndex = data.index(',')
      encoder_left = int(data[1:commaIndex])
      encoder_right = int(data[commaIndex+1:-1])
      # Publish dữ liệu encoder lên ROS
      encoder_msg = Int32MultiArray()
      encoder_msg.data = [encoder_left, encoder_right]
      pub_encoder.publish(encoder_msg)

    rospy.Rate(10).sleep() # Điều chỉnh tốc độ loop

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass