import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import socket
from time import sleep


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.xDir_dict = {
            1: 1, #'Forward',
            2: -1, #'back',
            3: 0 #'idle'
        }
        self.yDir_dict = {
            1: 1, #'left',
            2: -1,  #'right',
            3: 0, #'idle'
        }
        self.zRot_dict = {
            
            b'!': -1, #'right-1',
            b'"': -2, #'right-2',
            b'#': -3, #'right-3',
            b'$': -4, #'right-4',
            b'%': -5, #'right-5',
            b'&': -6, #'right-6',
            b'\'': -7, #'right-7',
            b'\x00': 0, #'rot-idle',
            b'\x03': 0,#'focus-mode-auto',
            b'\xff': 0, #'thumb-click',
            b'1' : 1, #'left-1',
            b'2' : 2, #'left-2',
            b'3' : 3, #'left-3',
            b'4' : 4, #'left-4',
            b'5' : 5, #'left-5',
            b'6' : 6, #'left-6',
            b'7' : 7, #'left-7',
        }

        self.xDir_percentage_multiplier = 1/24  #24 to -24
        self.xSpeed_percentage = 0 #used as multiplier (can be negative)
        self.xSpeed_max = 1.5 #meters/sec
        self.xSpeed_twist = 0 #stateful output for ROS2 twist commands

        self.yDir_percentage_multiplier = 1/24 #24 to -24
        self.ySpeed_percentage = 0 #used as multiplier (can be negative)
        self.ySpeed_max = 1.5 #meters/sec
        self.ySpeed_twist = 0 #stateful output for ROS2 twist commands

        self.zRot_percentage_multiplier = 1/7  #7 to -7
        self.zRotSpeed_percentage = 0 #used as multiplier (can be negative)
        self.zRotSpeed_max = 0.75 #rad/sec
        self.zRot_twist = 0 #stateful output for ROS2 twist commands

        # Socket setup
        self.port = 52381
        self.buffer_size = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('', self.port))
        
        self.acknowledge_message = bytearray.fromhex('90 4Y FF'.replace('Y', '1')) # 1 = socket number
        self.completion_message = bytearray.fromhex('90 5Y FF'.replace('Y', '1'))
        
        # Start a separate listener thread for socket communication
        self.create_timer(0.01, self.handle_socket_communication)

    def handle_socket_communication(self):
        try:
            data = self.s.recvfrom(self.buffer_size)
            message = data[0]
            address_port = data[1]
            payload_type = message[1:2]
            if payload_type == b'\x01': #control commands
        
                if(len(message)>7):
                    xDIR = message[7]
                    xDirection = self.xDir_dict[xDIR] 
                    self.xSpeed_percentage = (message[5] * self.xDir_percentage_multiplier)*xDirection
                    self.xSpeed_twist = self.xSpeed_percentage * self.xSpeed_max
                    # --------------------
                    yDIR = message[6]
                    yDirection = self.yDir_dict[yDIR] 
                    self.ySpeed_percentage = (message[4] * self.yDir_percentage_multiplier)*yDirection
                    self.ySpeed_twist = self.ySpeed_percentage * self.ySpeed_max 

                    # print("x-speed="+ str(self.xSpeed_twist)) 
                    # print("y-speed="+ str(self.ySpeed_twist)) 

                else:
                    if(message[4:5]==b'\x02'):
                        if(message[5]==255):
                            print("fosus-mode-manual")
                        else:
                            print("Preset Number")
                            print(message[5])

                    else:
                        zDIR = self.zRot_dict[message[4:5]] 
                        self.zRotSpeed_percentage = zDIR * self.zRot_percentage_multiplier
                        self.zRot_twist = self.zRotSpeed_percentage * self.zRotSpeed_max

                        # print( "z-rotation " + str(self.zRot_twist) )
            
            elif payload_type == b'\t': #connect commands
                print ( "connecting" )
            else:    
                print ( "unknown" )

            self.s.sendto(self.acknowledge_message, address_port)
            sleep(0.001)
            self.s.sendto(self.completion_message, address_port)

            #no matter what info has or has not been recieved, we publish a twist command of the stateful values 
            msg = Twist()
            msg.linear.x = float(self.xSpeed_twist)
            msg.linear.y = float(self.ySpeed_twist)
            msg.angular.z = float(self.zRot_twist)
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
            
        except BlockingIOError:
            pass  # Non-blocking mode: do nothing if no data is received
    

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
