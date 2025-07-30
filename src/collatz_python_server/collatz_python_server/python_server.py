import rclpy
from rclpy.node import Node
from rclpy import serialization
from collatz_core.msg import Num

import socket
import time

port = 4321     
target_port = 1234  

class Collatz_Py_Node(Node):

    def __init__(self):
        super().__init__('collatz_py')

        self.get_logger().info('Python Node Started')

        #get the parameter from launch file
        self.declare_parameter('target_IP', '127.0.0.1')
        self.target_ip = self.get_parameter('target_IP').get_parameter_value().string_value

        #sending a flag to the cpp program so it would start the process
        starting_flag = Num()
        starting_flag.num = -1
        self.send_data(starting_flag)
        self.get_logger().info('python: Initialization Started...')

        #server socket configuration
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', port))
        self.server_sock.listen(1)
        self.server_sock.settimeout(10.0)

        #publisher only used for rqt_plot
        self.publisher = self.create_publisher(Num, 'collatz_number', 10)

    #main process
    def collatz_process(self):

        while True:
            
            #receive a data from cpp
            data = self.receive_data()
            self.get_logger().info(f"python: Received: {data.num}")

            #divide the data if even
            while data.num % 2 == 0:
                data.num //= 2

                #publish & sleep a short time for better logging and visualizing
                self.graph_publisher(data)
                time.sleep(0.01)
            
            #finish the process if reached 1
            if data.num == 1:
                self.get_logger().info('python: Sequence reached 1, exiting...')
                #send number 1 to cpp program so that cpp would exit too
                self.send_data(data)
                rclpy.shutdown()
                return
            #send the data to cpp since its odd now
            else:
                self.get_logger().info(f'python: Sent: {data.num}')
                self.send_data(data)

    #method to send data over local network
    def send_data(self, data):

        #we serialize the ros message into raw bytes to be sent over netowrk
        serialized_data = serialization.serialize_message(data)

        #try to connect and send data repeatedly untill connection is sucsseful
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.target_ip, target_port))  
                sock.sendall(serialized_data)
                sock.close()
                break
            except Exception as e:
                self.get_logger().warn(f"Retrying send: {e}")
                time.sleep(0.5)

    #method to receive data over local network
    def receive_data(self):

        #listen on the chosen port to receive packets
        try:
            conn, _ = self.server_sock.accept()
            conn.settimeout(5.0)
            data = conn.recv(1024)
            conn.close()
        except Exception as e:
            self.get_logger().error(f"Receive failed: {e}")
            return Num()
        
        #return the deserilized format of bytes as a ros message
        return serialization.deserialize_message(data, Num)
    
    #method to publish data on topic
    def graph_publisher(self, data):
        self.publisher.publish(data)

    """after restarting the program, sometimes the server socket
    of the last program wasn't closed properly and would
    make the new program fail while configuration.
    so I use this destructor method to make sure
    after the program is done, the sockets are cloesd"""
    def __del__(self):
        try:
            self.server_sock.close()
        except:
            pass

#main program
def main(args=None):
    rclpy.init(args=args)
    node = Collatz_Py_Node()
    node.collatz_process()

if __name__ == '__main__':
    main()
