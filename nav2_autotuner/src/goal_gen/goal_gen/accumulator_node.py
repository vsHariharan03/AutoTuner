import rclpy
from rclpy.node import Node
from robot_interface.srv import Score
import socket
import json
import time

class AccumulatorNode(Node):

    def __init__(self):
        super().__init__('accumulator_node')

        self.srv = self.create_service(
            Score,
            'send_score',
            self.score_accumulate
        )

        self.declare_parameter('ros_domain', 0)
        self.domain = self.get_parameter('ros_domain').get_parameter_value().integer_value

        self.socket_path = f'/tmp/my_socket{self.domain}'

        self.curr_iter=0

        self.get_logger().info(f"Using socket {self.socket_path}")

    def score_accumulate(self, request, response):

        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

            client.connect(self.socket_path)

            param1=json.loads(request.params)
            param=dict()
            param['domain']=self.domain
            param['iter']=self.curr_iter
            param['score']=request.score
            param['time']=time.time()
            param['params']=param1
            self.curr_iter+=1
            


            msg = json.dumps(param)
            client.sendall(msg.encode())

            resp = client.recv(4096)

            if not resp:
                raise RuntimeError("Worker closed connection")

            response.success = True
            response.message = resp.decode().strip()

            client.close()

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main():
    rclpy.init()
    node = AccumulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()