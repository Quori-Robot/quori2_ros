
import rclpy

from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler,SimpleHTTPRequestHandler
import threading
import os

from rclpy.node import Node

class RH(BaseHTTPRequestHandler):

    def __init__(self,request,client_address,server):
        super().__init__(request=request,client_address=client_address,server=server)

       

    def do_GET(self):
            pass
      

    def do_POST(self):
        print("Got POST")

    def do_HEAD(self):
        print("Got HEAD")


class QuoriWebUI(Node):

    def __init__(self):
        super().__init__("quori_web_ui")

        self.html_path = os.path.abspath(os.curdir)
        self.port=8080
        self.address = "127.0.0.1"

        self.declare_parameter('html_path',self.html_path)
        self.declare_parameter('port',self.port)
        self.declare_parameter('address',self.address)
        

             
        self.html_path = self.get_parameter('html_path').value
        self.port = self.get_parameter('port').value
        self.address = self.get_parameter('address').value

        self.get_logger().info(f"HTML path: {self.html_path}")
        self.get_logger().info(f"HTML port: {self.port}")
        self.get_logger().info(f"HTML address: {self.address}")
       
        os.chdir(self.html_path)
        self.server = ThreadingHTTPServer((self.address,self.port), SimpleHTTPRequestHandler)

        self.server.node = self
        self.server.serve_forever()

        #thread = threading.Thread(target=)
        # thread.start()

    def run_server(self):
        self.server_forever()

def main(args=None):
    
    try:
        rclpy.init(args=args)

        web_ui = QuoriWebUI()

        rclpy.spin(web_ui)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
