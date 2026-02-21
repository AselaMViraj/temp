#!/usr/bin/env python3

import http.server
import socketserver
import os
from ament_index_python.packages import get_package_share_directory

PORT = 5000

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve from the web_ui directory in the package share
        self.base_path = os.path.join(get_package_share_directory('cleaning_robot_bringup'), 'web_ui')
        super().__init__(*args, **kwargs, directory=self.base_path)

def main():
    with socketserver.TCPServer(("", PORT), MyHandler) as httpd:
        print(f"Serving Rover UI at http://localhost:{PORT}")
        print(f"To access from other devices, use http://<ROBOT_IP>:{PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            httpd.shutdown()

if __name__ == '__main__':
    main()
