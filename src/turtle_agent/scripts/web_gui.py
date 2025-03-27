#!/usr/bin/env python3.9
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import sys
import rospy

# Import the web app
from web.app import app, initialize_rosa

def main():
    """
    Launch the ROSA-TurtleSim web GUI
    """
    # Initialize ROS node
    rospy.init_node('rosa_turtle_web_gui', anonymous=True)
    
    # Initialize ROSA
    initialize_rosa()
    
    # Print startup info
    print("\n" + "="*50)
    print("ROSA TurtleSim Web GUI")
    print("="*50)
    print("Access the web interface at: http://localhost:5000")
    print("Press Ctrl+C to exit")
    print("="*50 + "\n")
    
    # Run the Flask application
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down web GUI...")
        sys.exit(0) 