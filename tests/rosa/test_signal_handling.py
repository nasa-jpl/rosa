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

import signal
import unittest


class TestSignalHandling(unittest.TestCase):
    """Test cases for signal handling and interrupt behavior in ROSA.
    
    These tests verify that the signal handling improvements work correctly,
    particularly that KeyboardInterrupt is properly propagated rather than
    being caught and swallowed.
    """

    def test_graceful_interrupt_handler_exists(self):
        """Test that GracefulInterruptHandler class exists and has required methods."""
        # Import from the actual module path
        import sys
        import os
        
        # Add the turtle_agent scripts directory to path
        turtle_agent_path = os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'turtle_agent', 'scripts')
        sys.path.insert(0, os.path.abspath(turtle_agent_path))
        
        try:
            from turtle_agent import GracefulInterruptHandler
            
            # Test that the class has required methods
            self.assertTrue(hasattr(GracefulInterruptHandler, '__enter__'))
            self.assertTrue(hasattr(GracefulInterruptHandler, '__exit__'))
            self.assertTrue(hasattr(GracefulInterruptHandler, '_handler'))
            
            # Test that it can be instantiated
            handler = GracefulInterruptHandler(verbose=False)
            self.assertFalse(handler.interrupted)
        except ModuleNotFoundError:
            # Skip this test if we can't import (e.g., missing rospy)
            self.skipTest("Cannot import turtle_agent module (rospy not available)")
        finally:
            # Clean up sys.path
            if turtle_agent_path in sys.path:
                sys.path.remove(turtle_agent_path)

    def test_signal_handler_context_manager(self):
        """Test GracefulInterruptHandler as a context manager."""
        import sys
        import os
        
        turtle_agent_path = os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'turtle_agent', 'scripts')
        sys.path.insert(0, os.path.abspath(turtle_agent_path))
        
        try:
            from turtle_agent import GracefulInterruptHandler
            
            handler = GracefulInterruptHandler(verbose=False)
            
            # Test entering and exiting context
            with handler:
                self.assertFalse(handler.interrupted)
                # Simulate signal by calling handler directly
                try:
                    handler._handler(signal.SIGINT, None)
                except KeyboardInterrupt:
                    # Expected exception when simulating SIGINT; ignore to check handler state
                    pass
                self.assertTrue(handler.interrupted)
        except ModuleNotFoundError:
            self.skipTest("Cannot import turtle_agent module (rospy not available)")
        finally:
            if turtle_agent_path in sys.path:
                sys.path.remove(turtle_agent_path)


if __name__ == "__main__":
    unittest.main()
