#!/usr/bin/env python3.9
import os
import sys
import threading
from flask import Flask, render_template, request, jsonify

# Add the parent directory to the path to import the turtle agent
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from llm import get_llm
from prompts import get_prompts
from rosa import ROSA
import tools.turtle as turtle_tools

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rosa-turtlesim-secret'

# Global ROSA instance
rosa_instance = None

def initialize_rosa():
    """Initialize the ROSA instance for the web interface"""
    global rosa_instance
    
    # Initialize ROSA with the same configuration as in turtle_agent.py
    llm = get_llm(streaming=False)
    prompts = get_prompts()
    blacklist = ["master", "docker"]
    
    rosa_instance = ROSA(
        ros_version=1,
        llm=llm,
        tool_packages=[turtle_tools],
        blacklist=blacklist,
        prompts=prompts,
        verbose=True,
        accumulate_chat_history=True,
        streaming=False,
    )
    
    return rosa_instance

@app.route('/')
def index():
    """Render the main web interface"""
    examples = [
        "Give me a ROS tutorial using the turtlesim.",
        "Show me how to move the turtle forward.",
        "Draw a 5-point star using the turtle.",
        "Teleport to (3, 3) and draw a small hexagon.",
        "Give me a list of nodes, topics, services, params, and log files.",
        "Change the background color to light blue and the pen color to red.",
    ]
    return render_template('index.html', examples=examples)

@app.route('/query', methods=['POST'])
def query():
    """Handle user query and return ROSA's response"""
    global rosa_instance
    
    if rosa_instance is None:
        initialize_rosa()
    
    user_query = request.json.get('query', '')
    if not user_query:
        return jsonify({'error': 'No query provided'}), 400
    
    try:
        # Get response from ROSA
        response = rosa_instance.invoke(user_query)
        return jsonify({'response': response})
    except Exception as e:
        print(f"Error processing query: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/clear', methods=['POST'])
def clear():
    """Clear the chat history"""
    global rosa_instance
    
    if rosa_instance:
        rosa_instance.clear_chat()
        return jsonify({'status': 'success'})
    return jsonify({'status': 'no active session'})

def run_flask():
    """Run Flask in a separate thread"""
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    # Initialize ROSA
    initialize_rosa()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    print("Web GUI is running at http://localhost:5000")
    
    try:
        # Keep the main thread alive
        flask_thread.join()
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0) 