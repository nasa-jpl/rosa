# ROSA TurtleSim Web GUI

## Quick Start Guide

A modern web interface for interacting with the ROSA TurtleSim agent. This guide helps you get up and running quickly.

![ROSA TurtleSim Web Interface](https://via.placeholder.com/800x450.png?text=ROSA+TurtleSim+Web+Interface)

## Prerequisites

- Docker
- X11 server
- Web browser
- OpenAI API key

## Running the Web GUI

1. **Clone the repository:**
   ```bash
   git clone https://github.com/For-low/rosa.git
   cd rosa
   ```

2. **Set your OpenAI API key:**
   Create or edit `.env` file:
   ```
   OPENAI_API_KEY=your_api_key_here
   ```

3. **Start the container with web GUI enabled:**
   ```bash
   WEB_GUI=true ./demo.sh
   ```

4. **Access the web interface:**
   Open your browser and navigate to:
   ```
   http://localhost:5000
   ```

## Features

- **Modern Neo-Brutalist Design**: Clean, bold interface with monospace typography
- **Chat Interface**: Talk to the ROSA agent through a familiar chat interface
- **Example Commands**: One-click access to example commands
- **Responsive Design**: Works on desktop and mobile devices

## Example Commands

- "Move the turtle forward"
- "Draw a square"
- "Draw a 5-pointed star"
- "Teleport to position (3, 3) and draw a hexagon"
- "Change the background color to light blue"

## Stopping the Container

To stop the container, press Ctrl+C in the terminal or run:
```bash
docker stop rosa-turtlesim-demo
```

## For More Information

See `INSTRUCTIONS.txt` for detailed setup instructions and troubleshooting.

## License

Apache 2.0 