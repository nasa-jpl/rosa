<!-- Header block for project --> <hr>
<div align="center">
<!--   <img width="292" alt="ROSA_logo_dark_bg@2x" src="https://github.com/user-attachments/assets/7b4a8e64-9a08-4180-806a-5076d3672c05"> -->
<!--   <img width="213" alt="ROSA_sticker_color@2x" src="https://github.com/user-attachments/assets/5fa3a03e-5ef8-4942-84ac-95acf2f1777d"> -->
<!--   <img width="426" alt="ROSA_sticker_color@2x" src="https://github.com/user-attachments/assets/98b0a0ed-6b14-420c-83af-9067ab2d2d22"> -->
<!-- <img src="https://github.com/user-attachments/assets/d7175d5e-63d2-448c-b9d3-59ca0016ef7a"> -->
<img width="2057" alt="image" src="https://github.com/user-attachments/assets/ddbd3281-79f0-4d29-b0cd-30a5188ad061">

  
</div>
<div align="center">
  The ROS Agent (ROSA) is designed to interact with ROS-based<br>robotics systems using natural language queries. 🗣️🤖
</div>
<br>
<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2410.06472v1-b31b1b.svg)](https://arxiv.org/abs/2410.06472v1)
![ROS 1](https://img.shields.io/badge/ROS_1-Noetic-blue)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble|Iron|Jazzy-blue)
![License](https://img.shields.io/pypi/l/jpl-rosa)
[![SLIM](https://img.shields.io/badge/Best%20Practices%20from-SLIM-blue)](https://nasa-ammos.github.io/slim/)

![Main Branch](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/ci.yml?branch=main&label=main)
![Dev Branch](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/ci.yml?branch=dev&label=dev)
![Publish Status](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/publish.yml?label=publish)
![Version](https://img.shields.io/pypi/v/jpl-rosa)
![Downloads](https://img.shields.io/pypi/dw/jpl-rosa)

</div>
<!-- Header block for project -->

> [!IMPORTANT]
> 📚 **New to ROSA?** Check out our [Wiki](https://github.com/nasa-jpl/rosa/wiki) for documentation, guides and FAQs!


ROSA is your AI-powered assistant for ROS1 and ROS2 systems. Built on the [Langchain](https://python.langchain.com/v0.2/docs/introduction/) framework, ROSA helps you interact with robots using natural language, making robotics development more accessible and efficient.

#### ROSA Demo: NeBula-Spot in JPL's Mars Yard (click for YouTube)
[![Spot YouTube Thumbnail](https://github.com/user-attachments/assets/19a99b5c-6103-4be4-8875-1810cf4558c5)](https://www.youtube.com/watch?v=mZTrSg7tEsA)


## 🚀 Quick Start

### Requirements
- Python 3.9+
- ROS Noetic or higher

### Installation
```bash
pip3 install jpl-rosa
```

#### MacOS 

The following sections describe specific instructions for specific Mac hardware and operating system.

#### Sonoma 14.6.1, M2 Max (Anaconda Python)

The following instructions are based off a few things found on the internet, included [this helpful gist.](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088)

* Create a conda environment for python 3.9 and activate it
* Install jpl-rosa python package:
```bash
python -m pip install jpl-rosa
```
* Install MacOS homebrew if not already installed (https://brew.sh/)
* Update your brew packages:
```bash
brew update
```
* Install Xquartz (for X11 graphical render support)
```bash
brew install --cask xquartz
```
* Launch XQuarts:
```bash
open -a XQuartz
```
* You should see QXuartz in the top menu bar, choose settings, and make sure both options are checked in the X11 Preferences Security tab.
* Reboot your machine
* Allow X11 connections from anywhere:
```bash
xhost +
```
* Clone this repository and cd into its top-level directory
* Edit the ".env" file with at least your OPENAI_API_KEY
* Edit the "demo.sh" script and change the line "export DISPLAY=host.docker.internal:0" to "export=[IP_ADDRESS]:0" where IP_ADDRESS is your machine's IP address on the local internet.
* Edit the file "src/turtle_agent/scripts/llm.py" with these changes:
  * Add the following import statement at the top:  "from langchain_openai import ChatOpenAI"
  * Create an instance of ChatOpenAI() and make sure its return at the top of the function "def get_llm()"
* Launch the demo.sh script and wait several seconds for the Turtle_Sim window to appear
* When the docker container appears, start the simulation:
```bash
root@docker-desktop:/app# start streaming:=True
```
* Wait a few seconds and the simulation interactive console will appear and you can then type 'help' or 'examples' to get more information about commands you can run.
* The following commands worked for me:
```
> create a new turtle named 'george' at [3,3]
> move forward
```


#### Windows (WSL2)
The following sections describe instructions for a Windows 10 machine using WSL2 Ubuntu 22.04 and VSCode.

* Follow [this guide](https://learn.microsoft.com/en-us/windows/wsl/install) to install WSL2 Ubuntu on your machine
* Install [Chocolatey](https://chocolatey.org/install) package manager
* Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)

* Install jpl-rosa python package:
```bash
pip3 install jpl-rosa
```
* Install VcXsrv via Chocolatey (for X11 graphical render support)
```powershell
choco install vcxsrv
```
* When finished installing, launch VcXsrv (XLaunch)
  * Uncheck "Native opengl" and check "Disable access control"
* Launch Docker Desktop
* Clone this repository and cd into its top-level directory
* Configure VSCode to handle Linux line endings 
  * at the bottom bar, change the end of line sequence is LF for this directory
  * in settings, change "Files: Eol" to use "\n" line endings
* Make sure [Git is configured](https://docs.github.com/en/get-started/getting-started-with-git/configuring-git-to-handle-line-endings?platform=linux) handle Linux line endings
```bash
git config --global core.autocrlf input
git rm -rf --cached .
git reset --hard HEAD
```
* Edit the ".env" file with at least your OPENAI_API_KEY
* Edit the file "src/turtle_agent/scripts/llm.py" with these changes:
  * Add the following import statement at the top:  "from langchain_openai import ChatOpenAI"
  * Create an instance of ChatOpenAI() and make sure its return at the top of the function "def get_llm()"
* Launch the demo.sh script in WSL and wait several seconds for the Turtle_Sim window to appear
```bash
./demo.sh
```
* When the docker container appears, start the simulation:
```bash
root@docker-desktop:/app# start streaming:=True
```
* Wait a few seconds and the simulation interactive console will appear and you can then type 'help' or 'examples' to get more information about commands you can run.


### Usage Examples

```python
from rosa import ROSA

llm = get_your_llm_here()
agent = ROSA(ros_version=1, llm=llm)
agent.invoke("Show me a list of topics that have publishers but no subscribers")
```

For detailed information on configuring the LLM, please refer to our [Model Configuration Wiki page](https://github.com/nasa-jpl/rosa/wiki/Model-Configuration).


## Adapting ROSA for Your Robot 🔧

ROSA is designed to be easily adaptable to different robots and environments. You can create custom agents by either inheriting from the `ROSA` class or creating a new instance with custom parameters.

For detailed information on creating custom agents, adding tools, and customizing prompts, please refer to our [Custom Agents Wiki page](https://github.com/nasa-jpl/rosa/wiki/Custom-Agents).


## TurtleSim Demo 🐢

We have included a demo that uses ROSA to control the TurtleSim robot in simulation. To run the demo, you will need to have Docker installed on your machine. 🐳

The following video shows ROSA reasoning about how to draw a 5-point star, then 
executing the necessary commands to do so.

https://github.com/user-attachments/assets/77b97014-6d2e-4123-8d0b-ea0916d93a4e

For detailed instructions on setting up and running the TurtleSim demo, please refer to our [TurtleSim Demo Guide](https://github.com/nasa-jpl/rosa/wiki/Guide:-TurtleSim-Demo) in the Wiki.


## IsaacSim Extension (Coming Soon)

ROSA is coming to Nvidia IsaacSim! While you can already use ROSA with robots running in IsaacSim (using the ROS/ROS2 bridge), we are adding direct integration
in the form of an IsaacSim extension. This will allow you not only to control your robots in IsaacSim, but control IsaacSim itself. Check out the video below to learn mroe.

#### ROSA Demo: Nvidia IsaacSim Extension (click for YouTube)
[![Carter YouTube Thumbnail Play](https://github.com/user-attachments/assets/a6948d5e-2726-4dd8-8dee-19dfb5188f1d)](https://www.youtube.com/watch?v=mm5525G_EfQ)

## 📘 Learn More

- [📕 Read the paper](https://arxiv.org/abs/2410.06472v1)
- [🗺️ Roadmap](https://github.com/nasa-jpl/rosa/wiki/Feature-Roadmap)
- [🏷️ Releases](https://github.com/nasa-jpl/rosa/releases)
- [❓ FAQ](https://github.com/nasa-jpl/rosa/wiki/FAQ)


## Changelog

See our [CHANGELOG.md](CHANGELOG.md) for a history of our changes.  

## Contributing

Interested in contributing to our project? Please see our: [CONTRIBUTING.md](CONTRIBUTING.md)

For guidance on how to interact with our team, please see our code of conduct located
at: [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)

For guidance on our governance approach, including decision-making process and our various roles, please see our
governance model at: [GOVERNANCE.md](GOVERNANCE.md)

## License

See our: [LICENSE](LICENSE)

## Support

Key points of contact are:

- [@RobRoyce](https://github.com/RobRoyce) ([email](mailto:01-laptop-voiced@icloud.com))

---

<div align="center">
  ROSA: Robot Operating System Agent 🤖<br>
  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
</div>
