# Awesome Webots [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)


> Interesting projects, papers, books, demos and other resources related to [Webots](https://github.com/cyberbotics/webots) robot simulator.

![Webots Cover](assets/cover.png)

Webots is a free and open-source 3D robot simulator used in industry, education and research. It includes a large collection of freely modifiable models of robots, sensors, actuators and objects. The robot controller programs can be written outside of Webots in C, C++, Python, ROS, Java and MATLAB using a simple API. Webots can stream a simulation on web browsers using WebGL. 

(source [Wikipedia](https://en.wikipedia.org/wiki/Webots))


## Contents

- [Installation](#installation)
- [Tools and Libraries](#tools-and-libraries)
- [Integrations](#itegrations)
- [Tutorials](#tutorials)
- [Community](#community)
- [Open Source Projects](#projects)
- [Simulations](#simulaitons)
- [Papers](#papers)
- [Books](#books)
- [Podcasts](#podcasts)
- [Competitions](#competitions)
- [Related Awesome LIsts](#related-awesome-lists)


## Installation

Besides the official installation methods there are unofficial ones like AUR and Homebrew.

- [Official](https://github.com/cyberbotics/webots/releases/tag/R2021a): Windows (standard Windows installer), Linux ([snap](https://snapcraft.io/webots) package, Debian package, and `tar.bz2` archive), and macOS (`.dmg` bundle).
- [Homebrew](https://formulae.brew.sh/cask/webots)
- [AUR](https://aur.archlinux.org/packages/webots/)

## Tools and Libraries

- [RobotBenchmark](https://robotbenchmark.net/): Program simulated robots online. Compare your performance to the best. Share your achievements.
- [urdf2webots](https://github.com/cyberbotics/urdf2webots): Utility to convert URDF files to Webots PROTO nodes.
- [Robot Designer](https://github.com/cyberbotics/robot-designer): Online tool that lets users build custom robots in an easy and quick way.
- [Webots for Visual Code](https://marketplace.visualstudio.com/items?itemName=pymzor.language-proto-webots): Webots-flavoured PROTO Syntax Highlighting Support in VSCode.
- [Webots for Atom](https://github.com/tn12787/PROTO-Webots): Webots-flavoured PROTO Syntax Highlighting Support in Atom.

## Integrations

- [webots_ros2](https://github.com/cyberbotics/webots_ros2): Webots interface for [ROS 2](http://docs.ros.org/en/foxy/).
- [Deepbots](https://github.com/aidudezzz/deepbots): Webots interface for [Open AI Gym](https://gym.openai.com/).
- [SITL with Webots](https://ardupilot.org/dev/docs/sitl-with-webots.html): Webots integration for [ArduPilot](https://ardupilot.org/).
- [Webots-Blockly](https://github.com/victorhu3/Webots-Blockly): Webots integration for [Blockly](https://developers.google.com/blockly).
- [Webots Animation](https://github.com/marketplace/actions/webots-animation): Webots integration for [GitHub Actions](https://github.com/features/actions).

## Tutorials

Tutorials on how to use Webots.

- [Webots Tutorial](https://cyberbotics.com/doc/guide/tutorials)
- [Webots User Guide](https://cyberbotics.com/doc/guide/index)
- [Webots Reference Manual](https://cyberbotics.com/doc/reference/index)
- [Webots Documentation for Automobiles](https://cyberbotics.com/doc/automobile/index)
- [Course "Introduction to Robotics" from Colorado](http://correll.cs.colorado.edu/?s=webots)
- [Course "Distributed Intelligent Systems" from EPFL](https://www.epfl.ch/labs/disal/teaching/distributed_intelligent_systems/exercises/)

### Video Tutorials

- [Tutorials by Soft illusion](https://www.youtube.com/playlist?list=PLt69C9MnPchlWEV5AEhfT2HajlE2SJ55V)
- [[Turkish] Tutorials by harunlakodla](https://www.youtube.com/playlist?list=PL2gHOyQeamXK-UtKcDOBqy1TC7iazIaWE)


## Community

- [Discord Channel](https://discord.com/invite/nTWbN9m)
- [Webots Blog](https://www.cyberbotics.com/doc/blog/Webots-2020-a-release)
- [Stack Overflow](https://stackoverflow.com/questions/tagged/webots)


## Open Source Projects

List of open source projects that use Webots.

- [Eurobot Platform](https://github.com/memristor/mep3): Simulation of multiple robots with passive odometry, ROS 2, custom [Navigation2](https://navigation.ros.org/) plugins, Behavior Trees, and more built around the [Eurobot](https://www.eurobot.org/) competition.
- [Deep Reinforcement Learning with PyTorch](https://github.com/LucasWaelti/RL_Webots): This repository shows how Deep Reinforcement Learning can be used within Webots.
- [DJI Mavic 2 Pro PID Controller](https://github.com/alpinmaarif/Webots-DJI-Mavic-2-Pro-PID-Controller): Webots Simulation about controlling the UAV Quadrotor DJI Mavic 2 Pro using PID Controller in Python Programming.
- [Robot Positioning Estimation using ML Techniques](https://github.com/joangerard/webots-thesis): Machine Learning techniques together with non-parametric filters (such as Particles Filter) for robot positioning estimation.
- [Webots in Jupyter Lab](https://github.com/RobInLabUJI/Webots-Docker): Run Webots streaming server in a Docker image with Jupyter Lab.
- [Micromouse in Webots](https://emstef.github.io/Micromouse/): E-Puck robot solves a 16×16 maze using localization, mapping, path planning and motion control.


## Simulations

List of simulations created with Webots.

- [Webots Boston Dynamics Spot](https://www.youtube.com/watch?v=b5mVe6dk0wI)
- [Webots DJI Mavic 2 PRO](https://www.youtube.com/watch?v=-hJssj_Vcw8)
- [Webots TIAGo++](https://www.youtube.com/watch?v=2KYpuaREQm0)
- [Webots Autonomous Vehicle Simulation](https://www.youtube.com/watch?v=RhzZ6Ao6Shc)
- [Webots Universal Robots UR5e Simulation](https://www.youtube.com/watch?v=WIY9ebqSXUc)
- [AT-ST Inspired Walker](https://twitter.com/mantisrobot/status/1254693299702714369)

## Papers

List of scientific papers related to Webots.

- [Webots: Professional Mobile Robot Simulation](https://journals.sagepub.com/doi/pdf/10.5772/5618)
- [Developing Khepera robot applications in a Webots environment](https://ieeexplore.ieee.org/abstract/document/903293)
- [Aibo and Webots: Simulation, wireless remote control and controller transfer](https://www.sciencedirect.com/science/article/abs/pii/S0921889006000327)
- [Cooperative multi-agent mapping and exploration in Webots](https://ieeexplore.ieee.org/abstract/document/4803950)


## Books

Books about Webots.

- [Webots: Symbiosis Between Virtual and Real Mobile Robots](https://link.springer.com/chapter/10.1007/3-540-68686-X_24)
- [Cyberbotics' Robot Curriculum](https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum) 


## Podcasts

- [Using Webots Robot Simulator With ROS With Olivier Michel](https://www.theconstructsim.com/webots-robot-simulator-ros-olivier-michel/)


## Competitions

This is a list of recent competitions that utilize Webots.
In case you plan to organize online competition you may find the list useful.

- [Student Robotics Competition](https://studentrobotics.org/docs/competition-simulator/) (2020)
- [RoboCupJunior Rescue Simulation](https://junior.forum.robocup.org/t/call-for-volunteers-teams-robocupjunior-rescue-simulation-virtual-workshops/1441) (2020)


## Related Awesome Lists
- [Awesome Robotics](https://github.com/kiloreux/awesome-robotics/)
- [Awesome Robotics Libraries](https://github.com/jslee02/awesome-robotics-libraries)
- [Awesome ROS2](https://github.com/fkromer/awesome-ros2)
- [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
- [Awesome Reinforcement Learning](https://github.com/aikorea/awesome-rl/)

## Contribute

Contributions welcome! Read the [contribution guidelines](contributing.md) first.


## License

[![CC0](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg)](https://creativecommons.org/publicdomain/zero/1.0)

To the extent possible under law, Darko Lukic has waived all copyright and
related or neighboring rights to this work.
