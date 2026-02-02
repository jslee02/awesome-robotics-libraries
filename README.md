# Awesome Robotics Libraries

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of robotics simulators and libraries.

## Contents
* [Simulators](#simulators)
* [Libraries](#libraries)
  * [Dynamics Simulation](#dynamics-simulation)
  * [Inverse Kinematics](#inverse-kinematics)
  * [Machine Learning](#machine-learning)
  * [Motion Planning and Control](#motion-planning-and-control)
  * [Optimization](#optimization)
  * [Robot Modeling](#robot-modeling)
  * [Robot Platform](#robot-platform)
  * [Reinforcement Learning for Robotics](#reinforcement-learning-for-robotics)
  * [SLAM](#slam)
  * [Vision](#vision)
  * [Fluid](#fluid)
  * [Grasping](#grasping)
  * [Humanoid Robotics](#humanoid-robotics)
  * [Multiphysics](#multiphysics)
  * [Math](#math)
  * [ETC](#etc)
* [Other Awesome Lists](#other-awesome-lists)

## [Simulators](#contents)

###### Free or Open Source

* [AI2-THOR](https://ai2thor.allenai.org/) - Python framework with a Unity backend, providing interaction, navigation, and manipulation support for household based robotic agents [[github](https://github.com/allenai/ai2thor) ![AI2-THOR](https://img.shields.io/github/stars/allenai/ai2thor.svg?style=flat&label=Star&maxAge=86400)]
* AirSim - Simulator based on Unreal Engine for autonomous vehicles [[github](https://github.com/Microsoft/AirSim) ![AirSim](https://img.shields.io/github/stars/Microsoft/AirSim.svg?style=flat&label=Star&maxAge=86400)]
* [ARGoS](https://www.argos-sim.info/) - Physics-based simulator designed to simulate large-scale robot swarms [[github](https://github.com/ilpincy/argos3) ![ARGoS](https://img.shields.io/github/stars/ilpincy/argos3.svg?style=flat&label=Star&maxAge=86400)]
* [ARTE](http://arvc.umh.es/arte/index_en.html) - Matlab toolbox focussed on robotic manipulators [[github](https://github.com/4rtur1t0/ARTE) ![ARTE](https://img.shields.io/github/stars/4rtur1t0/ARTE.svg?style=flat&label=Star&maxAge=86400)]
* [AVIS Engine](https://avisengine.com) - Autonomous Vehicles Intelligent simulation software, A Fast and robust simulator software for Autonomous vehicle development. [[github](https://github.com/AvisEngine/AVIS-Engine-Python-API) ![AVIS Engine](https://img.shields.io/github/stars/AvisEngine/AVIS-Engine-Python-API.svg?style=flat&label=Star&maxAge=86400)]
* [CARLA](https://carla.org/) - Open-source simulator for autonomous driving research [[github](https://github.com/carla-simulator/carla) ![CARLA](https://img.shields.io/github/stars/carla-simulator/carla.svg?style=flat&label=Star&maxAge=86400)]
* [CoppeliaSim](https://www.coppeliarobotics.com/) - Formaly V-REP. Virtual robot experimentation platform [[github](https://github.com/CoppeliaRobotics/CoppeliaSimLib) ![CoppeliaSim](https://img.shields.io/github/stars/CoppeliaRobotics/CoppeliaSimLib.svg?style=flat&label=Star&maxAge=86400)]
* [Gazebo](https://gazebosim.org/) - Dynamic multi-robot simulator [[github](https://github.com/gazebosim/gazebo-classic) ![Gazebo](https://img.shields.io/github/stars/gazebosim/gazebo-classic.svg?style=flat&label=Star&maxAge=86400)]
* [GraspIt!](http://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs [[github](https://github.com/graspit-simulator/graspit) ![GraspIt!](https://img.shields.io/github/stars/graspit-simulator/graspit.svg?style=flat&label=Star&maxAge=86400)]
* [Habitat-Sim](https://aihabitat.org/) - Simulation platform for research in embodied artificial intelligence [[github](https://github.com/facebookresearch/habitat-sim) ![Habitat-Sim](https://img.shields.io/github/stars/facebookresearch/habitat-sim.svg?style=flat&label=Star&maxAge=86400)]
* [Hexapod Robot Simulator](https://hexapod.netlify.app/) - Open-source hexapod robot inverse kinematics and gaits visualizer [[github](https://github.com/mithi/hexapod) ![Hexapod Robot Simulator](https://img.shields.io/github/stars/mithi/hexapod.svg?style=flat&label=Star&maxAge=86400)]
* [Gazebo Sim](https://gazebosim.org/) - Open source robotics simulator (formerly Ignition Gazebo) [[github](https://github.com/gazebosim/gz-sim) ![Gazebo Sim](https://img.shields.io/github/stars/gazebosim/gz-sim.svg?style=flat&label=Star&maxAge=86400)]
* [Isaac Sim](https://developer.nvidia.com/isaac/sim) - Nvidia's robotic simulation environment with GPU physics simulation and ray tracing
* [ManiSkill](https://github.com/haosulab/ManiSkill) - A robot simulation and behavior learning package powered by SAPIEN, with a strong focus on manipulation skills. [[github](https://github.com/haosulab/ManiSkill) ![ManiSkill](https://img.shields.io/github/stars/haosulab/ManiSkill.svg?style=flat&label=Star&maxAge=86400)]
* [MORSE](http://morse-simulator.github.io/) - Modular open robots simulation engine [[github](https://github.com/morse-simulator/morse) ![MORSE](https://img.shields.io/github/stars/morse-simulator/morse.svg?style=flat&label=Star&maxAge=86400)]
* [Neurorobotics Platform](https://neurorobotics.net/) - Internet-accessible simulation of robots controlled by spiking neural networks [[bitbucket](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)]
* [PyBullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) - An easy to use simulator for robotics and deep reinforcement learning [[github](https://github.com/bulletphysics/bullet3) ![PyBullet](https://img.shields.io/github/stars/bulletphysics/bullet3.svg?style=flat&label=Star&maxAge=86400)]
* [PyBullet_Industrial](https://pybullet-industrial.readthedocs.io/en/latest/) - A extension to PyBullet that allows for the simulation of various robotic manufacturing processes such as milling or 3D-printing. [[github](https://github.com/WBK-Robotics/pybullet_industrial) ![PyBullet_Industrial](https://img.shields.io/github/stars/WBK-Robotics/pybullet_industrial.svg?style=flat&label=Star&maxAge=86400)]
* [Robot Gui](http://robot.glumb.de/) - A three.js based 3D robot interface [[github](https://github.com/glumb/robot-gui) ![Robot Gui](https://img.shields.io/github/stars/glumb/robot-gui.svg?style=flat&label=Star&maxAge=86400)]
* [SAPIEN](https://sapien.ucsd.edu) - A realistic and physics-rich simulated environment that hosts a large-scale set for articulated objects. [[github](https://github.com/haosulab/SAPIEN) ![SAPIEN](https://img.shields.io/github/stars/haosulab/SAPIEN.svg?style=flat&label=Star&maxAge=86400)]
* [Simbad](http://simbad.sourceforge.net/) - A Java 3D robot simulator, enables to write own robot controller with modifying environment using available sensors.
* [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics) - Popular game engine that now offers open-source tools, tutorials, and resources for robotics simulation [[github](https://github.com/Unity-Technologies/Unity-Robotics-Hub) ![Unity](https://img.shields.io/github/stars/Unity-Technologies/Unity-Robotics-Hub.svg?style=flat&label=Star&maxAge=86400)]
* [Webots](http://www.cyberbotics.com/) - A complete development environment to model, program and simulate robots, vehicles and mechanical systems [[github](https://github.com/cyberbotics/webots) ![Webots](https://img.shields.io/github/stars/cyberbotics/webots.svg?style=flat&label=Star&maxAge=86400)]

###### Commercial

* [Actin Simulation](http://www.energid.com/)
* [Artiminds](https://www.artiminds.com/) - Planning, programming, operation, analysis and optimization
* [Kineo](https://www.plm.automation.siemens.com/global/en/products/plm-components/kineo.html) - Path planning and trajectory optimization for industrial robotics and digital mock-up review applications
* [RobotDK](https://robodk.com/) - Simulation and OLP for robots
* [RobotStudio](https://www.abb.com/global/en/areas/robotics/products/software/robotstudio-suite)
* [Robot Virtual Worlds](http://www.robotvirtualworlds.com/)
* [Virtual Robotics Toolkit](https://www.virtualroboticstoolkit.com/)
* [Visual Components](https://www.visualcomponents.com/)

###### Cloud

* [AWS RoboMaker](https://aws.amazon.com/robomaker/) - Service that makes it easy to develop, test, and deploy intelligent robotics applications at scale

## [Libraries](#contents)

### [Dynamics Simulation](#contents)

> :warning: The following table is not complete. Please feel free to report if you find something incorrect or missing.

| Name | Models | Features | Languages | Licenses | Code | Popularity |
|:----:| ------ | -------- | --------- | -------- | ---- | ---------- |
| [ARCSim](http://graphics.berkeley.edu/resources/ARCSim/index.html) | soft | — | C++ | — |  |  |
| [Bullet](https://pybullet.org/) | rigid, soft | ik, id, urdf, sdf | C++, Python | Zlib | [github](https://github.com/bulletphysics/bullet3) | ![Bullet](https://img.shields.io/github/stars/bulletphysics/bullet3.svg?style=flat&label=Star&maxAge=86400) |
| [CHRONO::ENGINE](https://projectchrono.org/) | rigid, soft, granular, fluid | ik, urdf | C++, Python | BSD-3-Clause | [github](https://github.com/projectchrono/chrono) | ![CHRONO::ENGINE](https://img.shields.io/github/stars/projectchrono/chrono.svg?style=flat&label=Star&maxAge=86400) |
| [DART](http://dartsim.github.io/) | rigid, soft | ik, id, plan, urdf, sdf | C++, Python | BSD-2-Clause | [github](https://github.com/dartsim/dart) | ![DART](https://img.shields.io/github/stars/dartsim/dart.svg?style=flat&label=Star&maxAge=86400) |
| [Drake](https://drake.mit.edu/) | rigid, aero, fluid | ik, trj-opt, plan | C++, Matlab | BSD-3-Clause | [github](https://github.com/RobotLocomotion/drake) | ![Drake](https://img.shields.io/github/stars/RobotLocomotion/drake.svg?style=flat&label=Star&maxAge=86400) |
| [Flex](https://developer.nvidia.com/flex) | rigid, soft, particle, fluid | — | C++ | — | [github](https://github.com/NVIDIAGameWorks/FleX) | ![Flex](https://img.shields.io/github/stars/NVIDIAGameWorks/FleX.svg?style=flat&label=Star&maxAge=86400) |
| [FROST](https://ayonga.github.io/frost-dev/index.html) | rigid | — | MATLAB | BSD-3-Clause | [github](https://github.com/ayonga/frost-dev) | ![FROST](https://img.shields.io/github/stars/ayonga/frost-dev.svg?style=flat&label=Star&maxAge=86400) |
| [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) | rigid, particle | — | C++ | Zlib |  |  |
| idyntree | rigid | id | C++, Python, Matlab, Lua | LGPL-2.1 | [github](https://github.com/gbionics/idyntree) | ![idyntree](https://img.shields.io/github/stars/gbionics/idyntree.svg?style=flat&label=Star&maxAge=86400) |
| [KDL](https://www.orocos.org/kdl.html) | rigid | ik | C++ | LGPL-2.1 | [github](https://github.com/orocos/orocos_kinematics_dynamics) | ![KDL](https://img.shields.io/github/stars/orocos/orocos_kinematics_dynamics.svg?style=flat&label=Star&maxAge=86400) |
| kindr | rigid | — | C++, Matlab | BSD-3-Clause | [github](https://github.com/ANYbotics/kindr) | ![kindr](https://img.shields.io/github/stars/ANYbotics/kindr.svg?style=flat&label=Star&maxAge=86400) |
| [Klampt](https://klampt.org/) | — | — | C++, Python | BSD-3-Clause | [github](https://github.com/krishauser/Klampt) | ![Klampt](https://img.shields.io/github/stars/krishauser/Klampt.svg?style=flat&label=Star&maxAge=86400) |
| [LibrePilot](http://www.librepilot.org/site/index.html) | uav, vehicles | — | C++ | GPL-3.0 | [bitbucket](https://bitbucket.org/librepilot/librepilot), [github](https://github.com/librepilot/LibrePilot) | ![LibrePilot](https://img.shields.io/github/stars/librepilot/LibrePilot.svg?style=flat&label=Star&maxAge=86400) |
| [MARS](http://rock-simulation.github.io/mars/) | — | — | C++, Python | LGPL-3.0 | [github](https://github.com/rock-simulation/mars) | ![MARS](https://img.shields.io/github/stars/rock-simulation/mars.svg?style=flat&label=Star&maxAge=86400) |
| [MBDyn](https://www.mbdyn.org/) | — | — | C++ | GPL-2.1 | [download](https://www.mbdyn.org/?Software_Download) |  |
| [MBSim](https://www.mbsim-env.de/) | — | — | C++ | (not specified) | [github](https://github.com/mbsim-env/mbsim) | ![MBSim](https://img.shields.io/github/stars/mbsim-env/mbsim.svg?style=flat&label=Star&maxAge=86400) |
| [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) | — | — | C++ | LGPL-3.0 | [github](https://github.com/SIM-TU-Darmstadt/mbslib) | ![MBSlib](https://img.shields.io/github/stars/SIM-TU-Darmstadt/mbslib.svg?style=flat&label=Star&maxAge=86400) |
| metapod | — | — | C++ | LGPL-3.0 | [github](https://github.com/laas/metapod) | ![metapod](https://img.shields.io/github/stars/laas/metapod.svg?style=flat&label=Star&maxAge=86400) |
| [Moby](http://physsim.sourceforge.net/index.html) | rigid | id | C++ | GPL-2.0 | [github](https://github.com/PositronicsLab/Moby) | ![Moby](https://img.shields.io/github/stars/PositronicsLab/Moby.svg?style=flat&label=Star&maxAge=86400) |
| [mrpt](https://www.mrpt.org/) | vehicle | slam, cv | C++, Python, Matlab | BSD-3-Clause | [github](https://github.com/MRPT/mrpt) | ![mrpt](https://img.shields.io/github/stars/MRPT/mrpt.svg?style=flat&label=Star&maxAge=86400) |
| [MuJoCo](https://mujoco.org/) | — | id | C++, Python | Apache-2.0 | [github](https://github.com/google-deepmind/mujoco) | ![MuJoCo](https://img.shields.io/github/stars/google-deepmind/mujoco.svg?style=flat&label=Star&maxAge=86400) |
| [mvsim](http://wiki.ros.org/mvsim) | vehicle | — | C++ | GPL-3.0 | [github](https://github.com/MRPT/mvsim) | ![mvsim](https://img.shields.io/github/stars/MRPT/mvsim.svg?style=flat&label=Star&maxAge=86400) |
| [Newton Dynamics](https://newtondynamics.com/) | — | — | C++ | Zlib | [github](https://github.com/MADEAPPS/newton-dynamics) | ![Newton Dynamics](https://img.shields.io/github/stars/MADEAPPS/newton-dynamics.svg?style=flat&label=Star&maxAge=86400) |
| [nphysics](https://nphysics.org/) | — | — | Rust | BSD-3-Clause | [github](https://github.com/dimforge/nphysics) | ![nphysics](https://img.shields.io/github/stars/dimforge/nphysics.svg?style=flat&label=Star&maxAge=86400) |
| [ODE](https://ode.org/) | rigid | — | C++ | LGPL-2.1 or BSD-3-Clause | [bitbucket](https://bitbucket.org/odedevs/ode) |  |
| [OpenRAVE](https://www.openrave.org/) | — | — | C++, Python | LGPL-3.0 | [github](https://github.com/rdiankov/openrave) | ![OpenRAVE](https://img.shields.io/github/stars/rdiankov/openrave.svg?style=flat&label=Star&maxAge=86400) |
| [pinocchio](https://stack-of-tasks.github.io/pinocchio/) | rigid | ik, id, urdf, analytical derivatives, code generation | C++, Python | BSD-2-Clause | [github](https://github.com/stack-of-tasks/pinocchio) | ![pinocchio](https://img.shields.io/github/stars/stack-of-tasks/pinocchio.svg?style=flat&label=Star&maxAge=86400) |
| PositionBasedDynamics | — | — | C++ | MIT | [github](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics) | ![PositionBasedDynamics](https://img.shields.io/github/stars/InteractiveComputerGraphics/PositionBasedDynamics.svg?style=flat&label=Star&maxAge=86400) |
| [PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/index.html) | — | — | C++ | unknown | [github](https://github.com/NVIDIA-Omniverse/PhysX) | ![PhysX](https://img.shields.io/github/stars/NVIDIA-Omniverse/PhysX.svg?style=flat&label=Star&maxAge=86400) |
| [PyDy](https://www.pydy.org/) | — | — | Python | BSD-3-Clause | [github](https://github.com/pydy/pydy) | ![PyDy](https://img.shields.io/github/stars/pydy/pydy.svg?style=flat&label=Star&maxAge=86400) |
| [RBDL](https://rbdl.github.io/) | rigid | ik, id, urdf | C++, Python | Zlib | [github](https://github.com/rbdl/rbdl) | ![RBDL](https://img.shields.io/github/stars/rbdl/rbdl.svg?style=flat&label=Star&maxAge=86400) |
| RBDyn | rigid | — | C++, Python | LGPL-3.0 | [github](https://github.com/jrl-umi3218/RBDyn) | ![RBDyn](https://img.shields.io/github/stars/jrl-umi3218/RBDyn.svg?style=flat&label=Star&maxAge=86400) |
| [RaiSim](https://slides.com/jeminhwangbo/raisim-manual) | — | — | C++ | [custom](https://github.com/leggedrobotics/raisimLib/blob/a9e7673569997f35c0bc7eb5d11bc4fa188e863c/LICENSE.md) | [github](https://github.com/leggedrobotics/raisimLib) | ![RaiSim](https://img.shields.io/github/stars/leggedrobotics/raisimLib.svg?style=flat&label=Star&maxAge=86400) |
| [ReactPhysics3d](https://www.reactphysics3d.com/) | — | — | C++ | Zlib | [github](https://github.com/DanielChappuis/reactphysics3d) | ![ReactPhysics3d](https://img.shields.io/github/stars/DanielChappuis/reactphysics3d.svg?style=flat&label=Star&maxAge=86400) |
| RigidBodyDynamics.jl | rigid | — | Julia | MIT "Expat" | [github](https://github.com/JuliaRobotics/RigidBodyDynamics.jl) | ![RigidBodyDynamics.jl](https://img.shields.io/github/stars/JuliaRobotics/RigidBodyDynamics.jl.svg?style=flat&label=Star&maxAge=86400) |
| [Rigs of Rods](https://www.rigsofrods.org/) | rigid, soft, vehicle | — | C++ | GPL-3.0 | [github](https://github.com/RigsOfRods/rigs-of-rods) | ![Rigs of Rods](https://img.shields.io/github/stars/RigsOfRods/rigs-of-rods.svg?style=flat&label=Star&maxAge=86400) |
| [Robopy](https://adityadua24.github.io/robopy/) | — | — | Python 3 | MIT | [github](https://github.com/adityadua24/robopy) | ![Robopy](https://img.shields.io/github/stars/adityadua24/robopy.svg?style=flat&label=Star&maxAge=86400) |
| [Robotics Library](https://www.roboticslibrary.org/) | — | — | C++ | GPL-3.0 or BSD-2-Clause | [github](https://github.com/roboticslibrary/rl) | ![Robotics Library](https://img.shields.io/github/stars/roboticslibrary/rl.svg?style=flat&label=Star&maxAge=86400) |
| [RobWork](https://robwork.dk/) | — | — | C++ | Apache-2.0 | [gitlab](https://gitlab.com/sdurobotics/RobWork) |  |
| [siconos](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/) | — | — | C++, Python | Apache-2.0 | [github](https://github.com/siconos/siconos) | ![siconos](https://img.shields.io/github/stars/siconos/siconos.svg?style=flat&label=Star&maxAge=86400) |
| [Simbody](https://simtk.org/home/simbody/) | rigid, molecules | id, urdf | C++ | Apache-2.0 | [github](https://github.com/simbody/simbody) | ![Simbody](https://img.shields.io/github/stars/simbody/simbody.svg?style=flat&label=Star&maxAge=86400) |
| [SOFA](https://www.sofa-framework.org/) | rigid, soft, medical | — | C++ | LGPL-2.1 | [github](https://github.com/sofa-framework/sofa) | ![SOFA](https://img.shields.io/github/stars/sofa-framework/sofa.svg?style=flat&label=Star&maxAge=86400) |
| Tiny Differentiable Simulator | rigid | — | C++, Python | Apache-2.0 | [github](https://github.com/erwincoumans/tiny-differentiable-simulator) | ![Tiny Differentiable Simulator](https://img.shields.io/github/stars/erwincoumans/tiny-differentiable-simulator.svg?style=flat&label=Star&maxAge=86400) |
| [trep](http://murpheylab.github.io/trep/) | rigid | dm, trj-opt | C, Python | GPL-3.0 | [github](https://github.com/MurpheyLab/trep) | ![trep](https://img.shields.io/github/stars/MurpheyLab/trep.svg?style=flat&label=Star&maxAge=86400) |
| qu3e | rigid | — | C++ | Zlib | [github](https://github.com/RandyGaul/qu3e) | ![qu3e](https://img.shields.io/github/stars/RandyGaul/qu3e.svg?style=flat&label=Star&maxAge=86400) |

For simplicity, shortened names are used to represent the supported models and features as

* Supported Models
  * rigid: rigid bodies
  * soft: soft bodies
  * aero: aerodynamics
  * granular: granular materials (like sand)
  * fluid: fluid dynamics
  * vehicles
  * uav: unmanned aerial vehicles (like drones)
  * medical
  * molecules
  * parallel: parallel mechanism (like Stewart platform)

* Features on Simulation, Analysis, Planning, Control Design
  * dm: [discrete mechanics](https://www.cambridge.org/core/journals/acta-numerica/article/div-classtitlediscrete-mechanics-and-variational-integratorsdiv/C8F45478A9290DEC24E63BB7FBE3CEB5)
  * ik: [inverse kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics) solvers (please find IK specialized packages in [this list](#inverse-kinematics))
  * id: [inverse dynamics](https://en.wikipedia.org/wiki/Inverse_dynamics)
  * slam: [simultaneous localization and mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
  * trj-opt: trajectory optimization
  * plan: motion planning algorithms
  * cv: computer vision
  * urdf: [urdf](http://wiki.ros.org/urdf) parser
  * sdf: [sdf](http://sdformat.org/) parser

### [Inverse Kinematics](#contents)

  * IKBT - A python package to solve robot arm inverse kinematics in symbolic form [[github](https://github.com/uw-biorobotics/IKBT) ![IKBT](https://img.shields.io/github/stars/uw-biorobotics/IKBT.svg?style=flat&label=Star&maxAge=86400)]
  * Kinpy - A simple pure python package to solve inverse kinematics [[github](https://github.com/neka-nat/kinpy) ![Kinpy](https://img.shields.io/github/stars/neka-nat/kinpy.svg?style=flat&label=Star&maxAge=86400)]
  * Lively - A highly configurable toolkit for commanding robots in mixed modalities [[github](https://github.com/Wisc-HCI/lively) ![Lively](https://img.shields.io/github/stars/Wisc-HCI/lively.svg?style=flat&label=Star&maxAge=86400)]
  * RelaxedIK - Real-time Synthesis of Accurate and Feasible Robot Arm Motion [[github](https://github.com/uwgraphics/relaxed_ik) ![RelaxedIK](https://img.shields.io/github/stars/uwgraphics/relaxed_ik.svg?style=flat&label=Star&maxAge=86400)]
  * [Trip](https://trip-kinematics.readthedocs.io/en/main/index.html) - A python package that solves inverse kinematics of parallel-, serial- or hybrid-robots [[github](https://github.com/TriPed-Robot/trip_kinematics) ![Trip](https://img.shields.io/github/stars/TriPed-Robot/trip_kinematics.svg?style=flat&label=Star&maxAge=86400)]

### [Machine Learning](#contents)

* [AllenAct](https://allenact.org/) - Python/PyTorch-based Research Framework for Embodied AI [[github](https://github.com/allenai/allenact) ![AllenAct](https://img.shields.io/github/stars/allenai/allenact.svg?style=flat&label=Star&maxAge=86400)]
* Any4LeRobot - A collection of utilities and tools for LeRobot [[github](https://github.com/Tavish9/any4lerobot) ![Any4LeRobot](https://img.shields.io/github/stars/Tavish9/any4lerobot.svg?style=flat&label=Star&maxAge=86400)]
* DLL - Deep Learning Library (DLL) for C++ [[github](https://github.com/wichtounet/dll) ![DLL](https://img.shields.io/github/stars/wichtounet/dll.svg?style=flat&label=Star&maxAge=86400)]
* [DyNet](https://dynet.readthedocs.io/en/latest/) - The Dynamic Neural Network Toolkit [[github](https://github.com/clab/dynet) ![DyNet](https://img.shields.io/github/stars/clab/dynet.svg?style=flat&label=Star&maxAge=86400)]
* [Fido](http://fidoproject.github.io/) - Lightweight C++ machine learning library for embedded electronics and robotics [[github](https://github.com/FidoProject/Fido) ![Fido](https://img.shields.io/github/stars/FidoProject/Fido.svg?style=flat&label=Star&maxAge=86400)]
* [Ivy](https://lets-unify.ai/) - Unified Machine Learning Framework [[github](https://github.com/ivy-llc/ivy) ![Ivy](https://img.shields.io/github/stars/ivy-llc/ivy.svg?style=flat&label=Star&maxAge=86400)]
* LeRobot - State-of-the-art approaches, pretrained models, datasets, and simulation environments for real-world robotics in PyTorch. [[github](https://github.com/huggingface/lerobot) ![LeRobot](https://img.shields.io/github/stars/huggingface/lerobot.svg?style=flat&label=Star&maxAge=86400)]
* [LeRobot Episode Scoring Toolkit](https://github.com/RoboticsData/score_lerobot_episodes) - One-click tool to score, filter, and export higher-quality LeRobot datasets. [[github](https://github.com/RoboticsData/score_lerobot_episodes) ![LeRobot Episode Scoring Toolkit](https://img.shields.io/github/stars/RoboticsData/score_lerobot_episodes.svg?style=flat&label=Star&maxAge=86400)]
* MiniDNN - A header-only C++ library for deep neural networks [[github](https://github.com/yixuan/MiniDNN) ![MiniDNN](https://img.shields.io/github/stars/yixuan/MiniDNN.svg?style=flat&label=Star&maxAge=86400)]
* [mlpack](https://www.mlpack.org/) - Scalable C++ machine learning library [[github](https://github.com/mlpack/mlpack) ![mlpack](https://img.shields.io/github/stars/mlpack/mlpack.svg?style=flat&label=Star&maxAge=86400)]
* [Gymnasium](https://gymnasium.farama.org/) - Developing and comparing reinforcement learning algorithms [[github](https://github.com/Farama-Foundation/Gymnasium) ![Gymnasium](https://img.shields.io/github/stars/Farama-Foundation/Gymnasium.svg?style=flat&label=Star&maxAge=86400)]
  * gym-dart [[github](https://github.com/DartEnv/dart-env) ![gym-dart](https://img.shields.io/github/stars/DartEnv/dart-env.svg?style=flat&label=Star&maxAge=86400)]
  * gym-gazebo [[github](https://github.com/erlerobot/gym-gazebo) ![gym-gazebo](https://img.shields.io/github/stars/erlerobot/gym-gazebo.svg?style=flat&label=Star&maxAge=86400)]
* RLLib - Temporal-difference learning algorithms in reinforcement learning [[github](https://github.com/samindaa/RLLib) ![RLLib](https://img.shields.io/github/stars/samindaa/RLLib.svg?style=flat&label=Star&maxAge=86400)]
* [robosuite](https://robosuite.ai) - A modular simulation framework and benchmark for robot learning [[github](https://github.com/ARISE-Initiative/robosuite) ![robosuite](https://img.shields.io/github/stars/ARISE-Initiative/robosuite.svg?style=flat&label=Star&maxAge=86400)]
* [tiny-dnn](http://tiny-dnn.readthedocs.io/en/latest/) - Header only, dependency-free deep learning framework in C++14 [[github](https://github.com/tiny-dnn/tiny-dnn) ![tiny-dnn](https://img.shields.io/github/stars/tiny-dnn/tiny-dnn.svg?style=flat&label=Star&maxAge=86400)]

### [Motion Planning and Control](#contents)


* [AIKIDO](https://github.com/personalrobotics/aikido) - Solving robotic motion planning and decision making problems. [[github](https://github.com/personalrobotics/aikido) ![AIKIDO](https://img.shields.io/github/stars/personalrobotics/aikido.svg?style=flat&label=Star&maxAge=86400)]
* Bioptim - Bioptim, a Python Framework for Musculoskeletal Optimal Control in Biomechanics [[github](https://github.com/pyomeca/bioptim) ![Bioptim](https://img.shields.io/github/stars/pyomeca/bioptim.svg?style=flat&label=Star&maxAge=86400)]
* [CuiKSuite](http://www.iri.upc.edu/people/porta/Soft/CuikSuite2-Doc/html) - Applications to solve position analysis and path planning problems
* [cuRobo](https://curobo.org) - A CUDA accelerated library containing a suite of robotics algorithms that run significantly faster. [[github](https://github.com/nvlabs/curobo) ![cuRobo](https://img.shields.io/github/stars/nvlabs/curobo.svg?style=flat&label=Star&maxAge=86400)]
* [Control Toolbox](https://ethz-adrl.github.io/ct/) - Open-Source C++ Library for Robotics, Optimal and Model Predictive Control [[github](https://github.com/ethz-adrl/control-toolbox) ![Control Toolbox](https://img.shields.io/github/stars/ethz-adrl/control-toolbox.svg?style=flat&label=Star&maxAge=86400)]
* Crocoddyl - Optimal control library for robot control under contact sequence [[github](https://github.com/loco-3d/crocoddyl) ![Crocoddyl](https://img.shields.io/github/stars/loco-3d/crocoddyl.svg?style=flat&label=Star&maxAge=86400)]
* Fields2Cover - Robust and efficient coverage paths for autonomous agricultural vehicles [[github](https://github.com/fields2cover/fields2cover) ![Fields2Cover](https://img.shields.io/github/stars/fields2cover/fields2cover.svg?style=flat&label=Star&maxAge=86400)]
* GPMP2 - Gaussian Process Motion Planner 2 [[github](https://github.com/gtrll/gpmp2) ![GPMP2](https://img.shields.io/github/stars/gtrll/gpmp2.svg?style=flat&label=Star&maxAge=86400)]
* [HPP](https://humanoid-path-planner.github.io/hpp-doc/) - Path planning for kinematic chains in environments cluttered with obstacles
* [MoveIt!](https://moveit.ai/) - Motion planning framework [[github](https://github.com/moveit/moveit) ![MoveIt!](https://img.shields.io/github/stars/moveit/moveit.svg?style=flat&label=Star&maxAge=86400)]
* [OMPL](https://ompl.kavrakilab.org/) - Open motion planning library [[github](https://github.com/ompl/ompl) ![OMPL](https://img.shields.io/github/stars/ompl/ompl.svg?style=flat&label=Star&maxAge=86400)]
* OCS2 - Efficient continuous and discrete time optimal control implementation [[github](https://github.com/leggedrobotics/ocs2) ![OCS2](https://img.shields.io/github/stars/leggedrobotics/ocs2.svg?style=flat&label=Star&maxAge=86400)]
* pymanoid - Humanoid robotics prototyping environment based on OpenRAVE [[github](https://github.com/stephane-caron/pymanoid) ![pymanoid](https://img.shields.io/github/stars/stephane-caron/pymanoid.svg?style=flat&label=Star&maxAge=86400)]
* ROS Behavior Tree [[github](https://github.com/miccol/ROS-Behavior-Tree) ![ROS Behavior Tree](https://img.shields.io/github/stars/miccol/ROS-Behavior-Tree.svg?style=flat&label=Star&maxAge=86400)]
* [Ruckig](https://github.com/pantor/ruckig) - Real-time, time-optimal and jerk-constrained online trajectory generation. [[github](https://github.com/pantor/ruckig) ![Ruckig](https://img.shields.io/github/stars/pantor/ruckig.svg?style=flat&label=Star&maxAge=86400)]
* [The Kautham Project](https://sir.upc.es/projects/kautham/) - A robot simulation toolkit for motion planning [[github](https://github.com/iocroblab/kautham) ![The Kautham Project](https://img.shields.io/github/stars/iocroblab/kautham.svg?style=flat&label=Star&maxAge=86400)]
* [TOPP-RA](https://hungpham2511.github.io/toppra/) - Time-parameterizing robot trajectories subject to kinematic and dynamic constraints [[github](https://github.com/hungpham2511/toppra) ![TOPP-RA](https://img.shields.io/github/stars/hungpham2511/toppra.svg?style=flat&label=Star&maxAge=86400)]
* [Ungar](https://github.com/fdevinc/ungar) - Expressive and efficient implementation of optimal control problems using template metaprogramming [[github](https://github.com/fdevinc/ungar) ![Ungar](https://img.shields.io/github/stars/fdevinc/ungar.svg?style=flat&label=Star&maxAge=86400)]

###### Motion Optimizer

* TopiCo - Time-optimal Trajectory Generation and Control [[github](https://github.com/AIS-Bonn/TopiCo) ![TopiCo](https://img.shields.io/github/stars/AIS-Bonn/TopiCo.svg?style=flat&label=Star&maxAge=86400)]
* [towr](http://wiki.ros.org/towr) - A light-weight, Eigen-based C++ library for trajectory optimization for legged robots [[github](https://github.com/ethz-adrl/towr) ![towr](https://img.shields.io/github/stars/ethz-adrl/towr.svg?style=flat&label=Star&maxAge=86400)]
* TrajectoryOptimization - A fast trajectory optimization library written in Julia [[github](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl) ![TrajectoryOptimization](https://img.shields.io/github/stars/RoboticExplorationLab/TrajectoryOptimization.jl.svg?style=flat&label=Star&maxAge=86400)]
* [trajopt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/) - Framework for generating robot trajectories by local optimization [[github](https://github.com/joschu/trajopt) ![trajopt](https://img.shields.io/github/stars/joschu/trajopt.svg?style=flat&label=Star&maxAge=86400)]

###### Nearest Neighbor

* [Cover-Tree](http://hunch.net/~jl/projects/cover_tree/cover_tree.html) - Cover tree data structure for quick k-nearest-neighbor search [[github](https://github.com/DNCrane/Cover-Tree) ![Cover-Tree](https://img.shields.io/github/stars/DNCrane/Cover-Tree.svg?style=flat&label=Star&maxAge=86400)]
  * [Faster cover trees](http://proceedings.mlr.press/v37/izbicki15.pdf) - by Mike Izbicki et al., ICML 2015.
* [FLANN](http://www.cs.ubc.ca/research/flann/) - Fast Library for Approximate Nearest Neighbors [[github](https://github.com/flann-lib/flann) ![FLANN](https://img.shields.io/github/stars/flann-lib/flann.svg?style=flat&label=Star&maxAge=86400)]
* [nanoflann](http://www.cs.ubc.ca/research/flann/) - Nearest Neighbor search with KD-trees [[github](https://github.com/jlblancoc/nanoflann) ![nanoflann](https://img.shields.io/github/stars/jlblancoc/nanoflann.svg?style=flat&label=Star&maxAge=86400)]

###### 3D Mapping

* [libpointmatcher](http://libpointmatcher.readthedocs.io/en/latest/) - Iterative Closest Point library for 2-D/3-D mapping in Robotics [[github](https://github.com/norlab-ulaval/libpointmatcher) ![libpointmatcher](https://img.shields.io/github/stars/norlab-ulaval/libpointmatcher.svg?style=flat&label=Star&maxAge=86400)]
* Octree - Fast radius neighbor search with an Octree [[github](https://github.com/jbehley/octree) ![Octree](https://img.shields.io/github/stars/jbehley/octree.svg?style=flat&label=Star&maxAge=86400)]
* [OctoMap](http://octomap.github.io/) - Efficient Probabilistic 3D Mapping Framework Based on Octrees [[github](https://github.com/OctoMap/octomap) ![OctoMap](https://img.shields.io/github/stars/OctoMap/octomap.svg?style=flat&label=Star&maxAge=86400)]
* [PCL](https://pointclouds.org/) - 2D/3D image and point cloud processing [[github](https://github.com/PointCloudLibrary/pcl) ![PCL](https://img.shields.io/github/stars/PointCloudLibrary/pcl.svg?style=flat&label=Star&maxAge=86400)]
* Bonxai - Brutally fast, sparse, 3D Voxel Grid (formerly Treexy) [[github](https://github.com/facontidavide/Bonxai) ![Bonxai](https://img.shields.io/github/stars/facontidavide/Bonxai.svg?style=flat&label=Star&maxAge=86400)]
* voxblox - Flexible voxel-based mapping focusing on truncated and Euclidean signed distance fields [[github](https://github.com/ethz-asl/voxblox) ![voxblox](https://img.shields.io/github/stars/ethz-asl/voxblox.svg?style=flat&label=Star&maxAge=86400)]
* [wavemap](https://projects.asl.ethz.ch/wavemap/) - Fast, efficient and accurate multi-resolution, multi-sensor 3D occupancy mapping [[github](https://github.com/ethz-asl/wavemap) ![wavemap](https://img.shields.io/github/stars/ethz-asl/wavemap.svg?style=flat&label=Star&maxAge=86400)]
* Utility Software
* [Goxel](https://guillaumechereau.github.io/goxel/) - Free and open source 3D voxel editor [[github](https://github.com/guillaumechereau/goxel) ![Goxel](https://img.shields.io/github/stars/guillaumechereau/goxel.svg?style=flat&label=Star&maxAge=86400)]

### [Optimization](#contents)

* [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic framework for algorithmic differentiation and numeric optimization [[github](https://github.com/casadi/casadi) ![CasADi](https://img.shields.io/github/stars/casadi/casadi.svg?style=flat&label=Star&maxAge=86400)]
* [Ceres Solver](http://ceres-solver.org/) - Large scale nonlinear optimization library [[github](https://github.com/ceres-solver/ceres-solver) ![Ceres Solver](https://img.shields.io/github/stars/ceres-solver/ceres-solver.svg?style=flat&label=Star&maxAge=86400)]
* eigen-qld - Interface to use the QLD QP solver with the Eigen3 library [[github](https://github.com/jrl-umi3218/eigen-qld) ![eigen-qld](https://img.shields.io/github/stars/jrl-umi3218/eigen-qld.svg?style=flat&label=Star&maxAge=86400)]
* EXOTica - Generic optimisation toolset for robotics platforms [[github](https://github.com/ipab-slmc/exotica) ![EXOTica](https://img.shields.io/github/stars/ipab-slmc/exotica.svg?style=flat&label=Star&maxAge=86400)]
* hpipm - High-performance interior-point-method QP solvers (Ipopt, Snopt) [[github](https://github.com/giaf/hpipm) ![hpipm](https://img.shields.io/github/stars/giaf/hpipm.svg?style=flat&label=Star&maxAge=86400)]
* [HYPRE](https://hypre.readthedocs.io/) - Parallel solvers for sparse linear systems featuring multigrid methods [[github](https://github.com/hypre-space/hypre) ![HYPRE](https://img.shields.io/github/stars/hypre-space/hypre.svg?style=flat&label=Star&maxAge=86400)]
* ifopt - An Eigen-based, light-weight C++ Interface to Nonlinear Programming Solvers (Ipopt, Snopt) [[github](https://github.com/ethz-adrl/ifopt) ![ifopt](https://img.shields.io/github/stars/ethz-adrl/ifopt.svg?style=flat&label=Star&maxAge=86400)]
* [Ipopt](https://projects.coin-or.org/Ipopt) - Large scale nonlinear optimization library [[github](https://github.com/coin-or/Ipopt) ![Ipopt](https://img.shields.io/github/stars/coin-or/Ipopt.svg?style=flat&label=Star&maxAge=86400)]
* libcmaes - Blackbox stochastic optimization using the CMA-ES algorithm [[github](https://github.com/CMA-ES/libcmaes) ![libcmaes](https://img.shields.io/github/stars/CMA-ES/libcmaes.svg?style=flat&label=Star&maxAge=86400)]
* [limbo](http://www.resibots.eu/limbo/) - Gaussian processes and Bayesian optimization of black-box functions [[github](https://github.com/resibots/limbo) ![limbo](https://img.shields.io/github/stars/resibots/limbo.svg?style=flat&label=Star&maxAge=86400)]
* lpsolvers - Linear Programming solvers in Python with a unified API [[github](https://github.com/stephane-caron/lpsolvers) ![lpsolvers](https://img.shields.io/github/stars/stephane-caron/lpsolvers.svg?style=flat&label=Star&maxAge=86400)]
* [NLopt](https://nlopt.readthedocs.io/en/latest/) - Nonlinear optimization [[github](https://github.com/stevengj/nlopt) ![NLopt](https://img.shields.io/github/stars/stevengj/nlopt.svg?style=flat&label=Star&maxAge=86400)]
* [OptimLib](https://www.kthohr.com/optimlib.html) - Lightweight C++ library of numerical optimization methods for nonlinear functions [[github](https://github.com/kthohr/optim) ![OptimLib](https://img.shields.io/github/stars/kthohr/optim.svg?style=flat&label=Star&maxAge=86400)]
* [OSQP](https://osqp.org/) - The Operator Splitting QP Solver [[github](https://github.com/osqp/osqp) ![OSQP](https://img.shields.io/github/stars/osqp/osqp.svg?style=flat&label=Star&maxAge=86400)]
* [Pagmo](https://esa.github.io/pagmo2/index.html) - Scientific library for massively parallel optimization [[github](https://github.com/esa/pagmo2) ![Pagmo](https://img.shields.io/github/stars/esa/pagmo2.svg?style=flat&label=Star&maxAge=86400)]
* [ProxSuite](https://simple-robotics.github.io/proxsuite/) - The Advanced Proximal Optimization Toolbox [[github](https://github.com/Simple-Robotics/ProxSuite) ![ProxSuite](https://img.shields.io/github/stars/Simple-Robotics/ProxSuite.svg?style=flat&label=Star&maxAge=86400)]
* [pymoo](https://www.pymoo.org/) - Multi-objective Optimization in Python [[github](https://github.com/msu-coinlab/pymoo) ![pymoo](https://img.shields.io/github/stars/msu-coinlab/pymoo.svg?style=flat&label=Star&maxAge=86400)]
* qpsolvers - Quadratic Programming solvers in Python with a unified API [[github](https://github.com/qpsolvers/qpsolvers) ![qpsolvers](https://img.shields.io/github/stars/qpsolvers/qpsolvers.svg?style=flat&label=Star&maxAge=86400)]
* [RobOptim](http://roboptim.net/index.html) - Numerical Optimization for Robotics. [[github](https://github.com/roboptim/roboptim-core) ![RobOptim](https://img.shields.io/github/stars/roboptim/roboptim-core.svg?style=flat&label=Star&maxAge=86400)]
* [SCS](http://web.stanford.edu/~boyd/papers/scs.html) - Numerical optimization for solving large-scale convex cone problems [[github](https://github.com/cvxgrp/scs) ![SCS](https://img.shields.io/github/stars/cvxgrp/scs.svg?style=flat&label=Star&maxAge=86400)]
* SHOT - A solver for mixed-integer nonlinear optimization problems [[github](https://github.com/coin-or/SHOT) ![SHOT](https://img.shields.io/github/stars/coin-or/SHOT.svg?style=flat&label=Star&maxAge=86400)]
* sferes2 - Evolutionary computation [[github](https://github.com/sferes2/sferes2) ![sferes2](https://img.shields.io/github/stars/sferes2/sferes2.svg?style=flat&label=Star&maxAge=86400)]

### [Robot Modeling](#contents)

###### Robot Model Description Format

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control [[bitbucket](https://bitbucket.org/osrf/sdformat)]
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model [[github](https://github.com/ros/urdfdom) ![urdf](https://img.shields.io/github/stars/ros/urdfdom.svg?style=flat&label=Star&maxAge=86400)]

###### Utility to Build Robot Models

* [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API [[github](https://github.com/Rhoban/onshape-to-robot) ![onshape-to-robot](https://img.shields.io/github/stars/Rhoban/onshape-to-robot.svg?style=flat&label=Star&maxAge=86400)]
* phobos - Add-on for Blender creating URDF and SMURF robot models [[github](https://github.com/dfki-ric/phobos) ![phobos](https://img.shields.io/github/stars/dfki-ric/phobos.svg?style=flat&label=Star&maxAge=86400)]

### [Robot Platform](#contents)

* [AutoRally](http://autorally.github.io/) - High-performance testbed for advanced perception and control research [[github](https://github.com/autorally/autorally) ![AutoRally](https://img.shields.io/github/stars/autorally/autorally.svg?style=flat&label=Star&maxAge=86400)]
* [Linorobot](https://linorobot.org/) - ROS compatible ground robots [[github](https://github.com/linorobot/linorobot) ![Linorobot](https://img.shields.io/github/stars/linorobot/linorobot.svg?style=flat&label=Star&maxAge=86400)]
  * onine - Service Robot based on Linorobot and Braccio Arm [[github](https://github.com/grassjelly/onine) ![onine](https://img.shields.io/github/stars/grassjelly/onine.svg?style=flat&label=Star&maxAge=86400)]
* [Micro-ROS for Arduino](https://github.com/kaiaai/micro_ros_arduino_kaiaai) - a Micro-ROS fork available in the Arduino Library Manager [[github](https://github.com/kaiaai/micro_ros_arduino_kaiaai) ![Micro-ROS for Arduino](https://img.shields.io/github/stars/kaiaai/micro_ros_arduino_kaiaai.svg?style=flat&label=Star&maxAge=86400)]
* [Rock](https://www.rock-robotics.org/) - Software framework for robotic systems
* [ROS](https://www.ros.org/) - Flexible framework for writing robot software
* [ROS 2](https://github.com/ros2/ros2/wiki) - Version 2.0 of the Robot Operating System (ROS) software stack [[github](https://github.com/ros2/ros2) ![ROS 2](https://img.shields.io/github/stars/ros2/ros2.svg?style=flat&label=Star&maxAge=86400)]
* [YARP](https://www.yarp.it/) - Communication and device interfaces applicable from humanoids to embedded devices [[github](https://github.com/robotology/yarp) ![YARP](https://img.shields.io/github/stars/robotology/yarp.svg?style=flat&label=Star&maxAge=86400)]

### [Reinforcement Learning for Robotics](#contents)

| Name | Stars |
|------|-------|
| [CleanRL](https://github.com/vwxyzjn/cleanrl) | [![GitHub stars](https://img.shields.io/github/stars/vwxyzjn/cleanrl.svg?style=flat&label=Star&maxAge=86400)](https://github.com/vwxyzjn/cleanrl) |
| [rl_games](https://github.com/Denys88/rl_games) | [![GitHub stars](https://img.shields.io/github/stars/Denys88/rl_games.svg?style=flat&label=Star&maxAge=86400)](https://github.com/Denys88/rl_games) |
| [SKRL](https://github.com/Toni-SM/skrl) | [![GitHub stars](https://img.shields.io/github/stars/Toni-SM/skrl.svg?style=flat&label=Star&maxAge=86400)](https://github.com/Toni-SM/skrl) |
| [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) | [![GitHub stars](https://img.shields.io/github/stars/DLR-RM/stable-baselines3.svg?style=flat&label=Star&maxAge=86400)](https://github.com/DLR-RM/stable-baselines3) |

### [SLAM](#contents)


* AprilSAM - Real-time smoothing and mapping [[github](https://github.com/xipengwang/AprilSAM) ![AprilSAM](https://img.shields.io/github/stars/xipengwang/AprilSAM.svg?style=flat&label=Star&maxAge=86400)]
* Cartographer - Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations [[github](https://github.com/cartographer-project/cartographer) ![Cartographer](https://img.shields.io/github/stars/cartographer-project/cartographer.svg?style=flat&label=Star&maxAge=86400)]
* [DSO](https://vision.in.tum.de/research/vslam/dso) - Novel direct and sparse formulation for Visual Odometry [[github](https://github.com/JakobEngel/dso) ![DSO](https://img.shields.io/github/stars/JakobEngel/dso.svg?style=flat&label=Star&maxAge=86400)]
* ElasticFusion - Real-time dense visual SLAM system [[github](https://github.com/mp3guy/ElasticFusion) ![ElasticFusion](https://img.shields.io/github/stars/mp3guy/ElasticFusion.svg?style=flat&label=Star&maxAge=86400)]
* [fiducials](http://wiki.ros.org/fiducials) - Simultaneous localization and mapping using fiducial markers [[github](https://github.com/UbiquityRobotics/fiducials) ![fiducials](https://img.shields.io/github/stars/UbiquityRobotics/fiducials.svg?style=flat&label=Star&maxAge=86400)]
* GTSAM - Smoothing and mapping (SAM) in robotics and vision [[github](https://github.com/borglab/gtsam) ![GTSAM](https://img.shields.io/github/stars/borglab/gtsam.svg?style=flat&label=Star&maxAge=86400)]
* Kintinuous - Real-time large scale dense visual SLAM system [[github](https://github.com/mp3guy/Kintinuous) ![Kintinuous](https://img.shields.io/github/stars/mp3guy/Kintinuous.svg?style=flat&label=Star&maxAge=86400)]
* [LSD-SLAM](https://vision.in.tum.de/research/vslam/lsdslam) - Real-time monocular SLAM [[github](https://github.com/tum-vision/lsd_slam) ![LSD-SLAM](https://img.shields.io/github/stars/tum-vision/lsd_slam.svg?style=flat&label=Star&maxAge=86400)]
* ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras [[github](https://github.com/raulmur/ORB_SLAM2) ![ORB-SLAM2](https://img.shields.io/github/stars/raulmur/ORB_SLAM2.svg?style=flat&label=Star&maxAge=86400)]
* [RTAP-Map](http://introlab.github.io/rtabmap/) - RGB-D Graph SLAM approach based on a global Bayesian loop closure detector [[github](https://github.com/introlab/rtabmap) ![RTAP-Map](https://img.shields.io/github/stars/introlab/rtabmap.svg?style=flat&label=Star&maxAge=86400)]
* [SRBA](http://mrpt.github.io/srba/) - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies [[github](https://github.com/MRPT/srba) ![SRBA](https://img.shields.io/github/stars/MRPT/srba.svg?style=flat&label=Star&maxAge=86400)]

#### SLAM Dataset

* [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets) [[github](https://github.com/youngguncho/awesome-slam-datasets) ![Awesome SLAM Datasets](https://img.shields.io/github/stars/youngguncho/awesome-slam-datasets.svg?style=flat&label=Star&maxAge=86400)]

### [Vision](#contents)

* [ViSP](http://visp.inria.fr/) - Visual Servoing Platform [[github](https://github.com/lagadic/visp) ![ViSP](https://img.shields.io/github/stars/lagadic/visp.svg?style=flat&label=Star&maxAge=86400)]
* [BundleTrack](https://github.com/wenbowen123/BundleTrack) - 6D Pose Tracking for Novel Objects without 3D Models [[github](https://github.com/wenbowen123/BundleTrack) ![BundleTrack](https://img.shields.io/github/stars/wenbowen123/BundleTrack.svg?style=flat&label=Star&maxAge=86400)]
* [se(3)-TrackNet](https://github.com/wenbowen123/iros20-6d-pose-tracking) - 6D Pose Tracking for Novel Objects without 3D Models [[github](https://github.com/wenbowen123/iros20-6d-pose-tracking) ![se(3)-TrackNet](https://img.shields.io/github/stars/wenbowen123/iros20-6d-pose-tracking.svg?style=flat&label=Star&maxAge=86400)]

### [Fluid](#contents)

* [Fluid Engine Dev - Jet](https://fluidenginedevelopment.org/) - Fluid simulation engine for computer graphics applications [[github](https://github.com/doyubkim/fluid-engine-dev) ![Fluid Engine Dev - Jet](https://img.shields.io/github/stars/doyubkim/fluid-engine-dev.svg?style=flat&label=Star&maxAge=86400)]

### [Grasping](#contents)

| Name | Stars |
|------|-------|
| [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) | [![GitHub stars](https://img.shields.io/github/stars/graspnet/anygrasp_sdk.svg?style=flat&label=Star&maxAge=86400)](https://github.com/graspnet/anygrasp_sdk) |
| [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) | [![GitHub stars](https://img.shields.io/github/stars/NVlabs/contact_graspnet.svg?style=flat&label=Star&maxAge=86400)](https://github.com/NVlabs/contact_graspnet) |
| [GraspNet API](https://github.com/graspnet/graspnetAPI) | [![GitHub stars](https://img.shields.io/github/stars/graspnet/graspnetAPI.svg?style=flat&label=Star&maxAge=86400)](https://github.com/graspnet/graspnetAPI) |
| [GraspIt!](https://graspit-simulator.github.io/) | [![GitHub stars](https://img.shields.io/github/stars/graspit-simulator/graspit.svg?style=flat&label=Star&maxAge=86400)](https://github.com/graspit-simulator/graspit) |

### [Humanoid Robotics](#contents)

| Name | Stars |
|------|-------|
| [Humanoid-Gym](https://github.com/roboterax/humanoid-gym) | [![GitHub stars](https://img.shields.io/github/stars/roboterax/humanoid-gym.svg?style=flat&label=Star&maxAge=86400)](https://github.com/roboterax/humanoid-gym) |
| [Legged Gym](https://github.com/leggedrobotics/legged_gym) | [![GitHub stars](https://img.shields.io/github/stars/leggedrobotics/legged_gym.svg?style=flat&label=Star&maxAge=86400)](https://github.com/leggedrobotics/legged_gym) |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | [![GitHub stars](https://img.shields.io/github/stars/google-deepmind/mujoco_menagerie.svg?style=flat&label=Star&maxAge=86400)](https://github.com/google-deepmind/mujoco_menagerie) |

### [Multiphysics](#contents)

* [Kratos](http://www.cimne.com/kratos/) - Framework for building parallel multi-disciplinary simulation software [[github](https://github.com/KratosMultiphysics/Kratos) ![Kratos](https://img.shields.io/github/stars/KratosMultiphysics/Kratos.svg?style=flat&label=Star&maxAge=86400)]

### [Math](#contents)

* Fastor - Light-weight high performance tensor algebra framework in C++11/14/17 [[github](https://github.com/romeric/Fastor) ![Fastor](https://img.shields.io/github/stars/romeric/Fastor.svg?style=flat&label=Star&maxAge=86400)]
* linalg.h - Single header public domain linear algebra library for C++11 [[github](https://github.com/sgorsten/linalg) ![linalg.h](https://img.shields.io/github/stars/sgorsten/linalg.svg?style=flat&label=Star&maxAge=86400)]
* manif - Small c++11 header-only library for Lie theory. [[github](https://github.com/artivis/manif) ![manif](https://img.shields.io/github/stars/artivis/manif.svg?style=flat&label=Star&maxAge=86400)]
* Sophus - Lie groups using Eigen [[github](https://github.com/strasdat/Sophus) ![Sophus](https://img.shields.io/github/stars/strasdat/Sophus.svg?style=flat&label=Star&maxAge=86400)]
* SpaceVelAlg - Spatial vector algebra with the Eigen3 [[github](https://github.com/jrl-umi3218/SpaceVecAlg) ![SpaceVelAlg](https://img.shields.io/github/stars/jrl-umi3218/SpaceVecAlg.svg?style=flat&label=Star&maxAge=86400)]
* spatialmath-python - A python package provides classes to represent pose and orientation in 3D and 2D space, along with a toolbox of spatial operations. [[github](https://github.com/bdaiinstitute/spatialmath-python) ![spatialmath-python](https://img.shields.io/github/stars/bdaiinstitute/spatialmath-python.svg?style=flat&label=Star&maxAge=86400)]

### [ETC](#contents)

* fuse - General architecture for performing sensor fusion live on a robot [[github](https://github.com/locusrobotics/fuse) ![fuse](https://img.shields.io/github/stars/locusrobotics/fuse.svg?style=flat&label=Star&maxAge=86400)]
* [Foxglove Studio](https://foxglove.dev) - A fully integrated visualization and debugging desktop app for your robotics data

## [Other Awesome Lists](#contents)

* [Awesome Robotics](https://github.com/Kiloreux/awesome-robotics) - (Kiloreux)
* [Awesome Robotics](https://github.com/ahundt/awesome-robotics) - (ahundt)
* [Awesome Robotic Tooling](https://github.com/Ly0n/awesome-robotic-tooling)
* [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence)
* [Awesome Collision Detection](https://github.com/jslee02/awesome-collision-detection)
* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
* [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning)
* [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning)
* [Awesome Gazebo](https://github.com/fkromer/awesome-gazebo)
* [Awesome Grasping](https://github.com/Po-Jen/awesome-grasping)
* [Awesome Human Robot Interaction](https://github.com/Po-Jen/awesome-human-robot-interaction)
* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python sample codes for robotics algorithms
* [Robotics Coursework](https://github.com/mithi/robotics-coursework) - A list of robotics courses you can take online

## [Contributing](#contents)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#contents)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
