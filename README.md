# Awesome Robotics Libraries

A curated list of robotics simulators and libraries. 

#### Table of Contents
* [Simulators](#simulators)
* [Libraries](#libraries)
  * [Dynamics Simulation](#dynamics-simulation)
  * [Machine Learning](#machine-learning)
  * [Motion Planning and Control](#motion-planning-and-control)
  * [Optimization](#optimization)
  * [Robot Modeling](#robot-modeling)
  * [Robot Platform](#robot-platform)
  * [SLAM](#slam)
  * [Vision](#vision)
  * [Math](#math)
* [Other Awesome Lists](#other-awesome-lists)
* [Contributing](#contributing)

## [Simulators](#awesome-robotics-libraries)

###### Free or Open Source

* AirSim - Simulator based on Unreal Engine for autonomous vehicles [[github](https://github.com/Microsoft/AirSim) ![AirSim](https://img.shields.io/github/stars/Microsoft/AirSim.svg?style=flat&label=Star&maxAge=86400)]
* [ARTE](http://arvc.umh.es/arte/index_en.html) - Matlab toolbox focussed on robotic manipulators [[github](https://github.com/4rtur1t0/ARTE) ![4rtur1t0/ARTE](https://img.shields.io/github/stars/4rtur1t0/ARTE.svg?style=flat&label=Star&maxAge=86400)]
* [Gazebo](http://www.gazebosim.org/) - Dynamic multi-robot simulator [[bitbucket](https://bitbucket.org/osrf/gazebo)]
* [GraspIt!](http://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs [[github](https://github.com/graspit-simulator/graspit) ![graspit](https://img.shields.io/github/stars/graspit-simulator/graspit.svg?style=flat&label=Star&maxAge=86400)]
* [Isaac](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/) - Nvidia's virtual simulator for robots
* [MORSE](http://morse-simulator.github.io/) - Modular open robots simulation engine [[github](https://github.com/morse-simulator/morse) ![morse](https://img.shields.io/github/stars/morse-simulator/morse.svg?style=flat&label=Star&maxAge=86400)]
* [Neurorobotics Platform](https://neurorobotics.net/) - Internet-accessible simulation of robots controlled by spiking neural networks [[bitbucket](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)]
* [V-REP](http://www.coppeliarobotics.com/) - Virtual robot experimentation platform

###### Commercial

* [Actin Simulation](http://www.energid.com/)
* [RobotDK](https://robodk.com/) - Simulation and OLP for robots
* [RobotStudio](http://new.abb.com/products/robotics/robotstudio)
* [Robot Virtual Worlds](http://www.robotvirtualworlds.com/)
* [Virtual Robotics Toolkit](https://www.virtualroboticstoolkit.com/)
* [Visual Components](https://www.visualcomponents.com/)
* [Webots](http://www.cyberbotics.com/) - Robot simulator that provides a complete development environment

## [Libraries](#awesome-robotics-libraries)

### [Dynamics Simulation](#awesome-robotics-libraries)

> :warning: The following table is not complete. Please feel free to report if you find something incorrect or missing.

| Name | Models | Features | Languages | Licenses | Code | Popularity |
|:----:| ------ | -------- | --------- | -------- | ---- | ---------- |
| [ARCSim](http://graphics.berkeley.edu/resources/ARCSim/index.html) | soft |  | C++ | | |  |
| [Bullet](http://bulletphysics.org) | rigid, soft | ik, id | C++, Python | Zlib | [github](https://github.com/bulletphysics/bullet3) | ![bullet3](https://img.shields.io/github/stars/bulletphysics/bullet3.svg?style=flat&label=Star&maxAge=86400) |
| [CHRONO::ENGINE](http://chronoengine.info/) | rigid, soft, granular, fluid | ik, urdf | C++, Python | BSD-3-Clause | [github](https://github.com/projectchrono/chrono) | ![chrono](https://img.shields.io/github/stars/projectchrono/chrono.svg?style=flat&label=Star&maxAge=86400) |
| [DART](http://dartsim.github.io/) | rigid, soft | ik, id, plan, urdf, sdf | C++, Python | BSD-2-Clause | [github](https://github.com/dartsim/dart.git) | ![dart](https://img.shields.io/github/stars/dartsim/dart.svg?style=flat&label=Star&maxAge=86400) |
| [Drake](http://drake.mit.edu/) | rigid, aero, fluid | ik, trj-opt, plan | C++, Matlab | BSD-3-Clause | [github](https://github.com/RobotLocomotion/drake) | ![drake](https://img.shields.io/github/stars/RobotLocomotion/drake.svg?style=flat&label=Star&maxAge=86400) |
| [Flex](https://developer.nvidia.com/flex) | rigid, soft, particle, fluid  | | C++ | | [github](https://github.com/NVIDIAGameWorks/FleX) | ![NVIDIAGameWorks/FleX](https://img.shields.io/github/stars/NVIDIAGameWorks/FleX.svg?style=flat&label=Star&maxAge=86400) |
| [FROST](https://ayonga.github.io/frost-dev/index.html) | rigid  | | MATLAB | BSD-3-Clause | [github](https://github.com/ayonga/frost-dev) | ![ayonga/frost-dev](https://img.shields.io/github/stars/ayonga/frost-dev.svg?style=flat&label=Star&maxAge=86400) |
| [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) | rigid, particle | | C++ | Zlib | | |
| idyntree | rigid | id | C++, Python, Matlab, Lua | LGPL-2.1 | [github](https://github.com/robotology/idyntree) | ![idyntree](https://img.shields.io/github/stars/robotology/idyntree.svg?style=flat&label=Star&maxAge=86400) |
| [KDL](http://www.orocos.org/kdl) | rigid | ik | C++ | LGPL-2.1 | [github](https://github.com/orocos/orocos_kinematics_dynamics) | ![orocos_kinematics_dynamics](https://img.shields.io/github/stars/orocos/orocos_kinematics_dynamics.svg?style=flat&label=Star&maxAge=86400) |
| kindr | rigid | (todo) | C++, Matlab | BSD-3-Clause | [github](https://github.com/ethz-asl/kindr) | ![kindr](https://img.shields.io/github/stars/ethz-asl/kindr.svg?style=flat&label=Star&maxAge=86400) |
| [Klampt](http://motion.pratt.duke.edu/klampt/) | (todo) | (todo) | C++, Python | BSD-3-Clause | [github](https://github.com/krishauser/Klampt) | ![Klampt](https://img.shields.io/github/stars/krishauser/Klampt.svg?style=flat&label=Star&maxAge=86400) |
| [LibrePilot](http://www.librepilot.org/site/index.html) | uav, vehicles | (todo) | C++ | GPL-3.0 | [bitbucket](https://bitbucket.org/librepilot/librepilot), [github](https://github.com/librepilot/LibrePilot) | ![LibrePilot](https://img.shields.io/github/stars/librepilot/LibrePilot.svg?style=flat&label=Star&maxAge=86400) |
| [MARS](http://rock-simulation.github.io/mars/) | (todo) | (todo) | C++, Python | LGPL-3.0 | [github](https://github.com/rock-simulation/mars) | ![mars](https://img.shields.io/github/stars/rock-simulation/mars.svg?style=flat&label=Star&maxAge=86400) |
| [MBDyn](https://www.mbdyn.org/) | (todo) | (todo) | C++ | GPL-2.1 | [download](https://www.mbdyn.org/?Software_Download) | |
| [MBSim](mbsim-env/mbsim) | (todo) | (todo) | C++ | (not specified) | [github](https://github.com/mbsim-env/mbsim) | ![mbsim-env/mbsim](https://img.shields.io/github/stars/mbsim-env/mbsim.svg?style=flat&label=Star&maxAge=86400) |
| [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) | (todo) | (todo) | C++ | LGPL-3.0 | [github](https://github.com/SIM-TU-Darmstadt/mbslib) | ![mbslib](https://img.shields.io/github/stars/SIM-TU-Darmstadt/mbslib.svg?style=flat&label=Star&maxAge=86400) |
| metapod | (todo) | (todo) | C++ | LGPL-3.0 | [github](https://github.com/laas/metapod) | ![metapod](https://img.shields.io/github/stars/laas/metapod.svg?style=flat&label=Star&maxAge=86400) |
| [Moby](http://physsim.sourceforge.net/index.html) | rigid | id | C++ | GPL-2.0 | [github](https://github.com/PositronicsLab/Moby) | ![Moby](https://img.shields.io/github/stars/PositronicsLab/Moby.svg?style=flat&label=Star&maxAge=86400) |
| [mrpt](http://www.mrpt.org/) | vehicle | slam, cv | C++, Python, Matlab | BSD-3-Clause | [github](https://github.com/MRPT/mrpt) | ![mrpt](https://img.shields.io/github/stars/MRPT/mrpt.svg?style=flat&label=Star&maxAge=86400) |
| [MuJoCo](http://www.mujoco.org/index.html) | (todo) | id | C++, Python | [licenses](https://www.roboti.us/license.html) | closed source | |
| [Newton Dynamics](http://newtondynamics.com/) | (todo) | (todo) | C++ | Zlib | [github](https://github.com/MADEAPPS/newton-dynamics) | ![newton-dynamics](https://img.shields.io/github/stars/MADEAPPS/newton-dynamics.svg?style=flat&label=Star&maxAge=86400) |
| [nphysics](http://nphysics.org/) | (todo) | (todo) | Rust | BSD-3-Clause | [github](https://github.com/sebcrozet/nphysics) | ![sebcrozet/nphysics](https://img.shields.io/github/stars/sebcrozet/nphysics.svg?style=flat&label=Star&maxAge=86400) |
| [ODE](http://www.ode.org/) | rigid | | C++ | LGPL-2.1 or BSD-3-Clause | [bitbucket](https://bitbucket.org/odedevs/ode) | |
| [OpenRAVE](http://www.openrave.org) | (todo) | (todo) | C++, Python | LGPL-3.0 | [github](https://github.com/rdiankov/openrave) | ![openrave](https://img.shields.io/github/stars/rdiankov/openrave.svg?style=flat&label=Star&maxAge=86400) |
| [pinocchio](http://stack-of-tasks.github.io/pinocchio/) | rigid | ik, id | C++, Python | LGPL-3.0 | [github](https://github.com/stack-of-tasks/pinocchio) | ![pinocchio](https://img.shields.io/github/stars/stack-of-tasks/pinocchio.svg?style=flat&label=Star&maxAge=86400) |
| PositionBasedDynamics | (todo) | (todo) | C++ | MIT | [github](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics) | ![PositionBasedDynamics](https://img.shields.io/github/stars/InteractiveComputerGraphics/PositionBasedDynamics.svg?style=flat&label=Star&maxAge=86400) |
| [PyDy](http://www.pydy.org/) | (todo) | (todo) | Python | BSD-3-Clause | [github](https://github.com/pydy/pydy) | ![pydy](https://img.shields.io/github/stars/pydy/pydy.svg?style=flat&label=Star&maxAge=86400) |
| [RBDL](http://rbdl.bitbucket.org/) | rigid | ik | C++, Python | LGPL-3.0 | [bitbucket](https://bitbucket.org/rbdl/rbdl) | |
| RBDyn | rigid | (todo) | C++, Python | LGPL-3.0 | [github](https://github.com/jrl-umi3218/RBDyn) | ![RBDyn](https://img.shields.io/github/stars/jrl-umi3218/RBDyn.svg?style=flat&label=Star&maxAge=86400) |
| [ReactPhysics3d](http://www.reactphysics3d.com/) | (todo) | (todo) | C++ | Zlib | [github](https://github.com/DanielChappuis/reactphysics3d) | ![reactphysics3d](https://img.shields.io/github/stars/DanielChappuis/reactphysics3d.svg?style=flat&label=Star&maxAge=86400) |
| [Robopy](https://adityadua24.github.io/robopy/) | (todo) | (todo) | Python 3 | MIT | [github](https://github.com/adityadua24/robopy) | ![adityadua24/robopy](https://img.shields.io/github/stars/adityadua24/robopy.svg?style=flat&label=Star&maxAge=86400) |
| [Robotics Library](http://www.roboticslibrary.org/) | (todo) | (todo) | C++ | GPL-3.0 or BSD-2-Clause | [github](https://github.com/roboticslibrary/rl) | ![rl](https://img.shields.io/github/stars/roboticslibrary/rl.svg?style=flat&label=Star&maxAge=86400) |
| [RobWork](http://www.robwork.dk/apidoc/nightly/rw/index.html) | (todo) | (todo) | C++ | Apache-2.0 | [SVN](https://svnsrv.sdu.dk/svn/RobWork/trunk) (id: 'Guest', pw: '') | |
| [siconos](http://siconos.gforge.inria.fr) | (todo) | (todo) | C++, Python | Apache-2.0 | [github](https://github.com/siconos/siconos) | ![siconos](https://img.shields.io/github/stars/siconos/siconos.svg?style=flat&label=Star&maxAge=86400) |
| [Simbody](https://simtk.org/home/simbody/) | rigid, molecules | id, urdf | C++ | Apache-2.0 | [github](https://github.com/simbody/simbody.git) | ![simbody](https://img.shields.io/github/stars/simbody/simbody.svg?style=flat&label=Star&maxAge=86400) |
| [SOFA](https://www.sofa-framework.org/) | rigid, soft, medical | (todo) | C++ | LGPL-2.1 | [github](https://github.com/sofa-framework/sofa) | ![sofa](https://img.shields.io/github/stars/sofa-framework/sofa.svg?style=flat&label=Star&maxAge=86400) |
| [trep](http://murpheylab.github.io/trep/) | rigid | dm, trj-opt | C, Python | GPL-3.0 | [github](https://github.com/MurpheyLab/trep) | ![trep](https://img.shields.io/github/stars/MurpheyLab/trep.svg?style=flat&label=Star&maxAge=86400) |
| qu3e | rigid | - | C++ | Zlib | [github](https://github.com/RandyGaul/qu3e) | ![qu3e](https://img.shields.io/github/stars/RandyGaul/qu3e.svg?style=flat&label=Star&maxAge=86400) |

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
  * ik: [inverse kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics) solvers
  * id: [inverse dynamics](https://en.wikipedia.org/wiki/Inverse_dynamics)
  * slam: [simultaneous localization and mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
  * trj-opt: trajectory optimization
  * plan: motion planning algorithms
  * cv: computer vision
  * urdf: [urdf](http://wiki.ros.org/urdf) parser
  * sdf: [sdf](http://sdformat.org/) parser

### [Machine Learning](#awesome-robotics-libraries)

  * [Fido](http://fidoproject.github.io/) - Lightweight C++ machine learning library for embedded electronics and robotics [[github](http://github.com/FidoProject/Fido) ![FidoProject/Fido](https://img.shields.io/github/stars/FidoProject/Fido.svg?style=flat&label=Star&maxAge=86400)]
  * libdeep - C/C++ library for deep learning [[github](http://github.com/bashrc/libdeep) ![bashrc/libdeep](https://img.shields.io/github/stars/bashrc/libdeep.svg?style=flat&label=Star&maxAge=86400)]
  * [OpenAI Gym](https://gym.openai.com/) - Developing and comparing reinforcement learning algorithms [[github](http://github.com/openai/gym) ![gym](https://img.shields.io/github/stars/openai/gym.svg?style=flat&label=Star&maxAge=86400)]
    * gym-dart [[github](http://github.com/DartEnv/dart-env) ![dart-env](https://img.shields.io/github/stars/DartEnv/dart-env.svg?style=flat&label=Star&maxAge=86400)]
    * gym-gazebo [[github](http://github.com/erlerobot/gym-gazebo) ![dart-env](https://img.shields.io/github/stars/erlerobot/gym-gazebo.svg?style=flat&label=Star&maxAge=86400)]

### [Motion Planning and Control](#awesome-robotics-libraries)

* [Aikido](https://github.com/personalrobotics/aikido) - Solving robotic motion planning and decision making problems. [[github](https://github.com/personalrobotics/aikido) ![aikido](https://img.shields.io/github/stars/personalrobotics/aikido.svg?style=flat&label=Star&maxAge=86400)]
* [CuiKSuite](http://www.iri.upc.edu/people/porta/Soft/CuikSuite2-Doc/html) - Applications to solve position analysis and path planning problems
* [Control Toolbox](https://adrlab.bitbucket.io/ct/v2.1/ct_doc/doc/html/index.html) - Control, estimation, optimization and motion planning in robotics [[bitbucket](https://bitbucket.org/adrlab/ct)]
* [HPP](https://humanoid-path-planner.github.io/hpp-doc/) - Path planning for kinematic chains in environments cluttered with obstacles [[github](https://github.com/humanoid-path-planner)]
* [MoveIt!](http://moveit.ros.org/) - Motion planning framework [[github](https://github.com/ros-planning/moveit) ![moveit](https://img.shields.io/github/stars/ros-planning/moveit.svg?style=flat&label=Star&maxAge=86400)]
* [OMPL](http://ompl.kavrakilab.org/) - Open motion planning library [[bitbucket](https://bitbucket.org/ompl/ompl), [github](https://github.com/ompl/ompl) ![ompl](https://img.shields.io/github/stars/ompl/ompl.svg?style=flat&label=Star&maxAge=86400)]
* ROS Behavior Tree - [[github](https://github.com/miccol/ROS-Behavior-Tree) ![miccol/ROS-Behavior-Tree](https://img.shields.io/github/stars/miccol/ROS-Behavior-Tree.svg?style=flat&label=Star&maxAge=86400)]

###### Motion Optimizer

* [trajopt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/) - Framework for generating robot trajectories by local optimization [[github](https://github.com/joschu/trajopt) ![joschu/trajopt](https://img.shields.io/github/stars/joschu/trajopt.svg?style=flat&label=Star&maxAge=86400)]

###### Nearest Neighbor

* [Cover-Tree](http://hunch.net/~jl/projects/cover_tree/cover_tree.html) - Cover tree data structure for quick k-nearest-neighbor search [[github](https://github.com/DNCrane/Cover-Tree) ![Cover-Tree](https://img.shields.io/github/stars/DNCrane/Cover-Tree.svg?style=flat&label=Star&maxAge=86400)]
  * [Faster cover trees](http://proceedings.mlr.press/v37/izbicki15.pdf) by Mike Izbicki et al., ICML 2015.
* [FLANN](http://www.cs.ubc.ca/research/flann/) - Fast Library for Approximate Nearest Neighbors [[github](https://github.com/mariusmuja/flann) ![flann](https://img.shields.io/github/stars/mariusmuja/flann.svg?style=flat&label=Star&maxAge=86400)]
* [nanoflann](http://www.cs.ubc.ca/research/flann/) - Nearest Neighbor search with KD-trees [[github](https://github.com/jlblancoc/nanoflann) ![nanoflann](https://img.shields.io/github/stars/jlblancoc/nanoflann.svg?style=flat&label=Star&maxAge=86400)]

###### 3D Mapping

* [libpointmatcher](http://libpointmatcher.readthedocs.io/en/latest/) - Iterative Closest Point library for 2-D/3-D mapping in Robotics [[github](https://github.com/ethz-asl/libpointmatcher) ![ethz-asl/libpointmatcher](https://img.shields.io/github/stars/ethz-asl/libpointmatcher.svg?style=flat&label=Star&maxAge=86400)]
* [OctoMap](http://octomap.github.io/) - Efficient Probabilistic 3D Mapping Framework Based on Octrees [[github](https://github.com/OctoMap/octomap) ![octomap](https://img.shields.io/github/stars/OctoMap/octomap.svg?style=flat&label=Star&maxAge=86400)]
* [PCL](http://www.pointclouds.org/) - 2D/3D image and point cloud processing [[github](https://github.com/PointCloudLibrary/pcl) ![PointCloudLibrary/pcl](https://img.shields.io/github/stars/PointCloudLibrary/pcl.svg?style=flat&label=Star&maxAge=86400)]
* voxblox - Flexible voxel-based mapping focusing on truncated and Euclidean signed distance fields [[github](https://github.com/ethz-asl/voxblox) ![voxblox](https://img.shields.io/github/stars/ethz-asl/voxblox.svg?style=flat&label=Star&maxAge=86400)]

### [Optimization](#awesome-robotics-libraries)

* [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic framework for algorithmic differentiation and numeric optimization [[github](https://github.com/casadi/casadi) ![casadi](https://img.shields.io/github/stars/casadi/casadi.svg?style=flat&label=Star&maxAge=86400)]
* [Ceres Solver](http://ceres-solver.org/) - Large scale nonlinear optimization library [[github](https://github.com/ceres-solver/ceres-solver) ![ceres-solver](https://img.shields.io/github/stars/ceres-solver/ceres-solver.svg?style=flat&label=Star&maxAge=86400)]
* [Ipopt](https://projects.coin-or.org/Ipopt) - Large scale nonlinear optimization library [[github](https://github.com/coin-or/Ipopt) ![Ipopt](https://img.shields.io/github/stars/coin-or/Ipopt.svg?style=flat&label=Star&maxAge=86400)]
* libcmaes - Blackbox stochastic optimization using the CMA-ES algorithm [[github](https://github.com/beniz/libcmaes) ![beniz/libcmaes](https://img.shields.io/github/stars/beniz/libcmaes.svg?style=flat&label=Star&maxAge=86400)]
* [limbo](http://www.resibots.eu/limbo/) - Gaussian processes and Bayesian optimization of black-box functions [[github](https://github.com/resibots/limbo) ![resibots/limbo](https://img.shields.io/github/stars/resibots/limbo.svg?style=flat&label=Star&maxAge=86400)]
* [NLopt](http://ab-initio.mit.edu/wiki/index.php/NLopt) - Nonlinear optimization [[github](https://github.com/stevengj/nlopt) ![nlopt](https://img.shields.io/github/stars/stevengj/nlopt.svg?style=flat&label=Star&maxAge=86400)]
* [RobOptim](http://roboptim.net/index.html) - Numerical Optimization for Robotics. [[github](https://github.com/roboptim/roboptim-core) ![roboptim/roboptim-core](https://img.shields.io/github/stars/roboptim/roboptim-core.svg?style=flat&label=Star&maxAge=86400)]
* [SCS](http://web.stanford.edu/~boyd/papers/scs.html) - Numerical optimization for solving large-scale convex cone problems [[github](https://github.com/cvxgrp/scs) ![scs](https://img.shields.io/github/stars/cvxgrp/scs.svg?style=flat&label=Star&maxAge=86400)]
* sferes2 - Evolutionary computation [[github](https://github.com/sferes2/sferes2) ![sferes2/sferes2](https://img.shields.io/github/stars/sferes2/sferes2.svg?style=flat&label=Star&maxAge=86400)]

### [Robot Modeling](#awesome-robotics-libraries)

###### Robot Model Description Format
* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control ([bitbucket](https://bitbucket.org/osrf/sdformat))
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model [[github](https://github.com/ros/urdfdom) ![ros/urdfdom](https://img.shields.io/github/stars/ros/urdfdom.svg?style=flat&label=Star&maxAge=86400)]

###### Utility to Build Robot Models
* phobos - Add-on for Blender creating URDF and SMURF robot models [[github](https://github.com/rock-simulation/phobos) ![phobos](https://img.shields.io/github/stars/rock-simulation/phobos.svg?style=flat&label=Star&maxAge=86400)]

### [Robot Platform](#awesome-robotics-libraries)

* [Linorobot](https://linorobot.org/) - ROS compatible ground robots [[github](https://github.com/linorobot/linorobot) ![linorobot/linorobot](https://img.shields.io/github/stars/linorobot/linorobot.svg?style=flat&label=Star&maxAge=86400)]
  * onine - Service Robot based on [Linorobot](https://github.com/linorobot/linorobot) and Braccio Arm [[github](https://github.com/grassjelly/onine) ![grassjelly/onine](https://img.shields.io/github/stars/grassjelly/onine.svg?style=flat&label=Star&maxAge=86400)]
* [Rock](https://www.rock-robotics.org/stable/) - Software framework for robotic systems 
* [ROS](http://www.ros.org/) - Flexible framework for writing robot software [[github repos](http://wiki.ros.org/Tickets)]
* [ROS 2](https://github.com/ros2/ros2/wiki) - Version 2.0 of the Robot Operating System (ROS) software stack [[github repos](https://github.com/ros2)]
* [YARP](http://www.yarp.it/) - Communication and device interfaces applicable from humanoids to embedded devices [[github](https://github.com/robotology/yarp) ![robotology/yarp](https://img.shields.io/github/stars/robotology/yarp.svg?style=flat&label=Star&maxAge=86400)]
   
### [SLAM](#awesome-robotics-libraries)

* Cartographer - Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations [[github](https://github.com/googlecartographer/cartographer) ![cartographer](https://img.shields.io/github/stars/googlecartographer/cartographer.svg?style=flat&label=Star&maxAge=86400)]
* [DSO](https://vision.in.tum.de/research/vslam/dso) - Novel direct and sparse formulation for Visual Odometry [[github](https://github.com/JakobEngel/dso) ![dso](https://img.shields.io/github/stars/JakobEngel/dso.svg?style=flat&label=Star&maxAge=86400)]
* ElasticFusion - Real-time dense visual SLAM system [[github](http://github.com/mp3guy/ElasticFusion) ![ElasticFusion](https://img.shields.io/github/stars/mp3guy/ElasticFusion.svg?style=flat&label=Star&maxAge=86400)]
* Kintinuous - Real-time large scale dense visual SLAM system [[github](http://github.com/mp3guy/Kintinuous) ![Kintinuous](https://img.shields.io/github/stars/mp3guy/Kintinuous.svg?style=flat&label=Star&maxAge=86400)]
* [LSD-SLAM](https://vision.in.tum.de/research/vslam/lsdslam) - Real-time monocular SLAM [[github](http://github.com/tum-vision/lsd_slam) ![lsdslam](https://img.shields.io/github/stars/tum-vision/lsd_slam.svg?style=flat&label=Star&maxAge=86400)]
* ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras [[github](http://github.com/raulmur/ORB_SLAM2) ![ORB_SLAM2](https://img.shields.io/github/stars/raulmur/ORB_SLAM2.svg?style=flat&label=Star&maxAge=86400)]
* [SRBA](http://mrpt.github.io/srba/) - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies [[github](http://github.com/MRPT/srba) ![srba](https://img.shields.io/github/stars/MRPT/srba.svg?style=flat&label=Star&maxAge=86400)]

### [Vision](#awesome-robotics-libraries)

* [ViSP](http://visp.inria.fr/) - Visual Servoing Platform [[github](https://github.com/lagadic/visp) ![lagadic/visp](https://img.shields.io/github/stars/lagadic/visp.svg?style=flat&label=Star&maxAge=86400)]

### [Math](#awesome-robotics-libraries)

* Sophus - Lie groups using Eigen [[github](https://github.com/strasdat/Sophus) ![strasdat/Sophus](https://img.shields.io/github/stars/strasdat/Sophus.svg?style=flat&label=Star&maxAge=86400)]
* SpaceVelAlg - Spatial vector algebra with the Eigen3 [[github](https://github.com/jrl-umi3218/SpaceVecAlg) ![jrl-umi3218/SpaceVecAlg](https://img.shields.io/github/stars/jrl-umi3218/SpaceVecAlg.svg?style=flat&label=Star&maxAge=86400)]

## [Other Awesome Lists](#awesome-robotics-libraries)

* [Awesome Robotics](https://github.com/Kiloreux/awesome-robotics) (Kiloreux)
* [Awesome Robotics](https://github.com/ahundt/awesome-robotics) (ahundt)
* [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence)
* [Awesome Collision Detection](https://github.com/jslee02/awesome-collision-detection)
* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
* [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning)
* [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning)
* [Awesome Gazebo](https://github.com/fkromer/awesome-gazebo)
* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python sample codes for robotics algorithms

## [Contributing](#awesome-robotics-libraries)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#awesome-robotics-libraries)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
