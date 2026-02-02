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

_Simulation environments for testing and developing robotic systems._

###### Free or Open Source

* [AI2-THOR](https://ai2thor.allenai.org/) - Python framework with a Unity backend, providing interaction, navigation, and manipulation support for household based robotic agents. [[github](https://github.com/allenai/ai2thor) ⭐ 1.7k]
* AirSim - Simulator based on Unreal Engine for autonomous vehicles. [[github](https://github.com/Microsoft/AirSim) ⭐ 17.9k]
* [ARGoS](https://www.argos-sim.info/) - Physics-based simulator designed to simulate large-scale robot swarms. [[github](https://github.com/ilpincy/argos3) ⭐ 301]
* [ARTE](http://arvc.umh.es/arte/index_en.html) - Matlab toolbox focussed on robotic manipulators. [[github](https://github.com/4rtur1t0/ARTE) ⭐ 101]
* [AVIS Engine](https://avisengine.com) - Autonomous Vehicles Intelligent simulation software, A Fast and robust simulator software for Autonomous vehicle development. [[github](https://github.com/AvisEngine/AVIS-Engine-Python-API) ⭐ 21]
* [CARLA](https://carla.org/) - Open-source simulator for autonomous driving research. [[github](https://github.com/carla-simulator/carla) ⭐ 13.5k]
* [CoppeliaSim](https://www.coppeliarobotics.com/) - Formaly V-REP. Virtual robot experimentation platform. [[github](https://github.com/CoppeliaRobotics/CoppeliaSimLib) ⭐ 138]
* [Gazebo](https://gazebosim.org/) - Dynamic multi-robot simulator. [[github](https://github.com/gazebosim/gazebo-classic) ⭐ 1.3k]
* [Gazebo Sim](https://gazebosim.org/) - Open source robotics simulator (formerly Ignition Gazebo). [[github](https://github.com/gazebosim/gz-sim) ⭐ 1.2k]
* [GraspIt!](http://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs. [[github](https://github.com/graspit-simulator/graspit) ⭐ 207]
* [Habitat-Sim](https://aihabitat.org/) - Simulation platform for research in embodied artificial intelligence. [[github](https://github.com/facebookresearch/habitat-sim) ⭐ 3.5k]
* [Hexapod Robot Simulator](https://hexapod.netlify.app/) - Open-source hexapod robot inverse kinematics and gaits visualizer. [[github](https://github.com/mithi/hexapod) ⭐ 681]
* [Isaac Sim](https://developer.nvidia.com/isaac/sim) - Nvidia's robotic simulation environment with GPU physics simulation and ray tracing.
* [ManiSkill](https://github.com/haosulab/ManiSkill) - A robot simulation and behavior learning package powered by SAPIEN, with a strong focus on manipulation skills. [[github](https://github.com/haosulab/ManiSkill) ⭐ 2.5k]
* [MORSE](http://morse-simulator.github.io/) - Modular open robots simulation engine. [[github](https://github.com/morse-simulator/morse) ⭐ 370]
* [Neurorobotics Platform](https://neurorobotics.net/) - Internet-accessible simulation of robots controlled by spiking neural networks. [[bitbucket](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)]
* [PyBullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) - An easy to use simulator for robotics and deep reinforcement learning. [[github](https://github.com/bulletphysics/bullet3) ⭐ 14.2k]
* [PyBullet_Industrial](https://pybullet-industrial.readthedocs.io/en/latest/) - A extension to PyBullet that allows for the simulation of various robotic manufacturing processes such as milling or 3D-printing. [[github](https://github.com/WBK-Robotics/pybullet_industrial) ⭐ 48]
* [Robot Gui](http://robot.glumb.de/) - A three.js based 3D robot interface. [[github](https://github.com/glumb/robot-gui) ⭐ 384]
* [SAPIEN](https://sapien.ucsd.edu) - A realistic and physics-rich simulated environment that hosts a large-scale set for articulated objects. [[github](https://github.com/haosulab/SAPIEN) ⭐ 714]
* [Simbad](http://simbad.sourceforge.net/) - A Java 3D robot simulator, enables to write own robot controller with modifying environment using available sensors.
* [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics) - Popular game engine that now offers open-source tools, tutorials, and resources for robotics simulation. [[github](https://github.com/Unity-Technologies/Unity-Robotics-Hub) ⭐ 2.5k]
* [Webots](http://www.cyberbotics.com/) - A complete development environment to model, program and simulate robots, vehicles and mechanical systems. [[github](https://github.com/cyberbotics/webots) ⭐ 4.1k]

###### Commercial

* [Actin Simulation](http://www.energid.com/) - Real-time robot simulation and control software.
* [Artiminds](https://www.artiminds.com/) - Planning, programming, operation, analysis and optimization.
* [Kineo](https://www.plm.automation.siemens.com/global/en/products/plm-components/kineo.html) - Path planning and trajectory optimization for industrial robotics and digital mock-up review applications.
* [Robot Virtual Worlds](http://www.robotvirtualworlds.com/) - Virtual reality software for educational robotics.
* [RobotDK](https://robodk.com/) - Simulation and OLP for robots.
* [RobotStudio](https://www.abb.com/global/en/areas/robotics/products/software/robotstudio-suite) - ABB's simulation and offline programming software for robotics.
* [Virtual Robotics Toolkit](https://www.virtualroboticstoolkit.com/) - 3D virtual environment for programming and testing robots.
* [Visual Components](https://www.visualcomponents.com/) - 3D manufacturing simulation and visualization platform.

###### Cloud

* [AWS RoboMaker](https://aws.amazon.com/robomaker/) - Service that makes it easy to develop, test, and deploy intelligent robotics applications at scale.

## [Libraries](#contents)

### [Dynamics Simulation](#contents)

_Physics engines and rigid/soft body dynamics libraries. See also [Comparisons](COMPARISONS.md)._

* [ARCSim](http://graphics.berkeley.edu/resources/ARCSim/index.html) - Adaptive remeshing cloth and shell simulator for thin deformable objects.
* [Bullet](https://pybullet.org/) - Real-time physics simulation for games, visual effects, and robotics. [[github](https://github.com/bulletphysics/bullet3) ⭐ 14.2k]
* [CHRONO::ENGINE](https://projectchrono.org/) - Multi-physics simulation of rigid and flexible bodies, granular, and fluid systems. [[github](https://github.com/projectchrono/chrono) ⭐ 2.7k]
* [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit for multibody simulation and planning. [[github](https://github.com/dartsim/dart) ⭐ 1.1k]
* [Drake](https://drake.mit.edu/) - Planning, control, and analysis toolbox for nonlinear dynamical systems. [[github](https://github.com/RobotLocomotion/drake) ⭐ 3.9k]
* [Flex](https://developer.nvidia.com/flex) - GPU-based particle simulation for rigid bodies, fluids, and deformables. [[github](https://github.com/NVIDIAGameWorks/FleX) ⭐ 787]
* [FROST](https://ayonga.github.io/frost-dev/index.html) - Fast Robot Optimization and Simulation Toolkit for hybrid dynamical systems in MATLAB. [[github](https://github.com/ayonga/frost-dev) ⭐ 169]
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - Impulse-based dynamics simulation for rigid bodies and particle systems.
* idyntree - Library for estimation and whole-body dynamics of floating-base robots. [[github](https://github.com/gbionics/idyntree) ⭐ 223]
* [KDL](https://www.orocos.org/kdl.html) - Orocos Kinematics and Dynamics Library for kinematic chains. [[github](https://github.com/orocos/orocos_kinematics_dynamics) ⭐ 857]
* kindr - Kinematics and dynamics library for rigid body transformations. [[github](https://github.com/ANYbotics/kindr) ⭐ 607]
* [Klampt](https://klampt.org/) - Robot planning, control, and simulation with visualization support. [[github](https://github.com/krishauser/Klampt) ⭐ 427]
* [LibrePilot](http://www.librepilot.org/site/index.html) - Open-source autopilot for UAVs and other autonomous vehicles. [[github](https://github.com/librepilot/LibrePilot) ⭐ 348]
* [MARS](http://rock-simulation.github.io/mars/) - Machina Arte Robotum Simulans — a cross-platform simulation environment. [[github](https://github.com/rock-simulation/mars) ⭐ 67]
* [MBDyn](https://www.mbdyn.org/) - General-purpose multibody dynamics analysis software.
* [MBSim](https://www.mbsim-env.de/) - Multi-body simulation environment for flexible and rigid systems. [[github](https://github.com/mbsim-env/mbsim) ⭐ 51]
* [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) - Lightweight multibody system dynamics library. [[github](https://github.com/SIM-TU-Darmstadt/mbslib) ⭐ 11]
* metapod - Template-based robot dynamics library using spatial algebra. [[github](https://github.com/laas/metapod) ⭐ 14]
* [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation for rigid bodies with contact. [[github](https://github.com/PositronicsLab/Moby) ⭐ 37]
* [mrpt](https://www.mrpt.org/) - Mobile Robot Programming Toolkit for SLAM, navigation, and computer vision. [[github](https://github.com/MRPT/mrpt) ⭐ 2.1k]
* [MuJoCo](https://mujoco.org/) - Multi-joint dynamics with contact for physics-based simulation and control. [[github](https://github.com/google-deepmind/mujoco) ⭐ 11.9k]
* [mvsim](http://wiki.ros.org/mvsim) - Lightweight multi-vehicle 2D simulator with ROS integration. [[github](https://github.com/MRPT/mvsim) ⭐ 359]
* [Newton Dynamics](https://newtondynamics.com/) - Real-time physics engine for rigid body simulation. [[github](https://github.com/MADEAPPS/newton-dynamics) ⭐ 1k]
* [nphysics](https://nphysics.org/) - 2D and 3D rigid body physics engine written in Rust. [[github](https://github.com/dimforge/nphysics) ⭐ 1.6k]
* [ODE](https://ode.org/) - Open Dynamics Engine for simulating rigid body dynamics. [[bitbucket](https://bitbucket.org/odedevs/ode)]
* [OpenRAVE](https://www.openrave.org/) - Open Robotics Automation Virtual Environment for planning and simulation. [[github](https://github.com/rdiankov/openrave) ⭐ 798]
* [PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/index.html) - NVIDIA physics engine for real-time rigid body and vehicle simulation. [[github](https://github.com/NVIDIA-Omniverse/PhysX) ⭐ 4.4k]
* [pinocchio](https://stack-of-tasks.github.io/pinocchio/) - Fast and flexible algorithms for rigid-body dynamics with analytical derivatives. [[github](https://github.com/stack-of-tasks/pinocchio) ⭐ 3.1k]
* PositionBasedDynamics - Position-based methods for simulating deformable objects and fluids. [[github](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics) ⭐ 2.2k]
* [PyDy](https://www.pydy.org/) - Multibody dynamics analysis with symbolic Python using SymPy. [[github](https://github.com/pydy/pydy) ⭐ 405]
* qu3e - Lightweight 3D physics engine for rigid body dynamics. [[github](https://github.com/RandyGaul/qu3e) ⭐ 974]
* [RaiSim](https://slides.com/jeminhwangbo/raisim-manual) - Cross-platform physics engine for robotics and reinforcement learning. [[github](https://github.com/leggedrobotics/raisimLib) ⭐ 329]
* [RBDL](https://rbdl.github.io/) - Rigid Body Dynamics Library based on Featherstone algorithms. [[github](https://github.com/rbdl/rbdl) ⭐ 683]
* RBDyn - Rigid body dynamics algorithms using spatial algebra with Eigen. [[github](https://github.com/jrl-umi3218/RBDyn) ⭐ 211]
* [ReactPhysics3d](https://www.reactphysics3d.com/) - Open-source 3D physics engine for rigid body simulation and collision detection. [[github](https://github.com/DanielChappuis/reactphysics3d) ⭐ 1.7k]
* RigidBodyDynamics.jl - Julia library for rigid body dynamics algorithms. [[github](https://github.com/JuliaRobotics/RigidBodyDynamics.jl) ⭐ 307]
* [Rigs of Rods](https://www.rigsofrods.org/) - Soft-body vehicle simulator using beam physics. [[github](https://github.com/RigsOfRods/rigs-of-rods) ⭐ 1.1k]
* [Robopy](https://adityadua24.github.io/robopy/) - Python robotics toolbox inspired by Peter Corke's Robotics Toolbox. [[github](https://github.com/adityadua24/robopy) ⭐ 228]
* [Robotics Library](https://www.roboticslibrary.org/) - Self-contained C++ library for robot kinematics, planning, and control. [[github](https://github.com/roboticslibrary/rl) ⭐ 1.2k]
* [RobWork](https://robwork.dk/) - Framework for simulation and control of robot systems. [[gitlab](https://gitlab.com/sdurobotics/RobWork)]
* [siconos](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/) - Nonsmooth dynamical systems modeling and simulation platform. [[github](https://github.com/siconos/siconos) ⭐ 182]
* [Simbody](https://simtk.org/home/simbody/) - Multibody dynamics library for biomechanical and mechanical systems. [[github](https://github.com/simbody/simbody) ⭐ 2.5k]
* [SOFA](https://www.sofa-framework.org/) - Simulation Open Framework Architecture for medical and physics simulation. [[github](https://github.com/sofa-framework/sofa) ⭐ 1.1k]
* Tiny Differentiable Simulator - Header-only differentiable physics engine for robotics. [[github](https://github.com/erwincoumans/tiny-differentiable-simulator) ⭐ 1.3k]
* [trep](http://murpheylab.github.io/trep/) - Simulation and optimal control using variational integrators. [[github](https://github.com/MurpheyLab/trep) ⭐ 20]

### [Inverse Kinematics](#contents)

_Libraries for computing joint configurations from end-effector poses._

  * IKBT - A python package to solve robot arm inverse kinematics in symbolic form. [[github](https://github.com/uw-biorobotics/IKBT) ⭐ 215]
  * Kinpy - A simple pure python package to solve inverse kinematics. [[github](https://github.com/neka-nat/kinpy) ⭐ 179]
  * Lively - A highly configurable toolkit for commanding robots in mixed modalities. [[github](https://github.com/Wisc-HCI/lively) ⭐ 7]
  * RelaxedIK - Real-time Synthesis of Accurate and Feasible Robot Arm Motion. [[github](https://github.com/uwgraphics/relaxed_ik) ⭐ 235]
  * [Trip](https://trip-kinematics.readthedocs.io/en/main/index.html) - A python package that solves inverse kinematics of parallel-, serial- or hybrid-robots. [[github](https://github.com/TriPed-Robot/trip_kinematics) ⭐ 44]

### [Machine Learning](#contents)

_Machine learning frameworks and tools applied to robotics._

* [AllenAct](https://allenact.org/) - Python/PyTorch-based Research Framework for Embodied AI. [[github](https://github.com/allenai/allenact) ⭐ 376]
* Any4LeRobot - A collection of utilities and tools for LeRobot. [[github](https://github.com/Tavish9/any4lerobot) ⭐ 838]
* DLL - Deep Learning Library (DLL) for C++. [[github](https://github.com/wichtounet/dll) ⭐ 687]
* [DyNet](https://dynet.readthedocs.io/en/latest/) - The Dynamic Neural Network Toolkit. [[github](https://github.com/clab/dynet) ⭐ 3.4k]
* [Fido](http://fidoproject.github.io/) - Lightweight C++ machine learning library for embedded electronics and robotics. [[github](https://github.com/FidoProject/Fido) ⭐ 462]
* [Gymnasium](https://gymnasium.farama.org/) - Developing and comparing reinforcement learning algorithms. [[github](https://github.com/Farama-Foundation/Gymnasium) ⭐ 11.2k]
  * gym-dart - OpenAI Gym environments using the DART physics engine. [[github](https://github.com/DartEnv/dart-env) ⭐ 141]
  * gym-gazebo - OpenAI Gym environments for the Gazebo simulator. [[github](https://github.com/erlerobot/gym-gazebo) ⭐ 845]
* [Ivy](https://lets-unify.ai/) - Unified Machine Learning Framework. [[github](https://github.com/ivy-llc/ivy) ⭐ 14.2k]
* LeRobot - State-of-the-art approaches, pretrained models, datasets, and simulation environments for real-world robotics in PyTorch. [[github](https://github.com/huggingface/lerobot) ⭐ 21.4k]
* [LeRobot Episode Scoring Toolkit](https://github.com/RoboticsData/score_lerobot_episodes) - One-click tool to score, filter, and export higher-quality LeRobot datasets. [[github](https://github.com/RoboticsData/score_lerobot_episodes) ⭐ 48]
* MiniDNN - A header-only C++ library for deep neural networks. [[github](https://github.com/yixuan/MiniDNN) ⭐ 431]
* [mlpack](https://www.mlpack.org/) - Scalable C++ machine learning library. [[github](https://github.com/mlpack/mlpack) ⭐ 5.6k]
* RLLib - Temporal-difference learning algorithms in reinforcement learning. [[github](https://github.com/samindaa/RLLib) ⭐ 208]
* [robosuite](https://robosuite.ai) - A modular simulation framework and benchmark for robot learning. [[github](https://github.com/ARISE-Initiative/robosuite) ⭐ 2.2k]
* [tiny-dnn](http://tiny-dnn.readthedocs.io/en/latest/) - Header only, dependency-free deep learning framework in C++14. [[github](https://github.com/tiny-dnn/tiny-dnn) ⭐ 6k]

### [Motion Planning and Control](#contents)

_Libraries for robot motion planning, trajectory optimization, and control._


* [AIKIDO](https://github.com/personalrobotics/aikido) - Solving robotic motion planning and decision making problems. [[github](https://github.com/personalrobotics/aikido) ⭐ 228]
* Bioptim - Bioptim, a Python Framework for Musculoskeletal Optimal Control in Biomechanics. [[github](https://github.com/pyomeca/bioptim) ⭐ 113]
* [Control Toolbox](https://ethz-adrl.github.io/ct/) - Open-Source C++ Library for Robotics, Optimal and Model Predictive Control. [[github](https://github.com/ethz-adrl/control-toolbox) ⭐ 1.7k]
* Crocoddyl - Optimal control library for robot control under contact sequence. [[github](https://github.com/loco-3d/crocoddyl) ⭐ 1.2k]
* [CuiKSuite](http://www.iri.upc.edu/people/porta/Soft/CuikSuite2-Doc/html) - Applications to solve position analysis and path planning problems.
* [cuRobo](https://curobo.org) - A CUDA accelerated library containing a suite of robotics algorithms that run significantly faster. [[github](https://github.com/nvlabs/curobo) ⭐ 1.3k]
* Fields2Cover - Robust and efficient coverage paths for autonomous agricultural vehicles. [[github](https://github.com/fields2cover/fields2cover) ⭐ 745]
* GPMP2 - Gaussian Process Motion Planner 2. [[github](https://github.com/gtrll/gpmp2) ⭐ 351]
* [HPP](https://humanoid-path-planner.github.io/hpp-doc/) - Path planning for kinematic chains in environments cluttered with obstacles.
* [MoveIt!](https://moveit.ai/) - Motion planning framework. [[github](https://github.com/moveit/moveit) ⭐ 2k]
* OCS2 - Efficient continuous and discrete time optimal control implementation. [[github](https://github.com/leggedrobotics/ocs2) ⭐ 1.3k]
* [OMPL](https://ompl.kavrakilab.org/) - Open motion planning library. [[github](https://github.com/ompl/ompl) ⭐ 1.9k]
* pymanoid - Humanoid robotics prototyping environment based on OpenRAVE. [[github](https://github.com/stephane-caron/pymanoid) ⭐ 232]
* ROS Behavior Tree - Behavior tree implementation for ROS-based robot task planning. [[github](https://github.com/miccol/ROS-Behavior-Tree) ⭐ 362]
* [Ruckig](https://github.com/pantor/ruckig) - Real-time, time-optimal and jerk-constrained online trajectory generation. [[github](https://github.com/pantor/ruckig) ⭐ 1.1k]
* [The Kautham Project](https://sir.upc.es/projects/kautham/) - A robot simulation toolkit for motion planning. [[github](https://github.com/iocroblab/kautham) ⭐ 24]
* [TOPP-RA](https://hungpham2511.github.io/toppra/) - Time-parameterizing robot trajectories subject to kinematic and dynamic constraints. [[github](https://github.com/hungpham2511/toppra) ⭐ 830]
* [Ungar](https://github.com/fdevinc/ungar) - Expressive and efficient implementation of optimal control problems using template metaprogramming. [[github](https://github.com/fdevinc/ungar) ⭐ 109]

###### Motion Optimizer

* TopiCo - Time-optimal Trajectory Generation and Control. [[github](https://github.com/AIS-Bonn/TopiCo) ⭐ 143]
* [towr](http://wiki.ros.org/towr) - A light-weight, Eigen-based C++ library for trajectory optimization for legged robots. [[github](https://github.com/ethz-adrl/towr) ⭐ 1k]
* TrajectoryOptimization - A fast trajectory optimization library written in Julia. [[github](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl) ⭐ 386]
* [trajopt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/) - Framework for generating robot trajectories by local optimization. [[github](https://github.com/joschu/trajopt) ⭐ 449]

###### Nearest Neighbor

* [Cover-Tree](http://hunch.net/~jl/projects/cover_tree/cover_tree.html) - Cover tree data structure for quick k-nearest-neighbor search. [[github](https://github.com/DNCrane/Cover-Tree) ⭐ 64]
  * [Faster cover trees](http://proceedings.mlr.press/v37/izbicki15.pdf) - by Mike Izbicki et al., ICML 2015.
* [FLANN](http://www.cs.ubc.ca/research/flann/) - Fast Library for Approximate Nearest Neighbors. [[github](https://github.com/flann-lib/flann) ⭐ 2.4k]
* [nanoflann](http://www.cs.ubc.ca/research/flann/) - Nearest Neighbor search with KD-trees. [[github](https://github.com/jlblancoc/nanoflann) ⭐ 2.6k]

###### 3D Mapping

* Bonxai - Brutally fast, sparse, 3D Voxel Grid (formerly Treexy). [[github](https://github.com/facontidavide/Bonxai) ⭐ 810]
* [Goxel](https://guillaumechereau.github.io/goxel/) - Free and open source 3D voxel editor. [[github](https://github.com/guillaumechereau/goxel) ⭐ 3.1k]
* [libpointmatcher](http://libpointmatcher.readthedocs.io/en/latest/) - Iterative Closest Point library for 2-D/3-D mapping in Robotics. [[github](https://github.com/norlab-ulaval/libpointmatcher) ⭐ 1.8k]
* [OctoMap](http://octomap.github.io/) - Efficient Probabilistic 3D Mapping Framework Based on Octrees. [[github](https://github.com/OctoMap/octomap) ⭐ 2.3k]
* Octree - Fast radius neighbor search with an Octree. [[github](https://github.com/jbehley/octree) ⭐ 374]
* [PCL](https://pointclouds.org/) - 2D/3D image and point cloud processing. [[github](https://github.com/PointCloudLibrary/pcl) ⭐ 10.8k]
* Utility Software
* voxblox - Flexible voxel-based mapping focusing on truncated and Euclidean signed distance fields. [[github](https://github.com/ethz-asl/voxblox) ⭐ 1.6k]
* [wavemap](https://projects.asl.ethz.ch/wavemap/) - Fast, efficient and accurate multi-resolution, multi-sensor 3D occupancy mapping. [[github](https://github.com/ethz-asl/wavemap) ⭐ 551]

### [Optimization](#contents)

_Numerical optimization solvers and frameworks used in robotics._

* [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic framework for algorithmic differentiation and numeric optimization. [[github](https://github.com/casadi/casadi) ⭐ 2.1k]
* [Ceres Solver](http://ceres-solver.org/) - Large scale nonlinear optimization library. [[github](https://github.com/ceres-solver/ceres-solver) ⭐ 4.4k]
* eigen-qld - Interface to use the QLD QP solver with the Eigen3 library. [[github](https://github.com/jrl-umi3218/eigen-qld) ⭐ 16]
* EXOTica - Generic optimisation toolset for robotics platforms. [[github](https://github.com/ipab-slmc/exotica) ⭐ 161]
* hpipm - High-performance interior-point-method QP solvers (Ipopt, Snopt). [[github](https://github.com/giaf/hpipm) ⭐ 665]
* [HYPRE](https://hypre.readthedocs.io/) - Parallel solvers for sparse linear systems featuring multigrid methods. [[github](https://github.com/hypre-space/hypre) ⭐ 812]
* ifopt - An Eigen-based, light-weight C++ Interface to Nonlinear Programming Solvers (Ipopt, Snopt). [[github](https://github.com/ethz-adrl/ifopt) ⭐ 847]
* [Ipopt](https://projects.coin-or.org/Ipopt) - Large scale nonlinear optimization library. [[github](https://github.com/coin-or/Ipopt) ⭐ 1.7k]
* libcmaes - Blackbox stochastic optimization using the CMA-ES algorithm. [[github](https://github.com/CMA-ES/libcmaes) ⭐ 354]
* [limbo](http://www.resibots.eu/limbo/) - Gaussian processes and Bayesian optimization of black-box functions. [[github](https://github.com/resibots/limbo) ⭐ 260]
* lpsolvers - Linear Programming solvers in Python with a unified API. [[github](https://github.com/stephane-caron/lpsolvers) ⭐ 25]
* [NLopt](https://nlopt.readthedocs.io/en/latest/) - Nonlinear optimization. [[github](https://github.com/stevengj/nlopt) ⭐ 2.2k]
* [OptimLib](https://www.kthohr.com/optimlib.html) - Lightweight C++ library of numerical optimization methods for nonlinear functions. [[github](https://github.com/kthohr/optim) ⭐ 885]
* [OSQP](https://osqp.org/) - The Operator Splitting QP Solver. [[github](https://github.com/osqp/osqp) ⭐ 2.1k]
* [Pagmo](https://esa.github.io/pagmo2/index.html) - Scientific library for massively parallel optimization. [[github](https://github.com/esa/pagmo2) ⭐ 907]
* [ProxSuite](https://simple-robotics.github.io/proxsuite/) - The Advanced Proximal Optimization Toolbox. [[github](https://github.com/Simple-Robotics/ProxSuite) ⭐ 538]
* [pymoo](https://www.pymoo.org/) - Multi-objective Optimization in Python. [[github](https://github.com/msu-coinlab/pymoo) ⭐ 26]
* qpsolvers - Quadratic Programming solvers in Python with a unified API. [[github](https://github.com/qpsolvers/qpsolvers) ⭐ 725]
* [RobOptim](http://roboptim.net/index.html) - Numerical Optimization for Robotics. [[github](https://github.com/roboptim/roboptim-core) ⭐ 65]
* [SCS](http://web.stanford.edu/~boyd/papers/scs.html) - Numerical optimization for solving large-scale convex cone problems. [[github](https://github.com/cvxgrp/scs) ⭐ 613]
* sferes2 - Evolutionary computation. [[github](https://github.com/sferes2/sferes2) ⭐ 170]
* SHOT - A solver for mixed-integer nonlinear optimization problems. [[github](https://github.com/coin-or/SHOT) ⭐ 129]

### [Robot Modeling](#contents)

_Tools and formats for describing robot models._

###### Robot Model Description Format

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. [[bitbucket](https://bitbucket.org/osrf/sdformat)]
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. [[github](https://github.com/ros/urdfdom) ⭐ 118]

###### Utility to Build Robot Models

* [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API. [[github](https://github.com/Rhoban/onshape-to-robot) ⭐ 486]
* phobos - Add-on for Blender creating URDF and SMURF robot models. [[github](https://github.com/dfki-ric/phobos) ⭐ 855]

### [Robot Platform](#contents)

_Middleware and frameworks for building robot software systems._

* [AutoRally](http://autorally.github.io/) - High-performance testbed for advanced perception and control research. [[github](https://github.com/autorally/autorally) ⭐ 775]
* [Linorobot](https://linorobot.org/) - ROS compatible ground robots. [[github](https://github.com/linorobot/linorobot) ⭐ 1.1k]
  * onine - Service Robot based on Linorobot and Braccio Arm. [[github](https://github.com/grassjelly/onine) ⭐ 47]
* [Micro-ROS for Arduino](https://github.com/kaiaai/micro_ros_arduino_kaiaai) - a Micro-ROS fork available in the Arduino Library Manager. [[github](https://github.com/kaiaai/micro_ros_arduino_kaiaai) ⭐ 12]
* [Rock](https://www.rock-robotics.org/) - Software framework for robotic systems.
* [ROS](https://www.ros.org/) - Flexible framework for writing robot software.
* [ROS 2](https://github.com/ros2/ros2/wiki) - Version 2.0 of the Robot Operating System (ROS) software stack. [[github](https://github.com/ros2/ros2) ⭐ 5k]
* [YARP](https://www.yarp.it/) - Communication and device interfaces applicable from humanoids to embedded devices. [[github](https://github.com/robotology/yarp) ⭐ 585]

### [Reinforcement Learning for Robotics](#contents)

_Reinforcement learning libraries commonly used in robotic control._

* [CleanRL](https://github.com/vwxyzjn/cleanrl) - Single-file implementations of deep reinforcement learning algorithms. [[github](https://github.com/vwxyzjn/cleanrl) ⭐ 9k]
* [rl_games](https://github.com/Denys88/rl_games) - High-performance RL library used in Isaac Gym environments. [[github](https://github.com/Denys88/rl_games) ⭐ 1.3k]
* [SKRL](https://github.com/Toni-SM/skrl) - Modular reinforcement learning library with support for multiple ML frameworks. [[github](https://github.com/Toni-SM/skrl) ⭐ 982]
* [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) - Reliable implementations of reinforcement learning algorithms in PyTorch. [[github](https://github.com/DLR-RM/stable-baselines3) ⭐ 12.7k]

### [SLAM](#contents)

_Simultaneous Localization and Mapping libraries._


* AprilSAM - Real-time smoothing and mapping. [[github](https://github.com/xipengwang/AprilSAM) ⭐ 239]
* Cartographer - Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations. [[github](https://github.com/cartographer-project/cartographer) ⭐ 7.8k]
* [DSO](https://vision.in.tum.de/research/vslam/dso) - Novel direct and sparse formulation for Visual Odometry. [[github](https://github.com/JakobEngel/dso) ⭐ 2.4k]
* ElasticFusion - Real-time dense visual SLAM system. [[github](https://github.com/mp3guy/ElasticFusion) ⭐ 1.9k]
* [fiducials](http://wiki.ros.org/fiducials) - Simultaneous localization and mapping using fiducial markers. [[github](https://github.com/UbiquityRobotics/fiducials) ⭐ 278]
* GTSAM - Smoothing and mapping (SAM) in robotics and vision. [[github](https://github.com/borglab/gtsam) ⭐ 3.3k]
* Kintinuous - Real-time large scale dense visual SLAM system. [[github](https://github.com/mp3guy/Kintinuous) ⭐ 951]
* [LSD-SLAM](https://vision.in.tum.de/research/vslam/lsdslam) - Real-time monocular SLAM. [[github](https://github.com/tum-vision/lsd_slam) ⭐ 2.7k]
* ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras. [[github](https://github.com/raulmur/ORB_SLAM2) ⭐ 10.1k]
* [RTAP-Map](http://introlab.github.io/rtabmap/) - RGB-D Graph SLAM approach based on a global Bayesian loop closure detector. [[github](https://github.com/introlab/rtabmap) ⭐ 3.6k]
* [SRBA](http://mrpt.github.io/srba/) - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies. [[github](https://github.com/MRPT/srba) ⭐ 76]

#### SLAM Dataset

* [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets) - Curated list of SLAM-related datasets. [[github](https://github.com/youngguncho/awesome-slam-datasets) ⭐ 1.9k]

### [Vision](#contents)

_Computer vision libraries for robotic perception._

* [BundleTrack](https://github.com/wenbowen123/BundleTrack) - 6D Pose Tracking for Novel Objects without 3D Models. [[github](https://github.com/wenbowen123/BundleTrack) ⭐ 678]
* [se(3)-TrackNet](https://github.com/wenbowen123/iros20-6d-pose-tracking) - 6D Pose Tracking for Novel Objects without 3D Models. [[github](https://github.com/wenbowen123/iros20-6d-pose-tracking) ⭐ 420]
* [ViSP](http://visp.inria.fr/) - Visual Servoing Platform. [[github](https://github.com/lagadic/visp) ⭐ 849]

### [Fluid](#contents)

_Fluid dynamics simulation libraries._

* [Fluid Engine Dev - Jet](https://fluidenginedevelopment.org/) - Fluid simulation engine for computer graphics applications. [[github](https://github.com/doyubkim/fluid-engine-dev) ⭐ 2.1k]

### [Grasping](#contents)

_Libraries and tools for robotic grasping and manipulation._

* [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) - SDK for AnyGrasp, a 6-DoF grasp pose detection method. [[github](https://github.com/graspnet/anygrasp_sdk) ⭐ 752]
* [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) - 6-DoF grasp generation for parallel-jaw grippers using contact maps. [[github](https://github.com/NVlabs/contact_graspnet) ⭐ 449]
* [GraspIt!](https://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs. [[github](https://github.com/graspit-simulator/graspit) ⭐ 207]
* [GraspNet API](https://github.com/graspnet/graspnetAPI) - Python API and evaluation tools for the GraspNet benchmark. [[github](https://github.com/graspnet/graspnetAPI) ⭐ 323]

### [Humanoid Robotics](#contents)

_Environments and models for humanoid robot research._

* [Humanoid-Gym](https://github.com/roboterax/humanoid-gym) - Reinforcement learning environment for humanoid robot locomotion. [[github](https://github.com/roboterax/humanoid-gym) ⭐ 1.8k]
* [Legged Gym](https://github.com/leggedrobotics/legged_gym) - Isaac Gym environments for legged robot locomotion training. [[github](https://github.com/leggedrobotics/legged_gym) ⭐ 2.7k]
* [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - Collection of well-tuned MuJoCo models for research and development. [[github](https://github.com/google-deepmind/mujoco_menagerie) ⭐ 3k]

### [Multiphysics](#contents)

_Frameworks for coupled multi-physics simulations._

* [Kratos](http://www.cimne.com/kratos/) - Framework for building parallel multi-disciplinary simulation software. [[github](https://github.com/KratosMultiphysics/Kratos) ⭐ 1.2k]

### [Math](#contents)

_Mathematics libraries for spatial algebra, Lie groups, and linear algebra._

* Fastor - Light-weight high performance tensor algebra framework in C++11/14/17. [[github](https://github.com/romeric/Fastor) ⭐ 830]
* linalg.h - Single header public domain linear algebra library for C++11. [[github](https://github.com/sgorsten/linalg) ⭐ 939]
* manif - Small c++11 header-only library for Lie theory. [[github](https://github.com/artivis/manif) ⭐ 1.7k]
* Sophus - Lie groups using Eigen. [[github](https://github.com/strasdat/Sophus) ⭐ 2.4k]
* SpaceVelAlg - Spatial vector algebra with the Eigen3. [[github](https://github.com/jrl-umi3218/SpaceVecAlg) ⭐ 80]
* spatialmath-python - A python package provides classes to represent pose and orientation in 3D and 2D space, along with a toolbox of spatial operations. [[github](https://github.com/bdaiinstitute/spatialmath-python) ⭐ 615]

### [ETC](#contents)

_Other robotics-related tools and utilities._

* [Foxglove Studio](https://foxglove.dev) - A fully integrated visualization and debugging desktop app for your robotics data.
* fuse - General architecture for performing sensor fusion live on a robot. [[github](https://github.com/locusrobotics/fuse) ⭐ 847]

## [Other Awesome Lists](#contents)

_Related curated lists of robotics and AI resources._

* [Awesome Robotics](https://github.com/Kiloreux/awesome-robotics) - (Kiloreux).
* [Awesome Robotics](https://github.com/ahundt/awesome-robotics) - (ahundt).
* [Awesome Robotic Tooling](https://github.com/Ly0n/awesome-robotic-tooling)
* [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence)
* [Awesome Collision Detection](https://github.com/jslee02/awesome-collision-detection)
* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
* [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning)
* [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning)
* [Awesome Gazebo](https://github.com/fkromer/awesome-gazebo)
* [Awesome Grasping](https://github.com/Po-Jen/awesome-grasping)
* [Awesome Human Robot Interaction](https://github.com/Po-Jen/awesome-human-robot-interaction)
* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python sample codes for robotics algorithms.
* [Robotics Coursework](https://github.com/mithi/robotics-coursework) - A list of robotics courses you can take online.

## [Contributing](#contents)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#contents)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
