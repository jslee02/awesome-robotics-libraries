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

> **Legend**: 🟢 Active (<1yr) · 🟡 Slow (1-2yr) · 🔴 Stale (>2yr) · 💀 Archived

## [Simulators](#contents)

_Simulation environments for testing and developing robotic systems._

###### Free or Open Source

* 🟢 [AI2-THOR](https://ai2thor.allenai.org/) - Interactive household environment for embodied AI with Unity backend. [⭐ 1.7k](https://github.com/allenai/ai2thor)
* 🟢 AirSim - Simulator based on Unreal Engine for autonomous vehicles. [⭐ 17.9k](https://github.com/Microsoft/AirSim)
* 🟢 [ARGoS](https://www.argos-sim.info/) - Physics-based simulator designed to simulate large-scale robot swarms. [⭐ 301](https://github.com/ilpincy/argos3)
* 🟢 [ARTE](http://arvc.umh.es/arte/index_en.html) - Matlab toolbox focussed on robotic manipulators. [⭐ 101](https://github.com/4rtur1t0/ARTE)
* 🟢 [AVIS Engine](https://avisengine.com) - Fast simulation software for autonomous vehicle development. [⭐ 21](https://github.com/AvisEngine/AVIS-Engine-Python-API)
* 🟢 [CARLA](https://carla.org/) - Open-source simulator for autonomous driving research. [⭐ 13.5k](https://github.com/carla-simulator/carla)
* 🟢 [CoppeliaSim](https://www.coppeliarobotics.com/) - Formaly V-REP. Virtual robot experimentation platform. [⭐ 138](https://github.com/CoppeliaRobotics/CoppeliaSimLib)
* 💀 [Gazebo](https://gazebosim.org/) - Dynamic multi-robot simulator. [⭐ 1.3k](https://github.com/gazebosim/gazebo-classic)
* 🟢 [Gazebo Sim](https://gazebosim.org/) - Open source robotics simulator (formerly Ignition Gazebo). [⭐ 1.2k](https://github.com/gazebosim/gz-sim)
* 🔴 [GraspIt!](http://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs. [⭐ 207](https://github.com/graspit-simulator/graspit)
* 🟢 [Habitat-Sim](https://aihabitat.org/) - Simulation platform for research in embodied artificial intelligence. [⭐ 3.5k](https://github.com/facebookresearch/habitat-sim)
* 🟢 [Hexapod Robot Simulator](https://hexapod.netlify.app/) - Open-source hexapod robot inverse kinematics and gaits visualizer. [⭐ 681](https://github.com/mithi/hexapod)
* 🟢 [Isaac Sim](https://developer.nvidia.com/isaac/sim) - NVIDIA's GPU-accelerated robotics simulation platform with PhysX 5 and RTX rendering. [⭐ 2.5k](https://github.com/isaac-sim/IsaacSim)
* 🟢 [ManiSkill](https://github.com/haosulab/ManiSkill) - Robot simulation and manipulation learning package powered by SAPIEN. [⭐ 2.5k](https://github.com/haosulab/ManiSkill)
* 🔴 [MORSE](http://morse-simulator.github.io/) - Modular open robots simulation engine. [⭐ 370](https://github.com/morse-simulator/morse)
* [Neurorobotics Platform](https://neurorobotics.net/) - Internet-accessible simulation of robots controlled by spiking neural networks. [[bitbucket](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)]
* 🟢 [PyBullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) - An easy to use simulator for robotics and deep reinforcement learning. [⭐ 14.2k](https://github.com/bulletphysics/bullet3)
* 🟢 [PyBullet_Industrial](https://pybullet-industrial.readthedocs.io/en/latest/) - PyBullet extension for simulating robotic manufacturing processes like milling and 3D printing. [⭐ 48](https://github.com/WBK-Robotics/pybullet_industrial)
* 🔴 [Robot Gui](http://robot.glumb.de/) - A three.js based 3D robot interface. [⭐ 384](https://github.com/glumb/robot-gui)
* 🟢 [SAPIEN](https://sapien.ucsd.edu) - Physics-rich simulation environment for articulated objects and manipulation. [⭐ 714](https://github.com/haosulab/SAPIEN)
* [Simbad](http://simbad.sourceforge.net/) - Java 3D robot simulator with custom controller and sensor support.
* 🟡 [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics) - Game engine with open-source robotics simulation tools and tutorials. [⭐ 2.5k](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* 🟢 [Webots](http://www.cyberbotics.com/) - Development environment to model, program, and simulate robots and mechanical systems. [⭐ 4.1k](https://github.com/cyberbotics/webots)

###### Commercial

* [Actin Simulation](http://www.energid.com/) - Real-time robot simulation and control software.
* [Artiminds](https://www.artiminds.com/) - Planning, programming, operation, analysis and optimization.
* [Kineo](https://www.plm.automation.siemens.com/global/en/products/plm-components/kineo.html) - Path planning and trajectory optimization for industrial robotics.
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
* 🟢 [Brax](https://github.com/google/brax) - Massively parallel differentiable rigid body physics engine in JAX for robotics and RL. [⭐ 3k](https://github.com/google/brax)
* 🟢 [Bullet](https://pybullet.org/) - Real-time physics simulation for games, visual effects, and robotics. [⭐ 14.2k](https://github.com/bulletphysics/bullet3)
* 🟢 [CHRONO::ENGINE](https://projectchrono.org/) - Multi-physics simulation of rigid and flexible bodies, granular, and fluid systems. [⭐ 2.7k](https://github.com/projectchrono/chrono)
* 🟢 [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit for multibody simulation and planning. [⭐ 1.1k](https://github.com/dartsim/dart)
* 🟢 [Drake](https://drake.mit.edu/) - Planning, control, and analysis toolbox for nonlinear dynamical systems. [⭐ 3.9k](https://github.com/RobotLocomotion/drake)
* 💀 [Flex](https://developer.nvidia.com/flex) - GPU-based particle simulation for rigid bodies, fluids, and deformables. [⭐ 787](https://github.com/NVIDIAGameWorks/FleX)
* 🔴 [FROST](https://ayonga.github.io/frost-dev/index.html) - Fast Robot Optimization and Simulation Toolkit for hybrid dynamical systems in MATLAB. [⭐ 169](https://github.com/ayonga/frost-dev)
* 🟢 [Genesis](https://genesis-world.readthedocs.io) - Generative and universal physics platform for robotics with GPU-accelerated parallel simulation. [⭐ 28.1k](https://github.com/Genesis-Embodied-AI/Genesis)
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - Impulse-based dynamics simulation for rigid bodies and particle systems.
* 🟢 idyntree - Library for estimation and whole-body dynamics of floating-base robots. [⭐ 223](https://github.com/gbionics/idyntree)
* 🟢 [KDL](https://www.orocos.org/kdl.html) - Orocos Kinematics and Dynamics Library for kinematic chains. [⭐ 857](https://github.com/orocos/orocos_kinematics_dynamics)
* 🟢 kindr - Kinematics and dynamics library for rigid body transformations. [⭐ 607](https://github.com/ANYbotics/kindr)
* 🟢 [Klampt](https://klampt.org/) - Robot planning, control, and simulation with visualization support. [⭐ 427](https://github.com/krishauser/Klampt)
* 🔴 [LibrePilot](http://www.librepilot.org/site/index.html) - Open-source autopilot for UAVs and other autonomous vehicles. [⭐ 348](https://github.com/librepilot/LibrePilot)
* 🟢 [MARS](http://rock-simulation.github.io/mars/) - Machina Arte Robotum Simulans — a cross-platform simulation environment. [⭐ 67](https://github.com/rock-simulation/mars)
* [MBDyn](https://www.mbdyn.org/) - General-purpose multibody dynamics analysis software. [[code](https://www.mbdyn.org/?Software_Download)]
* 🟢 [MBSim](https://www.mbsim-env.de/) - Multi-body simulation environment for flexible and rigid systems. [⭐ 51](https://github.com/mbsim-env/mbsim)
* 🔴 [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) - Lightweight multibody system dynamics library. [⭐ 11](https://github.com/SIM-TU-Darmstadt/mbslib)
* 💀 metapod - Template-based robot dynamics library using spatial algebra. [⭐ 14](https://github.com/laas/metapod)
* 🔴 [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation for rigid bodies with contact. [⭐ 37](https://github.com/PositronicsLab/Moby)
* 🟢 [mrpt](https://www.mrpt.org/) - Mobile Robot Programming Toolkit for SLAM, navigation, and computer vision. [⭐ 2.1k](https://github.com/MRPT/mrpt)
* 🟢 [MuJoCo](https://mujoco.org/) - Multi-joint dynamics with contact for physics-based simulation and control. [⭐ 11.9k](https://github.com/google-deepmind/mujoco)
* 🟢 [mvsim](http://wiki.ros.org/mvsim) - Lightweight multi-vehicle 2D simulator with ROS integration. [⭐ 359](https://github.com/MRPT/mvsim)
* 🟢 [Newton](https://newton-physics.github.io/newton/) - GPU-accelerated differentiable physics engine built on NVIDIA Warp for robotics simulation. [⭐ 2.5k](https://github.com/newton-physics/newton)
* 🟢 [Newton Dynamics](https://newtondynamics.com/) - Real-time physics engine for rigid body simulation. [⭐ 1k](https://github.com/MADEAPPS/newton-dynamics)
* 🔴 [nphysics](https://nphysics.org/) - 2D and 3D rigid body physics engine written in Rust. [⭐ 1.6k](https://github.com/dimforge/nphysics)
* [ODE](https://ode.org/) - Open Dynamics Engine for simulating rigid body dynamics. [[bitbucket](https://bitbucket.org/odedevs/ode)]
* 🟢 [OpenRAVE](https://www.openrave.org/) - Open Robotics Automation Virtual Environment for planning and simulation. [⭐ 798](https://github.com/rdiankov/openrave)
* 🟢 [PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/index.html) - NVIDIA physics engine for real-time rigid body and vehicle simulation. [⭐ 4.4k](https://github.com/NVIDIA-Omniverse/PhysX)
* 🟢 [pinocchio](https://stack-of-tasks.github.io/pinocchio/) - Fast and flexible algorithms for rigid-body dynamics with analytical derivatives. [⭐ 3.1k](https://github.com/stack-of-tasks/pinocchio)
* 🟢 PositionBasedDynamics - Position-based methods for simulating deformable objects and fluids. [⭐ 2.2k](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics)
* 🟢 [PyDy](https://www.pydy.org/) - Multibody dynamics analysis with symbolic Python using SymPy. [⭐ 405](https://github.com/pydy/pydy)
* 💀 qu3e - Lightweight 3D physics engine for rigid body dynamics. [⭐ 974](https://github.com/RandyGaul/qu3e)
* 💀 [RaiSim](https://slides.com/jeminhwangbo/raisim-manual) - Cross-platform physics engine for robotics and reinforcement learning. [⭐ 329](https://github.com/leggedrobotics/raisimLib)
* 🟢 [RBDL](https://rbdl.github.io/) - Rigid Body Dynamics Library based on Featherstone algorithms. [⭐ 683](https://github.com/rbdl/rbdl)
* 🟢 RBDyn - Rigid body dynamics algorithms using spatial algebra with Eigen. [⭐ 211](https://github.com/jrl-umi3218/RBDyn)
* 🟢 [ReactPhysics3d](https://www.reactphysics3d.com/) - Open-source 3D physics engine for rigid body simulation and collision detection. [⭐ 1.7k](https://github.com/DanielChappuis/reactphysics3d)
* 🟡 RigidBodyDynamics.jl - Julia library for rigid body dynamics algorithms. [⭐ 307](https://github.com/JuliaRobotics/RigidBodyDynamics.jl)
* 🟢 [Rigs of Rods](https://www.rigsofrods.org/) - Soft-body vehicle simulator using beam physics. [⭐ 1.1k](https://github.com/RigsOfRods/rigs-of-rods)
* 🔴 [Robopy](https://adityadua24.github.io/robopy/) - Python robotics toolbox inspired by Peter Corke's Robotics Toolbox. [⭐ 228](https://github.com/adityadua24/robopy)
* 🟢 [Robotics Library](https://www.roboticslibrary.org/) - Self-contained C++ library for robot kinematics, planning, and control. [⭐ 1.2k](https://github.com/roboticslibrary/rl)
* [RobWork](https://robwork.dk/) - Framework for simulation and control of robot systems. [[gitlab](https://gitlab.com/sdurobotics/RobWork)]
* 🟢 [siconos](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/) - Nonsmooth dynamical systems modeling and simulation platform. [⭐ 182](https://github.com/siconos/siconos)
* 🟢 [Simbody](https://simtk.org/home/simbody/) - Multibody dynamics library for biomechanical and mechanical systems. [⭐ 2.5k](https://github.com/simbody/simbody)
* 🟢 [SOFA](https://www.sofa-framework.org/) - Simulation Open Framework Architecture for medical and physics simulation. [⭐ 1.1k](https://github.com/sofa-framework/sofa)
* 🟡 Tiny Differentiable Simulator - Header-only differentiable physics engine for robotics. [⭐ 1.3k](https://github.com/erwincoumans/tiny-differentiable-simulator)
* 🔴 [trep](http://murpheylab.github.io/trep/) - Simulation and optimal control using variational integrators. [⭐ 20](https://github.com/MurpheyLab/trep)

### [Inverse Kinematics](#contents)

_Libraries for computing joint configurations from end-effector poses._

  * 🟢 IKBT - A python package to solve robot arm inverse kinematics in symbolic form. [⭐ 215](https://github.com/uw-biorobotics/IKBT)
  * 🟢 Kinpy - A simple pure python package to solve inverse kinematics. [⭐ 179](https://github.com/neka-nat/kinpy)
  * 🔴 Lively - A highly configurable toolkit for commanding robots in mixed modalities. [⭐ 7](https://github.com/Wisc-HCI/lively)
  * 🔴 RelaxedIK - Real-time Synthesis of Accurate and Feasible Robot Arm Motion. [⭐ 235](https://github.com/uwgraphics/relaxed_ik)
  * 🔴 [Trip](https://trip-kinematics.readthedocs.io/en/main/index.html) - A python package that solves inverse kinematics of parallel-, serial- or hybrid-robots. [⭐ 44](https://github.com/TriPed-Robot/trip_kinematics)

### [Machine Learning](#contents)

_Machine learning frameworks and tools applied to robotics._

* 🟢 [AllenAct](https://allenact.org/) - Python/PyTorch-based Research Framework for Embodied AI. [⭐ 376](https://github.com/allenai/allenact)
* 🟢 Any4LeRobot - A collection of utilities and tools for LeRobot. [⭐ 838](https://github.com/Tavish9/any4lerobot)
* 🟢 DLL - Deep Learning Library (DLL) for C++. [⭐ 687](https://github.com/wichtounet/dll)
* 🔴 [DyNet](https://dynet.readthedocs.io/en/latest/) - The Dynamic Neural Network Toolkit. [⭐ 3.4k](https://github.com/clab/dynet)
* 🔴 [Fido](http://fidoproject.github.io/) - Lightweight C++ machine learning library for embedded electronics and robotics. [⭐ 462](https://github.com/FidoProject/Fido)
* 🟢 [Gymnasium](https://gymnasium.farama.org/) - Developing and comparing reinforcement learning algorithms. [⭐ 11.2k](https://github.com/Farama-Foundation/Gymnasium)
  * 🔴 gym-dart - OpenAI Gym environments using the DART physics engine. [⭐ 141](https://github.com/DartEnv/dart-env)
  * 💀 gym-gazebo - OpenAI Gym environments for the Gazebo simulator. [⭐ 845](https://github.com/erlerobot/gym-gazebo)
* 🟢 [Ivy](https://lets-unify.ai/) - Unified Machine Learning Framework. [⭐ 14.2k](https://github.com/ivy-llc/ivy)
* 🟢 LeRobot - Pretrained models, datasets, and simulation environments for real-world robotics in PyTorch. [⭐ 21.4k](https://github.com/huggingface/lerobot)
* 🟢 [LeRobot Episode Scoring Toolkit](https://github.com/RoboticsData/score_lerobot_episodes) - One-click tool to score, filter, and export higher-quality LeRobot datasets. [⭐ 48](https://github.com/RoboticsData/score_lerobot_episodes)
* 🔴 MiniDNN - A header-only C++ library for deep neural networks. [⭐ 431](https://github.com/yixuan/MiniDNN)
* 🟢 [mlpack](https://www.mlpack.org/) - Scalable C++ machine learning library. [⭐ 5.6k](https://github.com/mlpack/mlpack)
* 🔴 RLLib - Temporal-difference learning algorithms in reinforcement learning. [⭐ 208](https://github.com/samindaa/RLLib)
* 🟢 [robosuite](https://robosuite.ai) - A modular simulation framework and benchmark for robot learning. [⭐ 2.2k](https://github.com/ARISE-Initiative/robosuite)
* 🔴 [tiny-dnn](http://tiny-dnn.readthedocs.io/en/latest/) - Header only, dependency-free deep learning framework in C++14. [⭐ 6k](https://github.com/tiny-dnn/tiny-dnn)

### [Motion Planning and Control](#contents)

_Libraries for robot motion planning, trajectory optimization, and control._


* 🔴 [AIKIDO](https://github.com/personalrobotics/aikido) - Solving robotic motion planning and decision making problems. [⭐ 228](https://github.com/personalrobotics/aikido)
* 🟢 Bioptim - Bioptim, a Python Framework for Musculoskeletal Optimal Control in Biomechanics. [⭐ 113](https://github.com/pyomeca/bioptim)
* 🔴 [Control Toolbox](https://ethz-adrl.github.io/ct/) - Open-Source C++ Library for Robotics, Optimal and Model Predictive Control. [⭐ 1.7k](https://github.com/ethz-adrl/control-toolbox)
* 🟢 Crocoddyl - Optimal control library for robot control under contact sequence. [⭐ 1.2k](https://github.com/loco-3d/crocoddyl)
* [CuiKSuite](http://www.iri.upc.edu/people/porta/Soft/CuikSuite2-Doc/html) - Applications to solve position analysis and path planning problems.
* 🟢 [cuRobo](https://curobo.org) - A CUDA accelerated library containing a suite of robotics algorithms that run significantly faster. [⭐ 1.3k](https://github.com/nvlabs/curobo)
* 🟢 Fields2Cover - Robust and efficient coverage paths for autonomous agricultural vehicles. [⭐ 745](https://github.com/fields2cover/fields2cover)
* 🔴 GPMP2 - Gaussian Process Motion Planner 2. [⭐ 351](https://github.com/gtrll/gpmp2)
* [HPP](https://humanoid-path-planner.github.io/hpp-doc/) - Path planning for kinematic chains in environments cluttered with obstacles.
* 🟢 [MoveIt!](https://moveit.ai/) - Motion planning framework. [⭐ 2k](https://github.com/moveit/moveit)
* 🟢 OCS2 - Efficient continuous and discrete time optimal control implementation. [⭐ 1.3k](https://github.com/leggedrobotics/ocs2)
* 🟢 [OMPL](https://ompl.kavrakilab.org/) - Open motion planning library. [⭐ 1.9k](https://github.com/ompl/ompl)
* 💀 pymanoid - Humanoid robotics prototyping environment based on OpenRAVE. [⭐ 232](https://github.com/stephane-caron/pymanoid)
* 🔴 ROS Behavior Tree - Behavior tree implementation for ROS-based robot task planning. [⭐ 362](https://github.com/miccol/ROS-Behavior-Tree)
* 🟢 [Ruckig](https://github.com/pantor/ruckig) - Real-time, time-optimal and jerk-constrained online trajectory generation. [⭐ 1.1k](https://github.com/pantor/ruckig)
* 🟢 [The Kautham Project](https://sir.upc.es/projects/kautham/) - A robot simulation toolkit for motion planning. [⭐ 24](https://github.com/iocroblab/kautham)
* 🟢 [TOPP-RA](https://hungpham2511.github.io/toppra/) - Time-parameterizing robot trajectories subject to kinematic and dynamic constraints. [⭐ 830](https://github.com/hungpham2511/toppra)
* 🟡 [Ungar](https://github.com/fdevinc/ungar) - Expressive and efficient implementation of optimal control problems using template metaprogramming. [⭐ 109](https://github.com/fdevinc/ungar)

###### Motion Optimizer

* 🔴 TopiCo - Time-optimal Trajectory Generation and Control. [⭐ 143](https://github.com/AIS-Bonn/TopiCo)
* 🔴 [towr](http://wiki.ros.org/towr) - A light-weight, Eigen-based C++ library for trajectory optimization for legged robots. [⭐ 1k](https://github.com/ethz-adrl/towr)
* 🟢 TrajectoryOptimization - A fast trajectory optimization library written in Julia. [⭐ 386](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl)
* 🔴 [trajopt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/) - Framework for generating robot trajectories by local optimization. [⭐ 449](https://github.com/joschu/trajopt)

###### Nearest Neighbor

* 🔴 [Cover-Tree](http://hunch.net/~jl/projects/cover_tree/cover_tree.html) - Cover tree data structure for quick k-nearest-neighbor search. [⭐ 64](https://github.com/DNCrane/Cover-Tree)
  * [Faster cover trees](http://proceedings.mlr.press/v37/izbicki15.pdf) - by Mike Izbicki et al., ICML 2015.
* 🟡 [FLANN](http://www.cs.ubc.ca/research/flann/) - Fast Library for Approximate Nearest Neighbors. [⭐ 2.4k](https://github.com/flann-lib/flann)
* 🟢 [nanoflann](http://www.cs.ubc.ca/research/flann/) - Nearest Neighbor search with KD-trees. [⭐ 2.6k](https://github.com/jlblancoc/nanoflann)

###### 3D Mapping

* 🟢 Bonxai - Brutally fast, sparse, 3D Voxel Grid (formerly Treexy). [⭐ 810](https://github.com/facontidavide/Bonxai)
* 🟢 [Goxel](https://guillaumechereau.github.io/goxel/) - Free and open source 3D voxel editor. [⭐ 3.1k](https://github.com/guillaumechereau/goxel)
* 🟢 [libpointmatcher](http://libpointmatcher.readthedocs.io/en/latest/) - Iterative Closest Point library for 2-D/3-D mapping in Robotics. [⭐ 1.8k](https://github.com/norlab-ulaval/libpointmatcher)
* 🟢 [OctoMap](http://octomap.github.io/) - Efficient Probabilistic 3D Mapping Framework Based on Octrees. [⭐ 2.3k](https://github.com/OctoMap/octomap)
* 🔴 Octree - Fast radius neighbor search with an Octree. [⭐ 374](https://github.com/jbehley/octree)
* 🟢 [PCL](https://pointclouds.org/) - 2D/3D image and point cloud processing. [⭐ 10.8k](https://github.com/PointCloudLibrary/pcl)
* Utility Software
* 🟡 voxblox - Flexible voxel-based mapping focusing on truncated and Euclidean signed distance fields. [⭐ 1.6k](https://github.com/ethz-asl/voxblox)
* 🟡 [wavemap](https://projects.asl.ethz.ch/wavemap/) - Fast, efficient and accurate multi-resolution, multi-sensor 3D occupancy mapping. [⭐ 551](https://github.com/ethz-asl/wavemap)

### [Optimization](#contents)

_Numerical optimization solvers and frameworks used in robotics._

* 🟢 [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic framework for algorithmic differentiation and numeric optimization. [⭐ 2.1k](https://github.com/casadi/casadi)
* 🟢 [Ceres Solver](http://ceres-solver.org/) - Large scale nonlinear optimization library. [⭐ 4.4k](https://github.com/ceres-solver/ceres-solver)
* 🟢 eigen-qld - Interface to use the QLD QP solver with the Eigen3 library. [⭐ 16](https://github.com/jrl-umi3218/eigen-qld)
* 🟡 EXOTica - Generic optimisation toolset for robotics platforms. [⭐ 161](https://github.com/ipab-slmc/exotica)
* 🟢 hpipm - High-performance interior-point-method QP solvers (Ipopt, Snopt). [⭐ 665](https://github.com/giaf/hpipm)
* 🟢 [HYPRE](https://hypre.readthedocs.io/) - Parallel solvers for sparse linear systems featuring multigrid methods. [⭐ 812](https://github.com/hypre-space/hypre)
* 🟢 ifopt - An Eigen-based, light-weight C++ Interface to Nonlinear Programming Solvers (Ipopt, Snopt). [⭐ 847](https://github.com/ethz-adrl/ifopt)
* 🟢 [Ipopt](https://projects.coin-or.org/Ipopt) - Large scale nonlinear optimization library. [⭐ 1.7k](https://github.com/coin-or/Ipopt)
* 🟢 libcmaes - Blackbox stochastic optimization using the CMA-ES algorithm. [⭐ 354](https://github.com/CMA-ES/libcmaes)
* 🔴 [limbo](http://www.resibots.eu/limbo/) - Gaussian processes and Bayesian optimization of black-box functions. [⭐ 260](https://github.com/resibots/limbo)
* 🟢 lpsolvers - Linear Programming solvers in Python with a unified API. [⭐ 25](https://github.com/stephane-caron/lpsolvers)
* 🟢 [NLopt](https://nlopt.readthedocs.io/en/latest/) - Nonlinear optimization. [⭐ 2.2k](https://github.com/stevengj/nlopt)
* 🟡 [OptimLib](https://www.kthohr.com/optimlib.html) - Lightweight C++ library of numerical optimization methods for nonlinear functions. [⭐ 885](https://github.com/kthohr/optim)
* 🟢 [OSQP](https://osqp.org/) - The Operator Splitting QP Solver. [⭐ 2.1k](https://github.com/osqp/osqp)
* 🟢 [Pagmo](https://esa.github.io/pagmo2/index.html) - Scientific library for massively parallel optimization. [⭐ 907](https://github.com/esa/pagmo2)
* 🟢 [ProxSuite](https://simple-robotics.github.io/proxsuite/) - The Advanced Proximal Optimization Toolbox. [⭐ 538](https://github.com/Simple-Robotics/ProxSuite)
* 🔴 [pymoo](https://www.pymoo.org/) - Multi-objective Optimization in Python. [⭐ 26](https://github.com/msu-coinlab/pymoo)
* 🟢 qpsolvers - Quadratic Programming solvers in Python with a unified API. [⭐ 725](https://github.com/qpsolvers/qpsolvers)
* 🟢 [RobOptim](http://roboptim.net/index.html) - Numerical Optimization for Robotics. [⭐ 65](https://github.com/roboptim/roboptim-core)
* 🟢 [SCS](http://web.stanford.edu/~boyd/papers/scs.html) - Numerical optimization for solving large-scale convex cone problems. [⭐ 613](https://github.com/cvxgrp/scs)
* 🔴 sferes2 - Evolutionary computation. [⭐ 170](https://github.com/sferes2/sferes2)
* 🟢 SHOT - A solver for mixed-integer nonlinear optimization problems. [⭐ 129](https://github.com/coin-or/SHOT)

### [Robot Modeling](#contents)

_Tools and formats for describing robot models._

###### Robot Model Description Format

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. [[bitbucket](https://bitbucket.org/osrf/sdformat)]
* 🟢 [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. [⭐ 118](https://github.com/ros/urdfdom)

###### Utility to Build Robot Models

* 🟢 [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API. [⭐ 486](https://github.com/Rhoban/onshape-to-robot)
* 🟢 phobos - Add-on for Blender creating URDF and SMURF robot models. [⭐ 855](https://github.com/dfki-ric/phobos)

### [Robot Platform](#contents)

_Middleware and frameworks for building robot software systems._

* 🔴 [AutoRally](http://autorally.github.io/) - High-performance testbed for advanced perception and control research. [⭐ 775](https://github.com/autorally/autorally)
* 🔴 [Linorobot](https://linorobot.org/) - ROS compatible ground robots. [⭐ 1.1k](https://github.com/linorobot/linorobot)
  * 🔴 onine - Service Robot based on Linorobot and Braccio Arm. [⭐ 47](https://github.com/grassjelly/onine)
* 🟡 [Micro-ROS for Arduino](https://github.com/kaiaai/micro_ros_arduino_kaiaai) - a Micro-ROS fork available in the Arduino Library Manager. [⭐ 12](https://github.com/kaiaai/micro_ros_arduino_kaiaai)
* [Rock](https://www.rock-robotics.org/) - Software framework for robotic systems.
* [ROS](https://www.ros.org/) - Flexible framework for writing robot software.
* 🟢 [ROS 2](https://github.com/ros2/ros2/wiki) - Version 2.0 of the Robot Operating System (ROS) software stack. [⭐ 5k](https://github.com/ros2/ros2)
* 🟢 [YARP](https://www.yarp.it/) - Communication and device interfaces applicable from humanoids to embedded devices. [⭐ 585](https://github.com/robotology/yarp)

### [Reinforcement Learning for Robotics](#contents)

_Reinforcement learning libraries commonly used in robotic control._

* 🟢 [Brax](https://github.com/google/brax) - Massively parallel differentiable rigid body physics engine in JAX for robotics and RL. [⭐ 3k](https://github.com/google/brax)
* 🟢 [CleanRL](https://github.com/vwxyzjn/cleanrl) - Single-file implementations of deep reinforcement learning algorithms. [⭐ 9k](https://github.com/vwxyzjn/cleanrl)
* 🟢 [Isaac Lab](https://isaac-sim.github.io/IsaacLab) - GPU-accelerated open-source framework for robot learning built on NVIDIA Isaac Sim. [⭐ 6.2k](https://github.com/isaac-sim/IsaacLab)
* 🟢 [rl_games](https://github.com/Denys88/rl_games) - High-performance RL library used in Isaac Gym environments. [⭐ 1.3k](https://github.com/Denys88/rl_games)
* 🟢 [SKRL](https://github.com/Toni-SM/skrl) - Modular reinforcement learning library with support for multiple ML frameworks. [⭐ 982](https://github.com/Toni-SM/skrl)
* 🟢 [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) - Reliable implementations of reinforcement learning algorithms in PyTorch. [⭐ 12.7k](https://github.com/DLR-RM/stable-baselines3)

### [SLAM](#contents)

_Simultaneous Localization and Mapping libraries._


* 🔴 AprilSAM - Real-time smoothing and mapping. [⭐ 239](https://github.com/xipengwang/AprilSAM)
* 🔴 Cartographer - Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations. [⭐ 7.8k](https://github.com/cartographer-project/cartographer)
* 🟡 [DSO](https://vision.in.tum.de/research/vslam/dso) - Novel direct and sparse formulation for Visual Odometry. [⭐ 2.4k](https://github.com/JakobEngel/dso)
* 🟢 ElasticFusion - Real-time dense visual SLAM system. [⭐ 1.9k](https://github.com/mp3guy/ElasticFusion)
* 🟢 [fiducials](http://wiki.ros.org/fiducials) - Simultaneous localization and mapping using fiducial markers. [⭐ 278](https://github.com/UbiquityRobotics/fiducials)
* 🟢 GTSAM - Smoothing and mapping (SAM) in robotics and vision. [⭐ 3.3k](https://github.com/borglab/gtsam)
* 🔴 Kintinuous - Real-time large scale dense visual SLAM system. [⭐ 951](https://github.com/mp3guy/Kintinuous)
* 🔴 [LSD-SLAM](https://vision.in.tum.de/research/vslam/lsdslam) - Real-time monocular SLAM. [⭐ 2.7k](https://github.com/tum-vision/lsd_slam)
* 🟡 ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras. [⭐ 10.1k](https://github.com/raulmur/ORB_SLAM2)
* 🟢 [RTAP-Map](http://introlab.github.io/rtabmap/) - RGB-D Graph SLAM approach based on a global Bayesian loop closure detector. [⭐ 3.6k](https://github.com/introlab/rtabmap)
* 🔴 [SRBA](http://mrpt.github.io/srba/) - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies. [⭐ 76](https://github.com/MRPT/srba)

#### SLAM Dataset

* 🟡 [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets) - Curated list of SLAM-related datasets. [⭐ 1.9k](https://github.com/youngguncho/awesome-slam-datasets)

### [Vision](#contents)

_Computer vision libraries for robotic perception._

* 🔴 [BundleTrack](https://github.com/wenbowen123/BundleTrack) - 6D Pose Tracking for Novel Objects without 3D Models. [⭐ 678](https://github.com/wenbowen123/BundleTrack)
* 🔴 [se(3)-TrackNet](https://github.com/wenbowen123/iros20-6d-pose-tracking) - 6D Pose Tracking for Novel Objects without 3D Models. [⭐ 420](https://github.com/wenbowen123/iros20-6d-pose-tracking)
* 🟢 [ViSP](http://visp.inria.fr/) - Visual Servoing Platform. [⭐ 849](https://github.com/lagadic/visp)

### [Fluid](#contents)

_Fluid dynamics simulation libraries._

* 🔴 [Fluid Engine Dev - Jet](https://fluidenginedevelopment.org/) - Fluid simulation engine for computer graphics applications. [⭐ 2.1k](https://github.com/doyubkim/fluid-engine-dev)

### [Grasping](#contents)

_Libraries and tools for robotic grasping and manipulation._

* 🟢 [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) - SDK for AnyGrasp, a 6-DoF grasp pose detection method. [⭐ 752](https://github.com/graspnet/anygrasp_sdk)
* 🟡 [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) - 6-DoF grasp generation for parallel-jaw grippers using contact maps. [⭐ 449](https://github.com/NVlabs/contact_graspnet)
* 🔴 [GraspIt!](https://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs. [⭐ 207](https://github.com/graspit-simulator/graspit)
* 🟢 [GraspNet API](https://github.com/graspnet/graspnetAPI) - Python API and evaluation tools for the GraspNet benchmark. [⭐ 323](https://github.com/graspnet/graspnetAPI)

### [Humanoid Robotics](#contents)

_Environments and models for humanoid robot research._

* 🟡 [Humanoid-Gym](https://github.com/roboterax/humanoid-gym) - Reinforcement learning environment for humanoid robot locomotion. [⭐ 1.8k](https://github.com/roboterax/humanoid-gym)
* 🟢 [Legged Gym](https://github.com/leggedrobotics/legged_gym) - Isaac Gym environments for legged robot locomotion training. [⭐ 2.7k](https://github.com/leggedrobotics/legged_gym)
* 🟢 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - Collection of well-tuned MuJoCo models for research and development. [⭐ 3k](https://github.com/google-deepmind/mujoco_menagerie)

### [Multiphysics](#contents)

_Frameworks for coupled multi-physics simulations._

* 🟢 [Kratos](http://www.cimne.com/kratos/) - Framework for building parallel multi-disciplinary simulation software. [⭐ 1.2k](https://github.com/KratosMultiphysics/Kratos)

### [Math](#contents)

_Mathematics libraries for spatial algebra, Lie groups, and linear algebra._

* 🟢 Fastor - Light-weight high performance tensor algebra framework in C++11/14/17. [⭐ 830](https://github.com/romeric/Fastor)
* 🔴 linalg.h - Single header public domain linear algebra library for C++11. [⭐ 939](https://github.com/sgorsten/linalg)
* 🟢 manif - Small c++11 header-only library for Lie theory. [⭐ 1.7k](https://github.com/artivis/manif)
* 🟡 Sophus - Lie groups using Eigen. [⭐ 2.4k](https://github.com/strasdat/Sophus)
* 🟢 SpaceVelAlg - Spatial vector algebra with the Eigen3. [⭐ 80](https://github.com/jrl-umi3218/SpaceVecAlg)
* 🟢 spatialmath-python - Python classes for pose and orientation in 2D/3D with spatial operations toolbox. [⭐ 615](https://github.com/bdaiinstitute/spatialmath-python)

### [ETC](#contents)

_Other robotics-related tools and utilities._

* [Foxglove Studio](https://foxglove.dev) - A fully integrated visualization and debugging desktop app for your robotics data.
* 🟢 fuse - General architecture for performing sensor fusion live on a robot. [⭐ 847](https://github.com/locusrobotics/fuse)

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

## [Star History](#contents)

[![Star History Chart](https://api.star-history.com/svg?repos=jslee02/awesome-robotics-libraries&type=Date)](https://star-history.com/#jslee02/awesome-robotics-libraries)

## [License](#contents)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
