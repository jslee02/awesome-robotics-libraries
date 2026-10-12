# Awesome Robotics Libraries

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of robotics libraries and simulators.

## Contents
* [Middleware & Frameworks](#middleware--frameworks)
* [Robot Hardware & Drivers](#robot-hardware--drivers)
* [Robot Description & Models](#robot-description--models)
* [Kinematics & Dynamics](#kinematics--dynamics)
* [Physics Engines](#physics-engines)
* [Simulators](#simulators)
* [Motion Planning & Navigation](#motion-planning--navigation)
* [Control](#control)
* [Optimization](#optimization)
* [Math & Geometry](#math--geometry)
* [SLAM & State Estimation](#slam--state-estimation)
* [Perception](#perception)
* [Grasping & Manipulation](#grasping--manipulation)
* [Robot Learning](#robot-learning)
* [Robot Data & Visualization](#robot-data--visualization)
* [Other Awesome Lists](#other-awesome-lists)
* [Contributing](#contributing)

> **Legend**: 🟢 commit within 1 yr · 🟡 commit within 3 yr · 💀 archived · ⭐ GitHub stars as of 2026-10-11. Entries are ordered by stars; inactive entries (archived, or no commits for 3+ years) are folded at the end of each category.

## [Middleware & Frameworks](#contents)

_Communication middleware, robot software frameworks, runtime services such as launch and diagnostics, and agent orchestration for robots._

* 🟢 [ROS 2](https://github.com/ros2/ros2/wiki) - Version 2.0 of the Robot Operating System (ROS) software stack. [⭐ 6.1k](https://github.com/ros2/ros2)
* 🟢 [Dimensional](https://dimensional.org/) - Agentic operating system for physical space. [⭐ 4.6k](https://github.com/dimensionalOS/dimos)
* 🟢 [YARP](https://www.yarp.it/) - Communication and device interfaces applicable from humanoids to embedded devices. [⭐ 603](https://github.com/robotology/yarp)
* 🟢 [ros2_medkit](https://selfpatch.github.io/ros2_medkit/) - Fault management and diagnostics gateway for ROS 2 with a REST API. [⭐ 276](https://github.com/selfpatch/ros2_medkit)
* 🟢 [Robot Control Stack (RCS)](https://robotcontrolstack.org) - ROS-free Gymnasium framework unifying MuJoCo sim and real robots for robot learning. [⭐ 164](https://github.com/RobotControlStack/robot-control-stack)
* 🟡 [Micro-ROS for Arduino](https://github.com/kaiaai/micro_ros_arduino_kaiaai) - a Micro-ROS fork available in the Arduino Library Manager. [⭐ 14](https://github.com/kaiaai/micro_ros_arduino_kaiaai)
* [Rock](https://www.rock-robotics.org/) - Software framework for robotic systems.
* [ROS](https://www.ros.org/) - Flexible framework for writing robot software.

## [Robot Hardware & Drivers](#contents)

_Open robot platforms and hardware designs, and drivers or firmware for sensors and actuators._

* 🟢 [iCub](http://www.icub.org/) - Open-source cognitive humanoid robotic platform for embodied cognition research. [⭐ 121](https://github.com/robotology/icub-main)

<details>
<summary>Inactive (3): archived or no commits for 3+ years</summary>

* [Linorobot](https://linorobot.org/) - ROS compatible ground robots. [⭐ 1.1k](https://github.com/linorobot/linorobot) · last commit 2022
  * onine - Service Robot based on Linorobot and Braccio Arm. [⭐ 50](https://github.com/grassjelly/onine) · last commit 2018
* [AutoRally](http://autorally.github.io/) - High-performance testbed for advanced perception and control research. [⭐ 791](https://github.com/autorally/autorally) · last commit 2021

</details>

## [Robot Description & Models](#contents)

_Robot description formats (URDF, SDF, MJCF, USD), their parsers and exporters, and robot model collections._

* 🟢 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - Collection of well-tuned MuJoCo models for research and development. [⭐ 4.2k](https://github.com/google-deepmind/mujoco_menagerie)
* 🟢 phobos - Add-on for Blender creating URDF and SMURF robot models. [⭐ 908](https://github.com/dfki-ric/phobos)
* 🟢 [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API. [⭐ 638](https://github.com/Rhoban/onshape-to-robot)
* 🟢 [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. [⭐ 137](https://github.com/ros/urdfdom)
* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. [[bitbucket](https://bitbucket.org/osrf/sdformat)]

## [Kinematics & Dynamics](#contents)

_Forward and inverse kinematics and rigid-body dynamics algorithms, without contact time-stepping._

* 🟢 [pinocchio](https://stack-of-tasks.github.io/pinocchio/) - Fast and flexible algorithms for rigid-body dynamics with analytical derivatives. [⭐ 3.8k](https://github.com/stack-of-tasks/pinocchio)
* 🟡 [Robotics Library](https://www.roboticslibrary.org/) - Self-contained C++ library for robot kinematics, planning, and control. [⭐ 1.2k](https://github.com/roboticslibrary/rl)
* 🟢 [KDL](https://www.orocos.org/kdl.html) - Orocos Kinematics and Dynamics Library for kinematic chains. [⭐ 904](https://github.com/orocos/orocos_kinematics_dynamics)
* 🟡 [RBDL](https://rbdl.github.io/) - Rigid Body Dynamics Library based on Featherstone algorithms. [⭐ 708](https://github.com/rbdl/rbdl)
* 🟢 [PyDy](https://www.pydy.org/) - Multibody dynamics analysis with symbolic Python using SymPy. [⭐ 416](https://github.com/pydy/pydy)
* 🟡 RigidBodyDynamics.jl - Julia library for rigid body dynamics algorithms. [⭐ 312](https://github.com/JuliaRobotics/RigidBodyDynamics.jl)
* 🟢 idyntree - Library for estimation and whole-body dynamics of floating-base robots. [⭐ 234](https://github.com/gbionics/idyntree)
* 🟢 RBDyn - Rigid body dynamics algorithms using spatial algebra with Eigen. [⭐ 224](https://github.com/jrl-umi3218/RBDyn)
* 🟢 [ARTE](http://arvc.umh.es/arte/index_en.html) - Matlab toolbox focussed on robotic manipulators. [⭐ 106](https://github.com/4rtur1t0/ARTE)
* [RobWork](https://robwork.dk/) - Framework for simulation and control of robot systems. [[gitlab](https://gitlab.com/sdurobotics/RobWork)]

### Inverse Kinematics

* 🟢 IKBT - A python package to solve robot arm inverse kinematics in symbolic form. [⭐ 220](https://github.com/uw-biorobotics/IKBT)
* 🟢 ssik - Analytical inverse kinematics for 6R and 7R revolute robot arms, returning every IK branch. [⭐ 188](https://github.com/personalrobotics/ssik)
* 🟢 Kinpy - A simple pure python package to solve inverse kinematics. [⭐ 181](https://github.com/neka-nat/kinpy)

<details>
<summary>Inactive (6): archived or no commits for 3+ years</summary>

* RelaxedIK - Real-time Synthesis of Accurate and Feasible Robot Arm Motion. [⭐ 254](https://github.com/uwgraphics/relaxed_ik) · last commit 2023
* [Robopy](https://adityadua24.github.io/robopy/) - Python robotics toolbox inspired by Peter Corke's Robotics Toolbox. [⭐ 229](https://github.com/adityadua24/robopy) · last commit 2021
* [Trip](https://trip-kinematics.readthedocs.io/en/main/index.html) - A python package that solves inverse kinematics of parallel-, serial- or hybrid-robots. [⭐ 43](https://github.com/TriPed-Robot/trip_kinematics) · last commit 2022
* [trep](http://murpheylab.github.io/trep/) - Simulation and optimal control using variational integrators. [⭐ 20](https://github.com/MurpheyLab/trep) · last commit 2023
* 💀 metapod - Template-based robot dynamics library using spatial algebra. [⭐ 14](https://github.com/laas/metapod) · last commit 2018
* Lively - A highly configurable toolkit for commanding robots in mixed modalities. [⭐ 8](https://github.com/Wisc-HCI/lively) · last commit 2023

</details>

## [Physics Engines](#contents)

_Embeddable engines that time-step rigid, soft, fluid, or particle dynamics with contact. See also [Comparisons](COMPARISONS.md)._

* 🟢 [Genesis](https://genesis-world.readthedocs.io) - Generative and universal physics platform for robotics with GPU-accelerated parallel simulation. [⭐ 30k](https://github.com/Genesis-Embodied-AI/genesis-world)
* 🟢 [MuJoCo](https://mujoco.org/) - Multi-joint dynamics with contact for physics-based simulation and control. [⭐ 15.6k](https://github.com/google-deepmind/mujoco)
* 🟢 [Bullet](https://pybullet.org/) - Real-time physics simulation for games, visual effects, and robotics. [⭐ 14.8k](https://github.com/bulletphysics/bullet3)
* 🟢 [Newton](https://newton-physics.github.io/newton/) - GPU-accelerated differentiable physics engine built on NVIDIA Warp for robotics simulation. [⭐ 5.7k](https://github.com/newton-physics/newton)
* 🟢 [PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/index.html) - NVIDIA physics engine for real-time rigid body and vehicle simulation. [⭐ 4.8k](https://github.com/NVIDIA-Omniverse/PhysX)
* 🟢 [Drake](https://drake.mit.edu/) - Planning, control, and analysis toolbox for nonlinear dynamical systems. [⭐ 4.2k](https://github.com/RobotLocomotion/drake)
* 🟢 [Brax](https://github.com/google/brax) - Massively parallel differentiable rigid body physics engine in JAX for robotics and RL. [⭐ 3.2k](https://github.com/google/brax)
* 🟢 [CHRONO::ENGINE](https://projectchrono.org/) - Multi-physics simulation of rigid and flexible bodies, granular, and fluid systems. [⭐ 3.1k](https://github.com/projectchrono/chrono)
* 🟢 [Simbody](https://simtk.org/home/simbody/) - Multibody dynamics library for biomechanical and mechanical systems. [⭐ 2.6k](https://github.com/simbody/simbody)
* 🟡 [ReactPhysics3d](https://www.reactphysics3d.com/) - Open-source 3D physics engine for rigid body simulation and collision detection. [⭐ 1.8k](https://github.com/DanielChappuis/reactphysics3d)
* 🟢 [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit for multibody simulation and planning. [⭐ 1.2k](https://github.com/dartsim/dart)
* 🟢 [Newton Dynamics](https://newtondynamics.com/) - Real-time physics engine for rigid body simulation. [⭐ 1k](https://github.com/MADEAPPS/newton-dynamics)
* 🟢 [siconos](https://nonsmooth.gricad-pages.univ-grenoble-alpes.fr/siconos/) - Nonsmooth dynamical systems modeling and simulation platform. [⭐ 186](https://github.com/siconos/siconos)
* 🟢 [MBSim](https://www.mbsim-env.de/) - Multi-body simulation environment for flexible and rigid systems. [⭐ 53](https://github.com/mbsim-env/mbsim)
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - Impulse-based dynamics simulation for rigid bodies and particle systems.
* [MBDyn](https://www.mbdyn.org/) - General-purpose multibody dynamics analysis software. [[code](https://www.mbdyn.org/?Software_Download)]
* [ODE](https://ode.org/) - Open Dynamics Engine for simulating rigid body dynamics. [[bitbucket](https://bitbucket.org/odedevs/ode)]

### Soft, Fluid & Multiphysics

* 🟢 PositionBasedDynamics - Position-based methods for simulating deformable objects and fluids. [⭐ 2.3k](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics)
* 🟢 [Kratos](http://www.cimne.com/kratos/) - Framework for building parallel multi-disciplinary simulation software. [⭐ 1.4k](https://github.com/KratosMultiphysics/Kratos)
* 🟢 [SOFA](https://www.sofa-framework.org/) - Simulation Open Framework Architecture for medical and physics simulation. [⭐ 1.3k](https://github.com/sofa-framework/sofa)
* 🟢 [Ptera Software](https://docs.pterasoftware.com/) - Open-source package for analyzing flapping-wing flight. [⭐ 270](https://github.com/camUrban/PteraSoftware)
* [ARCSim](http://graphics.berkeley.edu/resources/ARCSim/index.html) - Adaptive remeshing cloth and shell simulator for thin deformable objects.

<details>
<summary>Inactive (8): archived or no commits for 3+ years</summary>

* [Fluid Engine Dev - Jet](https://fluidenginedevelopment.org/) - Fluid simulation engine for computer graphics applications. [⭐ 2.1k](https://github.com/doyubkim/fluid-engine-dev) · last commit 2022
* [nphysics](https://nphysics.org/) - 2D and 3D rigid body physics engine written in Rust. [⭐ 1.6k](https://github.com/dimforge/nphysics) · last commit 2021
* Tiny Differentiable Simulator - Header-only differentiable physics engine for robotics. [⭐ 1.4k](https://github.com/erwincoumans/tiny-differentiable-simulator) · last commit 2023
* 💀 qu3e - Lightweight 3D physics engine for rigid body dynamics. [⭐ 994](https://github.com/RandyGaul/qu3e) · last commit 2021
* 💀 [Flex](https://developer.nvidia.com/flex) - GPU-based particle simulation for rigid bodies, fluids, and deformables. [⭐ 814](https://github.com/NVIDIAGameWorks/FleX) · last commit 2021
* 💀 [RaiSim](https://slides.com/jeminhwangbo/raisim-manual) - Cross-platform physics engine for robotics and reinforcement learning. [⭐ 328](https://github.com/leggedrobotics/raisimLib) · last commit 2020
* [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation for rigid bodies with contact. [⭐ 37](https://github.com/PositronicsLab/Moby) · last commit 2021
* [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) - Lightweight multibody system dynamics library. [⭐ 11](https://github.com/SIM-TU-Darmstadt/mbslib) · last commit 2016

</details>

## [Simulators](#contents)

_Simulation applications with worlds, sensors, rendering, and robot middleware bridges._

* 🟢 AirSim - Simulator based on Unreal Engine for autonomous vehicles. [⭐ 18.5k](https://github.com/Microsoft/AirSim)
* 🟢 [PyBullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) - An easy to use simulator for robotics and deep reinforcement learning. [⭐ 14.8k](https://github.com/bulletphysics/bullet3)
* 🟢 [CARLA](https://carla.org/) - Open-source simulator for autonomous driving research. [⭐ 14.5k](https://github.com/carla-simulator/carla)
* 🟢 [Webots](http://www.cyberbotics.com/) - Development environment to model, program, and simulate robots and mechanical systems. [⭐ 4.7k](https://github.com/cyberbotics/webots)
* 🟢 [Isaac Sim](https://developer.nvidia.com/isaac/sim) - NVIDIA's GPU-accelerated robotics simulation platform with PhysX 5 and RTX rendering. [⭐ 4.2k](https://github.com/isaac-sim/IsaacSim)
* 🟢 [Habitat-Sim](https://aihabitat.org/) - Simulation platform for research in embodied artificial intelligence. [⭐ 3.8k](https://github.com/facebookresearch/habitat-sim)
* 🟢 [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics) - Game engine with open-source robotics simulation tools and tutorials. [⭐ 2.6k](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* 🟡 [AI2-THOR](https://ai2thor.allenai.org/) - Interactive household environment for embodied AI with Unity backend. [⭐ 1.8k](https://github.com/allenai/ai2thor)
* 🟢 [Gazebo Sim](https://gazebosim.org/) - Open source robotics simulator (formerly Ignition Gazebo). [⭐ 1.5k](https://github.com/gazebosim/gz-sim)
* 🟢 [Rigs of Rods](https://www.rigsofrods.org/) - Soft-body vehicle simulator using beam physics. [⭐ 1.3k](https://github.com/RigsOfRods/rigs-of-rods)
* 🟢 [SAPIEN](https://sapien.ucsd.edu) - Physics-rich simulation environment for articulated objects and manipulation. [⭐ 851](https://github.com/haosulab/SAPIEN)
* 🟢 [Hexapod Robot Simulator](https://hexapod.netlify.app/) - Open-source hexapod robot inverse kinematics and gaits visualizer. [⭐ 796](https://github.com/mithi/hexapod)
* 🟢 [mvsim](http://wiki.ros.org/mvsim) - Lightweight multi-vehicle 2D simulator with ROS integration. [⭐ 416](https://github.com/MRPT/mvsim)
* 🟢 [ARGoS](https://www.argos-sim.info/) - Physics-based simulator designed to simulate large-scale robot swarms. [⭐ 321](https://github.com/ilpincy/argos3)
* 🟢 [CoppeliaSim](https://www.coppeliarobotics.com/) - Formaly V-REP. Virtual robot experimentation platform. [⭐ 152](https://github.com/CoppeliaRobotics/CoppeliaSimLib)
* 🟡 [MARS](http://rock-simulation.github.io/mars/) - Machina Arte Robotum Simulans — a cross-platform simulation environment. [⭐ 69](https://github.com/rock-simulation/mars)
* 🟡 [PyBullet_Industrial](https://pybullet-industrial.readthedocs.io/en/latest/) - PyBullet extension for simulating robotic manufacturing processes like milling and 3D printing. [⭐ 56](https://github.com/WBK-Robotics/pybullet_industrial)
* 🟡 [AVIS Engine](https://avisengine.com) - Fast simulation software for autonomous vehicle development. [⭐ 22](https://github.com/AvisEngine/AVIS-Engine-Python-API)
* [Neurorobotics Platform](https://neurorobotics.net/) - Internet-accessible simulation of robots controlled by spiking neural networks. [[bitbucket](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)]
* [Simbad](http://simbad.sourceforge.net/) - Java 3D robot simulator with custom controller and sensor support.

### Commercial

* [Actin Simulation](http://www.energid.com/) - Real-time robot simulation and control software.
* [Artiminds](https://www.artiminds.com/) - Planning, programming, operation, analysis and optimization.
* [Kineo](https://www.plm.automation.siemens.com/global/en/products/plm-components/kineo.html) - Path planning and trajectory optimization for industrial robotics.
* [Robot Virtual Worlds](http://www.robotvirtualworlds.com/) - Virtual reality software for educational robotics.
* [RobotDK](https://robodk.com/) - Simulation and OLP for robots.
* [RobotStudio](https://www.abb.com/global/en/areas/robotics/products/software/robotstudio-suite) - ABB's simulation and offline programming software for robotics.
* [Virtual Robotics Toolkit](https://www.virtualroboticstoolkit.com/) - 3D virtual environment for programming and testing robots.
* [Visual Components](https://www.visualcomponents.com/) - 3D manufacturing simulation and visualization platform.

<details>
<summary>Inactive (2): archived or no commits for 3+ years</summary>

* 💀 [Gazebo Classic](https://gazebosim.org/) - Dynamic multi-robot simulator. [⭐ 1.4k](https://github.com/gazebosim/gazebo-classic) · last commit 2024
* [MORSE](http://morse-simulator.github.io/) - Modular open robots simulation engine. [⭐ 370](https://github.com/morse-simulator/morse) · last commit 2022

</details>

## [Motion Planning & Navigation](#contents)

_Path planning, trajectory generation and time-parameterization, task and behavior planning, and navigation stacks._

* 🟢 [ROS Motion Planning](https://github.com/ai-winter/ros_motion_planning) - ROS planner and controller plugins for mobile robot navigation. [⭐ 3.6k](https://github.com/ai-winter/ros_motion_planning)
* 🟢 [OMPL](https://ompl.kavrakilab.org/) - Open motion planning library. [⭐ 2.2k](https://github.com/ompl/ompl)
* 🟢 [MoveIt!](https://moveit.ai/) - Motion planning framework. [⭐ 2.1k](https://github.com/moveit/moveit)
* 🟢 [cuRobo](https://curobo.org) - A CUDA accelerated library containing a suite of robotics algorithms that run significantly faster. [⭐ 1.9k](https://github.com/nvlabs/curobo)
* 🟢 [Ruckig](https://github.com/pantor/ruckig) - Real-time, time-optimal and jerk-constrained online trajectory generation. [⭐ 1.4k](https://github.com/pantor/ruckig)
* 🟢 [Python Motion Planning](https://github.com/ai-winter/python_motion_planning) - Python implementations of path planners, trajectory trackers, and curve generators. [⭐ 1.1k](https://github.com/ai-winter/python_motion_planning)
* 🟢 [TOPP-RA](https://hungpham2511.github.io/toppra/) - Time-parameterizing robot trajectories subject to kinematic and dynamic constraints. [⭐ 933](https://github.com/hungpham2511/toppra)
* 🟢 Fields2Cover - Robust and efficient coverage paths for autonomous agricultural vehicles. [⭐ 909](https://github.com/fields2cover/fields2cover)
* 🟢 [OpenRAVE](https://www.openrave.org/) - Open Robotics Automation Virtual Environment for planning and simulation. [⭐ 821](https://github.com/rdiankov/openrave)
* 🟢 [Klampt](https://klampt.org/) - Robot planning, control, and simulation with visualization support. [⭐ 437](https://github.com/krishauser/Klampt)
* 🟢 [The Kautham Project](https://sir.upc.es/projects/kautham/) - A robot simulation toolkit for motion planning. [⭐ 24](https://github.com/iocroblab/kautham)
* [CuiKSuite](http://www.iri.upc.edu/people/porta/Soft/CuikSuite2-Doc/html) - Applications to solve position analysis and path planning problems.
* [HPP](https://humanoid-path-planner.github.io/hpp-doc/) - Path planning for kinematic chains in environments cluttered with obstacles.

<details>
<summary>Inactive (7): archived or no commits for 3+ years</summary>

* [trajopt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/) - Framework for generating robot trajectories by local optimization. [⭐ 466](https://github.com/joschu/trajopt) · last commit 2017
* ROS Behavior Tree - Behavior tree implementation for ROS-based robot task planning. [⭐ 365](https://github.com/miccol/ROS-Behavior-Tree) · last commit 2018
* GPMP2 - Gaussian Process Motion Planner 2. [⭐ 358](https://github.com/gtrll/gpmp2) · last commit 2022
* [LibrePilot](http://www.librepilot.org/site/index.html) - Open-source autopilot for UAVs and other autonomous vehicles. [⭐ 358](https://github.com/librepilot/LibrePilot) · last commit 2019
* [AIKIDO](https://github.com/personalrobotics/aikido) - Solving robotic motion planning and decision making problems. [⭐ 235](https://github.com/personalrobotics/aikido) · last commit 2023
* 💀 pymanoid - Humanoid robotics prototyping environment based on OpenRAVE. [⭐ 234](https://github.com/stephane-caron/pymanoid) · last commit 2024
* TopiCo - Time-optimal Trajectory Generation and Control. [⭐ 146](https://github.com/AIS-Bonn/TopiCo) · last commit 2021

</details>

## [Control](#contents)

_Model-based control: MPC, trajectory optimization, whole-body and task-space control._

* 🟢 OCS2 - Efficient continuous and discrete time optimal control implementation. [⭐ 1.5k](https://github.com/leggedrobotics/ocs2)
* 🟢 Crocoddyl - Optimal control library for robot control under contact sequence. [⭐ 1.3k](https://github.com/loco-3d/crocoddyl)
* 🟡 TrajectoryOptimization - A fast trajectory optimization library written in Julia. [⭐ 400](https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl)
* 🟢 Bioptim - Python framework for musculoskeletal optimal control in biomechanics. [⭐ 120](https://github.com/pyomeca/bioptim)
* 🟡 [Ungar](https://github.com/fdevinc/ungar) - Expressive and efficient implementation of optimal control problems using template metaprogramming. [⭐ 108](https://github.com/fdevinc/ungar)

<details>
<summary>Inactive (3): archived or no commits for 3+ years</summary>

* [Control Toolbox](https://ethz-adrl.github.io/ct/) - Open-Source C++ Library for Robotics, Optimal and Model Predictive Control. [⭐ 1.7k](https://github.com/ethz-adrl/control-toolbox) · last commit 2021
* [towr](http://wiki.ros.org/towr) - A light-weight, Eigen-based C++ library for trajectory optimization for legged robots. [⭐ 1.1k](https://github.com/ethz-adrl/towr) · last commit 2021
* [FROST](https://ayonga.github.io/frost-dev/index.html) - Fast Robot Optimization and Simulation Toolkit for hybrid dynamical systems in MATLAB. [⭐ 170](https://github.com/ayonga/frost-dev) · last commit 2021

</details>

## [Optimization](#contents)

_General-purpose numerical solvers (LP, QP, NLP, least squares, black-box) and modeling layers._

* 🟢 [Ceres Solver](http://ceres-solver.org/) - Large scale nonlinear optimization library. [⭐ 4.6k](https://github.com/ceres-solver/ceres-solver)
* 🟢 [pymoo](https://www.pymoo.org/) - Multi-objective Optimization in Python. [⭐ 3k](https://github.com/anyoptimization/pymoo)
* 🟢 [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic framework for algorithmic differentiation and numeric optimization. [⭐ 2.3k](https://github.com/casadi/casadi)
* 🟢 [NLopt](https://nlopt.readthedocs.io/en/latest/) - Nonlinear optimization. [⭐ 2.3k](https://github.com/stevengj/nlopt)
* 🟢 [OSQP](https://osqp.org/) - The Operator Splitting QP Solver. [⭐ 2.2k](https://github.com/osqp/osqp)
* 🟢 [Ipopt](https://projects.coin-or.org/Ipopt) - Large scale nonlinear optimization library. [⭐ 1.8k](https://github.com/coin-or/Ipopt)
* 🟢 [Pagmo](https://esa.github.io/pagmo2/index.html) - Scientific library for massively parallel optimization. [⭐ 937](https://github.com/esa/pagmo2)
* 🟡 [OptimLib](https://github.com/kthohr/optim) - Lightweight C++ library of numerical optimization methods for nonlinear functions. [⭐ 898](https://github.com/kthohr/optim)
* 🟢 ifopt - An Eigen-based, light-weight C++ Interface to Nonlinear Programming Solvers (Ipopt, Snopt). [⭐ 867](https://github.com/ethz-adrl/ifopt)
* 🟢 qpsolvers - Quadratic Programming solvers in Python with a unified API. [⭐ 760](https://github.com/qpsolvers/qpsolvers)
* 🟢 hpipm - High-performance interior-point-method QP solvers (Ipopt, Snopt). [⭐ 710](https://github.com/giaf/hpipm)
* 🟢 [Optimization Engine (OpEn)](https://alphaville.github.io/optimization-engine/) - Embedded nonconvex optimization with code generation for real-time control, with ROS support. [⭐ 651](https://github.com/alphaville/optimization-engine)
* 🟢 [SCS](http://web.stanford.edu/~boyd/papers/scs.html) - Numerical optimization for solving large-scale convex cone problems. [⭐ 638](https://github.com/cvxgrp/scs)
* 🟢 [ProxSuite](https://simple-robotics.github.io/proxsuite/) - The Advanced Proximal Optimization Toolbox. [⭐ 596](https://github.com/Simple-Robotics/ProxSuite)
* 🟢 libcmaes - Blackbox stochastic optimization using the CMA-ES algorithm. [⭐ 372](https://github.com/CMA-ES/libcmaes)
* 🟡 [limbo](http://www.resibots.eu/limbo/) - Gaussian processes and Bayesian optimization of black-box functions. [⭐ 270](https://github.com/resibots/limbo)
* 🟡 EXOTica - Generic optimisation toolset for robotics platforms. [⭐ 164](https://github.com/ipab-slmc/exotica)
* 🟢 SHOT - A solver for mixed-integer nonlinear optimization problems. [⭐ 135](https://github.com/coin-or/SHOT)
* 🟡 [RobOptim](http://roboptim.net/index.html) - Numerical Optimization for Robotics. [⭐ 66](https://github.com/roboptim/roboptim-core)
* 🟢 lpsolvers - Linear Programming solvers in Python with a unified API. [⭐ 26](https://github.com/stephane-caron/lpsolvers)
* 🟢 eigen-qld - Interface to use the QLD QP solver with the Eigen3 library. [⭐ 17](https://github.com/jrl-umi3218/eigen-qld)

<details>
<summary>Inactive (1): archived or no commits for 3+ years</summary>

* sferes2 - Evolutionary computation. [⭐ 170](https://github.com/sferes2/sferes2) · last commit 2021

</details>

## [Math & Geometry](#contents)

_Linear algebra, Lie groups, spatial transforms, and spatial data structures such as nearest-neighbor search._

* 🟢 [nanoflann](http://www.cs.ubc.ca/research/flann/) - Nearest Neighbor search with KD-trees. [⭐ 2.7k](https://github.com/jlblancoc/nanoflann)
* 🟡 Sophus - Lie groups using Eigen. [⭐ 2.5k](https://github.com/strasdat/Sophus)
* 🟢 manif - Small c++11 header-only library for Lie theory. [⭐ 1.8k](https://github.com/artivis/manif)
* 🟢 [HYPRE](https://hypre.readthedocs.io/) - Parallel solvers for sparse linear systems featuring multigrid methods. [⭐ 865](https://github.com/hypre-space/hypre)
* 🟡 Fastor - Light-weight high performance tensor algebra framework in C++11/14/17. [⭐ 844](https://github.com/romeric/Fastor)
* 🟢 spatialmath-python - Python classes for pose and orientation in 2D/3D with spatial operations toolbox. [⭐ 646](https://github.com/rai-opensource/spatialmath-python)
* 🟡 kindr - Kinematics and dynamics library for rigid body transformations. [⭐ 618](https://github.com/ANYbotics/kindr)
* 🟢 SpaceVelAlg - Spatial vector algebra with the Eigen3. [⭐ 81](https://github.com/jrl-umi3218/SpaceVecAlg)

<details>
<summary>Inactive (4): archived or no commits for 3+ years</summary>

* [FLANN](http://www.cs.ubc.ca/research/flann/) - Fast Library for Approximate Nearest Neighbors. [⭐ 2.4k](https://github.com/flann-lib/flann) · last commit 2022
* linalg.h - Single header public domain linear algebra library for C++11. [⭐ 968](https://github.com/sgorsten/linalg) · last commit 2023
* Octree - Fast radius neighbor search with an Octree. [⭐ 378](https://github.com/jbehley/octree) · last commit 2019
* [Cover-Tree](http://hunch.net/~jl/projects/cover_tree/cover_tree.html) - Cover tree data structure for quick k-nearest-neighbor search. [⭐ 64](https://github.com/DNCrane/Cover-Tree) · last commit 2017

</details>

## [SLAM & State Estimation](#contents)

_SLAM, odometry, factor graphs, filtering and sensor fusion, calibration, and mapping._

* 🟡 Cartographer - Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations. [⭐ 8k](https://github.com/cartographer-project/cartographer)
* 🟢 [RTAP-Map](http://introlab.github.io/rtabmap/) - RGB-D Graph SLAM approach based on a global Bayesian loop closure detector. [⭐ 4k](https://github.com/introlab/rtabmap)
* 🟢 GTSAM - Smoothing and mapping (SAM) in robotics and vision. [⭐ 3.7k](https://github.com/borglab/gtsam)
* 🟢 [mrpt](https://www.mrpt.org/) - Mobile Robot Programming Toolkit for SLAM, navigation, and computer vision. [⭐ 2.2k](https://github.com/MRPT/mrpt)
* 🟡 ElasticFusion - Real-time dense visual SLAM system. [⭐ 1.9k](https://github.com/mp3guy/ElasticFusion)
* 🟢 fuse - General architecture for performing sensor fusion live on a robot. [⭐ 882](https://github.com/locusrobotics/fuse)
* 🟢 [FusionCore](https://manankharwar.github.io/fusioncore/) - ROS 2 UKF fusing GPS, IMU, and wheel odometry with adaptive noise and GPS outlier rejection. [⭐ 377](https://github.com/manankharwar/fusioncore)
* 🟢 [fiducials](http://wiki.ros.org/fiducials) - Simultaneous localization and mapping using fiducial markers. [⭐ 285](https://github.com/UbiquityRobotics/fiducials)

### Mapping

* 🟢 [OctoMap](http://octomap.github.io/) - Efficient Probabilistic 3D Mapping Framework Based on Octrees. [⭐ 2.4k](https://github.com/OctoMap/octomap)
* 🟢 [libpointmatcher](http://libpointmatcher.readthedocs.io/en/latest/) - Iterative Closest Point library for 2-D/3-D mapping in Robotics. [⭐ 1.8k](https://github.com/norlab-ulaval/libpointmatcher)
* 🟢 Bonxai - Brutally fast, sparse, 3D Voxel Grid (formerly Treexy). [⭐ 884](https://github.com/facontidavide/Bonxai)
* 🟡 [wavemap](https://projects.asl.ethz.ch/wavemap/) - Fast, efficient and accurate multi-resolution, multi-sensor 3D occupancy mapping. [⭐ 573](https://github.com/ethz-asl/wavemap)

<details>
<summary>Inactive (7): archived or no commits for 3+ years</summary>

* ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras. [⭐ 10.2k](https://github.com/raulmur/ORB_SLAM2) · last commit 2017
* [LSD-SLAM](https://vision.in.tum.de/research/vslam/lsdslam) - Real-time monocular SLAM. [⭐ 2.7k](https://github.com/tum-vision/lsd_slam) · last commit 2014
* [DSO](https://vision.in.tum.de/research/vslam/dso) - Novel direct and sparse formulation for Visual Odometry. [⭐ 2.5k](https://github.com/JakobEngel/dso) · last commit 2021
* voxblox - Flexible voxel-based mapping focusing on truncated and Euclidean signed distance fields. [⭐ 1.7k](https://github.com/ethz-asl/voxblox) · last commit 2021
* Kintinuous - Real-time large scale dense visual SLAM system. [⭐ 953](https://github.com/mp3guy/Kintinuous) · last commit 2022
* AprilSAM - Real-time smoothing and mapping. [⭐ 239](https://github.com/xipengwang/AprilSAM) · last commit 2021
* [SRBA](http://mrpt.github.io/srba/) - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies. [⭐ 75](https://github.com/MRPT/srba) · last commit 2016

</details>

## [Perception](#contents)

_Image and point-cloud processing, detection, pose estimation and tracking, and visual servoing._

* 🟢 [PCL](https://pointclouds.org/) - 2D/3D image and point cloud processing. [⭐ 11.2k](https://github.com/PointCloudLibrary/pcl)
* 🟢 [ViSP](http://visp.inria.fr/) - Visual Servoing Platform. [⭐ 928](https://github.com/lagadic/visp)
* 🟢 [BundleTrack](https://github.com/wenbowen123/BundleTrack) - 6D Pose Tracking for Novel Objects without 3D Models. [⭐ 687](https://github.com/wenbowen123/BundleTrack)

<details>
<summary>Inactive (1): archived or no commits for 3+ years</summary>

* [se(3)-TrackNet](https://github.com/wenbowen123/iros20-6d-pose-tracking) - 6D Pose Tracking for Novel Objects without 3D Models. [⭐ 424](https://github.com/wenbowen123/iros20-6d-pose-tracking) · last commit 2023

</details>

## [Grasping & Manipulation](#contents)

_Grasp synthesis, detection, and planning, and dexterous manipulation._

* 🟢 [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) - SDK for AnyGrasp, a 6-DoF grasp pose detection method. [⭐ 1k](https://github.com/graspnet/anygrasp_sdk)
* 🟡 [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) - 6-DoF grasp generation for parallel-jaw grippers using contact maps. [⭐ 538](https://github.com/NVlabs/contact_graspnet)
* 🟢 [GraspNet API](https://github.com/graspnet/graspnetAPI) - Python API and evaluation tools for the GraspNet benchmark. [⭐ 368](https://github.com/graspnet/graspnetAPI)

<details>
<summary>Inactive (1): archived or no commits for 3+ years</summary>

* [GraspIt!](https://graspit-simulator.github.io/) - Simulator for grasping research that can accommodate arbitrary hand and robot designs. [⭐ 221](https://github.com/graspit-simulator/graspit) · last commit 2020

</details>

## [Robot Learning](#contents)

_Robot foundation models (VLA), imitation and reinforcement learning libraries, learning environments, and benchmarks._

* 🟢 LeRobot - Pretrained models, datasets, and simulation environments for real-world robotics in PyTorch. [⭐ 28.1k](https://github.com/huggingface/lerobot)
* 🟢 [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) - Reliable implementations of reinforcement learning algorithms in PyTorch. [⭐ 13.9k](https://github.com/DLR-RM/stable-baselines3)
* 🟢 [Gymnasium](https://gymnasium.farama.org/) - Developing and comparing reinforcement learning algorithms. [⭐ 12.7k](https://github.com/Farama-Foundation/Gymnasium)
* 🟢 [CleanRL](https://github.com/vwxyzjn/cleanrl) - Single-file implementations of deep reinforcement learning algorithms. [⭐ 10.5k](https://github.com/vwxyzjn/cleanrl)
* 🟢 [Isaac Lab](https://isaac-sim.github.io/IsaacLab) - GPU-accelerated open-source framework for robot learning built on NVIDIA Isaac Sim. [⭐ 8.3k](https://github.com/isaac-sim/IsaacLab)
* 🟢 [ManiSkill](https://github.com/haosulab/ManiSkill) - Robot simulation and manipulation learning package powered by SAPIEN. [⭐ 3.4k](https://github.com/mani-skill/ManiSkill)
* 🟡 [Legged Gym](https://github.com/leggedrobotics/legged_gym) - Isaac Gym environments for legged robot locomotion training. [⭐ 3.1k](https://github.com/leggedrobotics/legged_gym)
* 🟢 [robosuite](https://robosuite.ai/docs/) - A modular simulation framework and benchmark for robot learning. [⭐ 2.6k](https://github.com/ARISE-Initiative/robosuite)
* 🟡 [Humanoid-Gym](https://github.com/roboterax/humanoid-gym) - Reinforcement learning environment for humanoid robot locomotion. [⭐ 2.1k](https://github.com/roboterax/humanoid-gym)
* 🟢 [rl_games](https://github.com/Denys88/rl_games) - High-performance RL library used in Isaac Gym environments. [⭐ 1.4k](https://github.com/Denys88/rl_games)
* 🟢 [SKRL](https://github.com/Toni-SM/skrl) - Modular reinforcement learning library with support for multiple ML frameworks. [⭐ 1.1k](https://github.com/Toni-SM/skrl)
* 🟡 [AllenAct](https://allenact.org/) - Python/PyTorch-based Research Framework for Embodied AI. [⭐ 384](https://github.com/allenai/allenact)

### General Machine Learning

* 🟢 [mlpack](https://www.mlpack.org/) - Scalable C++ machine learning library. [⭐ 5.7k](https://github.com/mlpack/mlpack)
* 🟡 [DyNet](https://dynet.readthedocs.io/en/latest/) - The Dynamic Neural Network Toolkit. [⭐ 3.4k](https://github.com/clab/dynet)
* 🟡 DLL - Deep Learning Library (DLL) for C++. [⭐ 688](https://github.com/wichtounet/dll)

<details>
<summary>Inactive (7): archived or no commits for 3+ years</summary>

* 💀 [Ivy](https://lets-unify.ai/) - Unified Machine Learning Framework. [⭐ 14.2k](https://github.com/unifyai/ivy) · last commit 2026
* [tiny-dnn](http://tiny-dnn.readthedocs.io/en/latest/) - Header only, dependency-free deep learning framework in C++14. [⭐ 6k](https://github.com/tiny-dnn/tiny-dnn) · last commit 2018
* 💀 gym-gazebo - OpenAI Gym environments for the Gazebo simulator. [⭐ 845](https://github.com/erlerobot/gym-gazebo) · last commit 2019
* [Fido](http://fidoproject.github.io/) - Lightweight C++ machine learning library for embedded electronics and robotics. [⭐ 464](https://github.com/FidoProject/Fido) · last commit 2017
* MiniDNN - A header-only C++ library for deep neural networks. [⭐ 434](https://github.com/yixuan/MiniDNN) · last commit 2021
* RLLib - Temporal-difference learning algorithms in reinforcement learning. [⭐ 211](https://github.com/samindaa/RLLib) · last commit 2016
* gym-dart - OpenAI Gym environments using the DART physics engine. [⭐ 140](https://github.com/DartEnv/dart-env) · last commit 2020

</details>

## [Robot Data & Visualization](#contents)

_Logging, storing, converting, and visualizing robot data, dataset tools, and teleoperation for data collection._

* 🟢 Any4LeRobot - A collection of utilities and tools for LeRobot. [⭐ 1.2k](https://github.com/Tavish9/any4lerobot)
* 🟢 [ReductStore](https://www.reduct.store) - High-performance, time-indexed object storage for robotics and industrial IoT. [⭐ 373](https://github.com/reductstore/reductstore)
* 🟢 [LeRobot Episode Scoring Toolkit](https://github.com/RoboticsData/score_lerobot_episodes) - One-click tool to score, filter, and export higher-quality LeRobot datasets. [⭐ 77](https://github.com/RoboticsData/score_lerobot_episodes)
* [Foxglove Studio](https://foxglove.dev) - A fully integrated visualization and debugging desktop app for your robotics data.

<details>
<summary>Inactive (1): archived or no commits for 3+ years</summary>

* [Robot Gui](http://robot.glumb.de/) - A three.js based 3D robot interface. [⭐ 391](https://github.com/glumb/robot-gui) · last commit 2020

</details>

## [Other Awesome Lists](#contents)

_Related curated lists and collections._

* 🟢 [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning) - Curated list of machine learning frameworks, libraries, and software. [⭐ 74.6k](https://github.com/josephmisiti/awesome-machine-learning)
* 🟢 [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python sample codes for robotics algorithms. [⭐ 30.7k](https://github.com/AtsushiSakai/PythonRobotics)
* 🟡 [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning) - Curated list of deep learning tutorials, projects, and communities. [⭐ 29k](https://github.com/ChristosChristofidis/awesome-deep-learning)
* 🟢 [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence) - Curated list of AI courses, books, video lectures, and papers. [⭐ 16.6k](https://github.com/owainlewis/awesome-artificial-intelligence)
* 🟡 [Awesome Robotics (Kiloreux)](https://github.com/Kiloreux/awesome-robotics) - Curated list of robotics resources (Kiloreux). [⭐ 7.2k](https://github.com/Kiloreux/awesome-robotics)
* 🟢 [Robotics Coursework](https://github.com/mithi/robotics-coursework) - A list of robotics courses you can take online. [⭐ 5.3k](https://github.com/mithi/robotics-coursework)
* 🟡 [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets) - Curated list of SLAM-related datasets. [⭐ 1.9k](https://github.com/youngguncho/awesome-slam-datasets)
* 🟢 [Awesome Collision Detection](https://github.com/jslee02/awesome-collision-detection) - Curated list of collision detection libraries and resources. [⭐ 1k](https://github.com/jslee02/awesome-collision-detection)
* 🟢 [Awesome Physical AI](https://github.com/keon/awesome-physical-ai) - Curated list of academic papers and resources on Physical AI. [⭐ 439](https://github.com/keon/awesome-physical-ai)
* 🟢 [Awesome VLA Study](https://github.com/MilkClouds/awesome-vla-study) - Study guide to vision-language-action (VLA) papers for robot learning. [⭐ 402](https://github.com/MilkClouds/awesome-vla-study)

<details>
<summary>Inactive (6): archived or no commits for 3+ years</summary>

* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision) - Curated list of computer vision resources. [⭐ 23.6k](https://github.com/jbhuang0604/awesome-computer-vision) · last commit 2021
* [Awesome Robotic Tooling](https://github.com/Ly0n/awesome-robotic-tooling) - Tooling for professional robotic development in C++ and Python with ROS and autonomous driving. [⭐ 3.9k](https://github.com/Ly0n/awesome-robotic-tooling) · last commit 2023
* [Awesome Robotics (ahundt)](https://github.com/ahundt/awesome-robotics) - Curated list of robotics links and software libraries (ahundt). [⭐ 1.5k](https://github.com/ahundt/awesome-robotics) · last commit 2023
* 💀 [Awesome Gazebo](https://github.com/fkromer/awesome-gazebo) - Curated list of Gazebo simulation framework resources. [⭐ 155](https://github.com/fkromer/awesome-gazebo) · last commit 2021
* [Awesome Human Robot Interaction](https://github.com/Po-Jen/awesome-human-robot-interaction) - Curated list of human-robot interaction libraries and resources. [⭐ 153](https://github.com/Po-Jen/awesome-human-robot-interaction) · last commit 2019
* [Awesome Grasping](https://github.com/Po-Jen/awesome-grasping) - Curated list of grasping libraries and resources. [⭐ 99](https://github.com/Po-Jen/awesome-grasping) · last commit 2019

</details>

## [Contributing](#contents)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [Star History](#contents)

<a href="https://www.star-history.com/?repos=jslee02%2Fawesome-robotics-libraries">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&theme=dark&legend=top-left" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&legend=top-left" />
    <img alt="Star History Chart" src="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&legend=top-left" />
  </picture>
</a>

## [License](#contents)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
