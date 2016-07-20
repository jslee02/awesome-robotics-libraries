# Awesome Robotics Libraries

A curated list of open source robotics libraries and software. 

#### Table of Contents
* [Libraries](#libraries)
  * [Multibody Dynamics](#multibody-dynamics)
  * [Motion Planning](#motion-planning)
  * [Optimization](#optimization)
  * [Robot Model Description Format](#robot-model-description-format)
  * [Robot Platform](#robot-platform)
* [Software](#software)
  * [Simulator](#simulator)
* [Comparisons](#comparisons)
* [Other Awesome Lists](#other-awesome-lists)
* [Contributing](#contributing)

## [Libraries](#awesome-robotics-libraries)

### [Multibody Dynamics](#awesome-robotics-libraries)

#### C++

* [Bullet](http://bulletphysics.org/wordpress/) ([github](https://github.com/bulletphysics/bullet3)) - Real-Time Physics Simulation
* [CHRONO::ENGINE](http://chronoengine.info/) ([github](https://github.com/projectchrono/chrono)) - C++ library for multibody dynamics simulations.
* [DART](http://dartsim.github.io/) ([github](https://github.com/dartsim/dart.git)) - Dynamic Animation and Robotics Toolkit.
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) ([github](https://github.com/RobotLocomotion/drake)) - A planning, control, and analysis toolbox for nonlinear dynamical systems.
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - A library for dynamic simulation of multi-body systems in C++.
* [KDL](http://www.orocos.org/kdl) ([github](https://github.com/orocos/orocos_kinematics_dynamics)) - Orocos Kinematics and Dynamics C++ library.
* [Klampt](http://motion.pratt.duke.edu/klampt/) ([github](https://github.com/krishauser/Klampt)) - Kris' Locomotion and Manipulation Planning Toolkit
* [MBDyn](https://www.mbdyn.org/) - Free MultiBody Dynamics Simulation Software
* [MBSlib](http://www.sim.informatik.tu-darmstadt.de/res/sw/mbslib) ([github](https://github.com/SIM-TU-Darmstadt/mbslib), [paper](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7403876)) - An efficient and modular multibody systems library for kinematics and dynamics simulation, optimization and sensitivity analysis.
* [metapod](https://github.com/laas/metapod) ([github](https://github.com/laas/metapod)) - A template-based robot dynamics library.
* [Moby](http://physsim.sourceforge.net/index.html) ([github](https://github.com/PositronicsLab/Moby)) - Multi-body dynamics simulation library written in C++.
* [mrpt](http://www.mrpt.org/) ([github](https://github.com/MRPT/mrpt)) - The Mobile Robot Programming Toolkit.
* [MuJoCo](http://www.mujoco.org/index.html) (closed source) - A physics engine aiming to facilitate research and development in robotics, biomechanics, graphics and animation, and other areas where fast and accurate simulation is needed.
* [Newton Dynamics](http://newtondynamics.com/) ([github](https://github.com/MADEAPPS/newton-dynamics)) - A cross-platform life-like physics simulation library.
* [ODE](http://www.ode.org/) ([bitbucket](https://bitbucket.org/odedevs/ode)) - An open source, high performance library for simulating rigid body dynamics.
* [OpenRAVE](http://www.openrave.org) ([github](https://github.com/rdiankov/openrave)) - An environment for testing, developing, and deploying robotics motion planning algorithms.
* [pinocchio](http://stack-of-tasks.github.io/pinocchio/) ([github](https://github.com/stack-of-tasks/pinocchio)) - Dynamic computations using Spatial Algebra.
* [PositionBasedDynamics](https://github.com/janbender/PositionBasedDynamics) - A library for the physically-based simulation of rigid bodies, deformable solids and fluids.
* [RBDL](http://rbdl.bitbucket.org/) ([bitbucket](https://bitbucket.org/rbdl/rbdl)) - Rigid Body Dynamics Library.
* [ReactPhysics3d](http://www.reactphysics3d.com/) ([github](https://github.com/DanielChappuis/reactphysics3d)) - An open source C++ physics engine library that can be used in 3D simulations and games.
* [Robotics Library](http://www.roboticslibrary.org/) ([github](https://github.com/roboticslibrary/rl)) - A self-contained C++ library for robot kinematics, motion planning and control.
* [RobWork](http://www.robwork.dk/apidoc/nightly/rw/index.html) - A collection of C++ libraries for simulation and control of robot systems.
* [siconos](http://siconos.gforge.inria.fr) ([github](https://github.com/siconos/siconos)) - A software package for the modeling and simulation of nonsmooth dynamical systems.
* [Simbody](https://simtk.org/home/simbody/) ([github](https://github.com/simbody/simbody.git)) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems.
* [SOFA](https://www.sofa-framework.org/) ([github](https://github.com/sofa-framework/sofa)) - An Open Source framework primarily targeted at real-time simulation, with an emphasis on medical simulation.
* qu3e ([github](https://github.com/RandyGaul/qu3e)) - Lightweight and Simple 3D Open Source Physics Engine in C++.

#### Python

* [PyDy](http://www.pydy.org/) ([github](https://github.com/pydy/pydy)) - A tool kit written in the Python programming language that utilizes an array of scientific programs to enable the study of multibody dynamics.
* [trep](http://murpheylab.github.io/trep/) ([github](https://github.com/MurpheyLab/trep)) - Python module for modeling articulated rigid body mechanical systems in generalized coordinates.

### [Motion Planning](#awesome-robotics-libraries)

* [Aikido](https://github.com/personalrobotics/aikido) ([github](https://github.com/personalrobotics/aikido)) - A C++ library for solving robotic motion planning and decision making problems.
* [OMPL](http://ompl.kavrakilab.org/) ([bitbucket](https://bitbucket.org/ompl/ompl), [github](https://github.com/ompl/ompl)) - The Open Motion Planning Library.

### [Optimization](#awesome-robotics-libraries)

* [Ceres Solver](http://ceres-solver.org/) ([github](https://github.com/ceres-solver/ceres-solver)) - A large scale non-linear optimization library . Ceres Solver has been used in production at Google for more than four years now. It is clean, extensively tested and well documented code that is actively developed and supported.
* [Ipopt](https://projects.coin-or.org/Ipopt) ([github](https://github.com/coin-or/Ipopt)) - Ipopt (Interior Point OPTimizer, pronounced eye-pea-Opt) is a software package for large-scale ​nonlinear optimization.
* [NLopt](http://ab-initio.mit.edu/wiki/index.php/NLopt) ([github](https://github.com/stevengj/nlopt)) - NLopt is a free/open-source library for nonlinear optimization, providing a common interface for a number of different free optimization routines available online as well as original implementations of various other algorithms.

### [Robot Model Description Format](#awesome-robotics-libraries)

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. ([bitbucket](https://bitbucket.org/osrf/sdformat))
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. ([github](https://github.com/ros/urdfdom))

### [Robot Platform](#awesome-robotics-libraries)

* [ROS](http://www.ros.org/) ([github repos](http://wiki.ros.org/Tickets)) - A set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project.
* [YARP](http://www.yarp.it/) ([github](https://github.com/robotology/yarp)) - A library and toolkit for communication and device interfaces, used on everything from humanoids to embedded devices.

## [Software](#awesome-robotics-libraries)

### [Simulator](#awesome-robotics-libraries)

* [Gazebo](http://www.gazebosim.org/) ([bitbucket](https://bitbucket.org/osrf/gazebo)) - A dynamic multi-robot simulator.
* [GraspIt!](http://graspit-simulator.github.io/) ([github](https://github.com/graspit-simulator/graspit)) - A simulator for grasping research that can accommodate arbitrary hand and robot designs developed by [the Columbia University Robotics Group](http://www.cs.columbia.edu/robotics/)
* [MORSE](http://morse-simulator.github.io/) ([github](https://github.com/morse-simulator/morse)) - The Modular OpenRobots Simulation Engine.
* [V-REP](http://www.coppeliarobotics.com/) - Virtual robot experimentation platform.

## [Comparisons](#awesome-robotics-libraries)

The comparisons are moved to [COMPARISONS.md](https://github.com/jslee02/awesome-robotics-libraries/blob/master/COMPARISONS.md).

## [Other Awesome Lists](#awesome-robotics-libraries)

* [Awesome Robotics](https://github.com/Kiloreux/awesome-robotics) - This is a list of various books, courses and other resources for robotics. It's an attempt to gather useful material in one place for everybody who wants to learn more about the field.
* [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence)
* [Awesome Collision Detection](https://github.com/jslee02/awesome-collision-detection)
* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
* [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning)
* [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning)

## [Contributing](#awesome-robotics-libraries)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#awesome-robotics-libraries)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
