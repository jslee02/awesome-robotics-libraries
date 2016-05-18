# Awesome Robotics Libraries

A curated list of open source robotics libraries and software.

#### Table of Contents
* [Libraries](#libraries)
  * [Multibody Dynamics](#multibody-dynamics)
    * [Full List of Libraries](#full-list-of-libraries)
    * [Selected Libraries to Compare](#selected-libraries-to-compare)
    * [Comparison](#comparison)
  * [Motion Planning](#motion-planning)
  * [Collision Detection](#collision-detection)
  * [Optimization](#optimization)
  * [Robot Model Description Format](#robot-model-description-format)
  * [Robot Platform](#robot-platform)
* [Software](#software)
  * [Simulator](#simulator)
* [Other Awesome Lists](#other-awesome-lists)
* [Contributing](#contributing)

## [Libraries](#awesome-robotics-libraries)

### [Multibody Dynamics](#awesome-robotics-libraries)

#### [Full List of Libraries](#awesome-robotics-libraries)

##### C++

* [Bullet](http://bulletphysics.org/wordpress/) ([github](https://github.com/bulletphysics/bullet3)) - Real-Time Physics Simulation
* [CHRONO::ENGINE](http://chronoengine.info/) ([github](https://github.com/projectchrono/chrono)) - C++ library for multibody dynamics simulations.
* [DART](http://dartsim.github.io/) ([github](https://github.com/dartsim/dart.git)) - Dynamic Animation and Robotics Toolkit.
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) ([github](https://github.com/RobotLocomotion/drake)) - A planning, control, and analysis toolbox for nonlinear dynamical systems.
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - A library for dynamic simulation of multi-body systems in C++.
* [KDL](http://www.orocos.org/kdl) ([github](https://github.com/orocos/orocos_kinematics_dynamics)) - Orocos Kinematics and Dynamics C++ library.
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

##### Python

* [PyDy](http://www.pydy.org/) ([github](https://github.com/pydy/pydy)) - A tool kit written in the Python programming language that utilizes an array of scientific programs to enable the study of multibody dynamics.

#### [Selected Libraries to Compare](#awesome-robotics-libraries)

* [Bullet](http://bulletphysics.org/wordpress/) ([github](https://github.com/bulletphysics/bullet3)) - Real-Time Physics Simulation
* [CHRONO::ENGINE](http://chronoengine.info/) ([github](https://github.com/projectchrono/chrono)) - C++ library for multibody dynamics simulations.
* [DART](http://dartsim.github.io/) ([github](https://github.com/dartsim/dart.git)) - Dynamic Animation and Robotics Toolkit.
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) ([github](https://github.com/RobotLocomotion/drake)) - A planning, control, and analysis toolbox for nonlinear dynamical systems.
* [ODE](http://www.ode.org/) ([bitbucket](https://bitbucket.org/odedevs/ode)) - An open source, high performance library for simulating rigid body dynamics.
* [Simbody](https://simtk.org/home/simbody/) ([github](https://github.com/simbody/simbody.git)) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems.

#### [Comparison](#awesome-robotics-libraries)

Comparisons inspired by [Wikipedia's Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators) page. 
> :warning: **Warning**: Most fields are missing. Any help would be appreciated.

##### General Information

| Library  | Developer(s) | Latest Release | Platform Supported | License | Github :star: |
|:--------:| ------------ | -------------- | ------------------ | ------- |:------------:|
| chrono   | [University of Wisconsin-Madison](http://www.projectchrono.org/about/) | IntPoint1.2 (2016-02-27) | Linux, Windows | BSD 3-Clause | 57 |
| Bullet   | Erwin Coumans | 2.83 (2016-01-08) | Linux, MacOS, Windows | Zlib | 1644 |
| DART     | Georgia Tech and CMU | [6.0.0 (2016-05-10)](https://github.com/dartsim/dart/releases/tag/v6.0.0) | Linux, MacOS, Windows | BSD 2-Clause | [90](https://github.com/dartsim/dart/stargazers) |
| Drake    | MIT and TRI | 0.9.11 (2015-10-08) | Linux, MacOS, Windows | BSD 3-Clause | 443 |
| ODE      | [Russell Smith](http://www.q12.org/) | [0.14 (2015-12-18)](https://bitbucket.org/odedevs/ode/commits/tag/0.14) | Linux, MacOS, Windows | LGPL 2.1 or BSD 3-Clause | N/A |
| Simbody  | [Simtk.org](https://simtk.org/xml/index.xml) | 3.5.3 (2015-06-15) | Linux, MacOS, Windows | Apache 2.0 | 535 |
:star: updated Apr 19, 2016

##### Technical Information

| Library  | Main Programming Language | External APIs | File Formats Support |
|:--------:|:-------------------------:|:-------------:|:--------------------:|
| chrono   | C++ | C++, Python | unknown |
| Bullet   | C++ | C++ | URDF |
| DART     | C++ | C++, Python | URDF, SDF, VSK, SKEL |
| Drake    | C++ and MATLAB | C++, Python | own json format |
| ODE      | C++ | C++ | unknown |
| Simbody  | C++ | C++ | URDF |

##### Support

| Library  | Mailing List | API Documentation | Public Forum/Help System | User Manual | Tutorial | Issue Tracker | Wiki |
|:--------:|:------------:|:-----------------:|:------------------------:|:-----------:|:--------:|:-------------:|:----:|
| DART    | No | [Yes](http://dartsim.github.io/dart/) | No | No | [Yes](http://dart.readthedocs.org/en/release-5.1/) | [Yes](https://github.com/dartsim/dart/issues) | [Yes](https://github.com/dartsim/dart/wiki) |

##### Code Quality

| Library  | Lines of Code | Lines of Comments | Coverage | Continuous Integration | Static Code Checker | Style Checker |
|:--------:|:-------------:|:-----------------:|:--------:|:----------------------:|:-------------------:|:-------------:|
| DART    | [75k](https://github.com/dartsim/dart/wiki/Project-Status) | [34k](https://github.com/dartsim/dart/wiki/Project-Status)  | [51%](https://coveralls.io/github/dartsim/dart) | Travis CI, Appveyor | None | None |

##### Simulation Features

###### Families of Robots

| Library  | UGV (ground mobile robot) | UAV (aerial robots) | AUV (underwater robots) | Robotic Arms | Robotic Hands (grasping simulation) | Humanoid Robots | Human Avatars |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|
| DART     | Yes | No | No | Yes | Yes | Yes | No |

###### Supported Kinematics/Dynamics Algorithms

| Library  | Inverse Kinematics | Inverse Dynamics | Forward Dynamics | Hybrid Dynamics |
|:--------:|:------------------:|:----------------:|:----------------:|:---------------:|
| DART     | Yes | Yes | Yes  | Yes |

###### Supported Joint Types

| Library  | Revolute | Prismatic | Screw | Universal | [Cylindrical](https://en.wikipedia.org/wiki/Cylindrical_joint) | Ball | Euler | Translational | Planar | Free |
|:--------:|:--------:|:---------:|:-----:|:---------:|:------------:|:----:|:-----:|:-------------:|:------:|:----:|
| DART     | Yes | Yes  | Yes | Yes | [No](https://github.com/dartsim/dart/issues/721) | Yes | Yes | Yes | Yes | Yes |

###### Supported Actuators

| Library  | Kinematic Actuators | Force-controlled Actuators | Servos |
|:--------:|:-------------------:|:--------------------------:|:------:|
| DART     | Yes | Yes  | Yes |

###### Supported Sensors

| Library  | Odometry | IMU | Collision | GPS | Monocular Cameras | Stereo Cameras | Depth Cameras | 2D Laser Scanners | 3D Laser Scanners |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|:------------:|:----------:|
| DART     | No | No | No | No | No | No | No | No | No |

###### Collision Detection Features

| Library | Collision Detectors |
|:-------:|:--------------------|
| DART    | Builtin (supports only sphere, box), fcl, bullet |

#### Benchmark

##### Problems

* [IFToMM](http://iftomm-multibody.org/benchmark/) - A tool for the international multibody dynamics community to propose, solve, and refer to a collection of benchmark problems.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) - Benchmark Problems for Multibody Dynamics (BPMD) Database.

##### Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015. ([pdf](https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf))
* Y. Lu et al. [Comparison of Multibody Dynamics Solver Performance: Synthetic versus Realistic Data](http://proceedings.asmedigitalcollection.asme.org/proceeding.aspx?articleid=2483796). ASME IDETC/CIEC 2015. ([pdf](http://www.cs.rpi.edu/foswiki/pub/RoboticsWeb/LabPublications/LTasme2015.pdf))
* S. Ivaldi et al. [Tools for simulating humanoid robot dynamics: a survey based on user feedback](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7041462). Humanoids 2014. ([pdf](http://arxiv.org/pdf/1402.7050.pdf))

##### Articles

* [9 open source robotics projects](https://opensource.com/life/16/4/open-source-robotics-projects) by [Jason Baker](https://opensource.com/users/jason-baker)
* [Comparison of Rigid Body Dynamic Simulators for Robotic Simulation in Gazebo](http://www.osrfoundation.org/wordpress2/wp-content/uploads/2015/04/roscon2014_scpeters.pdf) by [Steven Peters](http://www.osrfoundation.org/team/steven-peters/) and [John Hsu](http://www.osrfoundation.org/team/john-hsu/)
* [Survey of multibody dynamics software](http://homepages.rpi.edu/~luy4/survey.html) maintained by [Ying Lu](http://homepages.rpi.edu/~luy4/)
* [Wikipedia: Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators)

### [Motion Planning](#awesome-robotics-libraries)

* [Aikido](https://github.com/personalrobotics/aikido) ([github](https://github.com/personalrobotics/aikido)) - A C++ library for solving robotic motion planning and decision making problems.
* [OMPL](http://ompl.kavrakilab.org/) ([bitbucket](https://bitbucket.org/ompl/ompl), [github](https://github.com/ompl/ompl)) - The Open Motion Planning Library.

### [Collision Detection](#awesome-robotics-libraries)

* [ColDet](https://sourceforge.net/projects/coldet/) - A collision detection library for generic polyhedra.
* [FCL](https://github.com/flexible-collision-library/fcl) ([github](https://github.com/flexible-collision-library/fcl)) - Flexible Collision Library.
* [libccd](https://github.com/danfis/libccd) ([github](https://github.com/danfis/libccd)) - Library for collision detection between two convex shapes.
* [OZCollide](http://www.tsarevitch.org/ozcollide/) - A fast, complete and free collision detection library.

> Note that some multibody dynamics libraries (e.g., ODE and Bullet) can be used only for collision detection.

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

#### Comparison

##### General Information

| Library  | Developer(s) | Latest Release | Platform Supported | License | Github :star: |
|:--------:| ------------ | -------------- | ------------------ | ------- |:------------:|
| Gazebo   | [OSRF](http://www.osrfoundation.org/) | [7.1.0 (2016-04-08)](http://www.gazebosim.org/download) | Linux, Mac, Windows | Apache 2.0 | N/A |
| MORSE    | [LAAS-CNRS and many](http://www.openrobots.org/morse/doc/latest/credits.html) | 1.4 (2016-02-08) | Linux | BSD 3-Clause | 153 |
| V-REP    | [Coppelia Robotics](http://www.coppeliarobotics.com/contact.html) | [3.3.0 (2016-02-19)](http://www.coppeliarobotics.com/downloads.html) | Linux, Mac, Windows | [Commercial / Free educational](http://www.coppeliarobotics.com/licensing.html) | N/A |
:star: updated Apr 19, 2016

##### Papers

* A. Staranowicz, G. L. Mariottini, [A Survey and Comparison of Commercial and
Open-Source Robotic Simulator Software](http://dl.acm.org/citation.cfm?id=2141689), PETRA 2011. ([pdf](http://ranger.uta.edu/~gianluca/papers/StMa_PETRA11.pdf))

## [Other Awesome Lists](#awesome-robotics-libraries)

* [Awesome Robotics](https://github.com/Kiloreux/awesome-robotics) - This is a list of various books, courses and other resources for robotics. It's an attempt to gather useful material in one place for everybody who wants to learn more about the field.
* [Awesome Artificial Intelligence](https://github.com/owainlewis/awesome-artificial-intelligence)
* [Awesome Computer Vision](https://github.com/jbhuang0604/awesome-computer-vision)
* [Awesome Machine Learning](https://github.com/josephmisiti/awesome-machine-learning)
* [Awesome Deep Learning](https://github.com/ChristosChristofidis/awesome-deep-learning)

## [Contributing](#awesome-robotics-libraries)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#awesome-robotics-libraries)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
