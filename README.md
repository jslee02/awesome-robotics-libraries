# Awesome Robotics Libraries

## Multibody Dynamics

* [Bullet](http://bulletphysics.org/wordpress/) - Real-Time Physics Simulation ([github](https://github.com/bulletphysics/bullet3))
* [CHRONO::ENGINE](http://chronoengine.info/) - C++ library for multibody dynamics simulations. ([github](https://github.com/projectchrono/chrono))
* [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit. ([github](https://github.com/dartsim/dart.git))
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) - A planning, control, and analysis toolbox for nonlinear dynamical systems. ([github](https://github.com/RobotLocomotion/drake))
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - A library for dynamic simulation of multi-body systems in C++.
* [KDL](http://www.orocos.org/kdl) - Orocos Kinematics and Dynamics C++ library. ([github](https://github.com/orocos/orocos_kinematics_dynamics))
* [MBDyn](https://www.mbdyn.org/) - Free MultiBody Dynamics Simulation Software
* [metapod](https://github.com/laas/metapod) - A template-based robot dynamics library. ([github](https://github.com/laas/metapod))
* [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation library written in C++. ([github](https://github.com/PositronicsLab/Moby))
* [mrpt](http://www.mrpt.org/) - The Mobile Robot Programming Toolkit. ([github](https://github.com/MRPT/mrpt))
* [Newton Dynamics](http://newtondynamics.com/) - A cross-platform life-like physics simulation library. ([github](https://github.com/MADEAPPS/newton-dynamics))
* [ODE](http://www.ode.org/) - An open source, high performance library for simulating rigid body dynamics. ([bitbucket](https://bitbucket.org/odedevs/ode))
* [OpenRAVE](http://www.openrave.org) - An environment for testing, developing, and deploying robotics motion planning algorithms. ([github](https://github.com/rdiankov/openrave))
* [pinocchio](http://stack-of-tasks.github.io/pinocchio/) - Dynamic computations using Spatial Algebra. ([github](https://github.com/stack-of-tasks/pinocchio))
* [PositionBasedDynamics](https://github.com/janbender/PositionBasedDynamics) - A library for the physically-based simulation of rigid bodies, deformable solids and fluids.
* [RBDL](http://rbdl.bitbucket.org/) - Rigid Body Dynamics Library. ([bitbucket](https://bitbucket.org/rbdl/rbdl))
* [Robotics Library](http://www.roboticslibrary.org/) - A self-contained C++ library for robot kinematics, motion planning and control. ([github](https://github.com/roboticslibrary/rl))
* [RobWork](http://www.robwork.dk/apidoc/nightly/rw/index.html) - A collection of C++ libraries for simulation and control of robot systems.
* [siconos](http://siconos.gforge.inria.fr) - A software package for the modeling and simulation of nonsmooth dynamical systems. ([github](https://github.com/siconos/siconos))
* [Simbody](https://simtk.org/home/simbody/) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems. ([github](https://github.com/simbody/simbody.git))
* [yarp](http://www.yarp.it/) - Yet Another Robot Platform. ([github](https://github.com/robotology/yarp))

### Comparison

Comparisons inspired by [Wikipedia's Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators) page. 
> :warning: **Warning**: Most fields are missing. Any help would be appreciated.

#### General Information

| Library  | Developer(s) | Latest Release | Platform Supported | License | Github :star: |
|:--------:| ------------ | -------------- | ------------------ | ------- |:------------:|
| chrono   | [University of Wisconsin-Madison](http://www.projectchrono.org/about/) | IntPoint1.2 (2016-02-27) | Linux, Windows | BSD 3-Clause | 57 |
| Bullet   | Erwin Coumans | 2.83 (2016-01-08) | Linux, MacOS, Windows | Zlib | 1644 |
| DART     | Georgia Tech | 5.1.1 (2015-11-06) | Linux, MacOS, Windows | BSD 2-Clause | 90 |
| Drake    | MIT and TRI | 0.9.11 (2015-10-08) | Linux, MacOS, Windows | BSD 3-Clause | 443 |
| MORSE    | [LAAS-CNRS and many](http://www.openrobots.org/morse/doc/latest/credits.html) | 1.4 (2016-02-08) | Linux | BSD 3-Clause | 153 |
| Simbody  | [Simtk.org](https://simtk.org/xml/index.xml) | 3.5.3 (2015-06-15) | Linux, MacOS, Windows | Apache 2.0 | 535 |
:star: updated Apr 19, 2016

#### Technical Information

| Library  | Main Programming Language | External APIs | Formats Support |
|:--------:|:-------------------------:|:-------------:|:---------------:|
| DART     | C++ | C++, Python | URDF, SDF, VSK, SKEL |

#### Support

| Library  | Mailing List | API Documentation | Public Forum/Help System | User Manual | Tutorial | Issue Tracker | Wiki |
|:--------:|:------------:|:-----------------:|:------------------------:|:-----------:|:--------:|:-------------:|:----:|
| DART    | No | [Yes](http://dartsim.github.io/dart/) | No | No | [Yes](http://dart.readthedocs.org/en/release-5.1/) | [Yes](https://github.com/dartsim/dart/issues) | [Yes](https://github.com/dartsim/dart/wiki) |

#### Code Quality

| Library  | Lines of Code | Lines of Comments | Coverage | Continuous Integration | Static Code Checker | Style Checker |
|:--------:|:-------------:|:-----------------:|:--------:|:----------------------:|:-------------------:|:-------------:|
| DART    | 58k | 26k  | Unknown | Travis CI, Appveyor | None | None |

#### Simulation Features

##### Families of robots

| Library  | UGV (ground mobile robot) | UAV (aerial robots) | AUV (underwater robots) | Robotic Arms | Robotic Hands (grasping simulation) | Humanoid Robots | Human Avatars |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|
| DART     | Yes | No | No | Yes | Yes | Yes | No |

##### Supported dynamics algorithms

| Library  | Inverse Dynamics | Forward Dynamics | Hybrid Dynamics |
|:--------:|:----------------:|:----------------:|:---------------:|
| DART     | Yes | Yes  | Yes |

##### Supported joint types

| Library  | Revolute Joint | Prismatic Joint | Screw Joint | Universal Joint | Ball Joint | Euler Joint | Translational Joint | Planar Joint | Free Joint |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|:------------:|:----------:|
| DART     | Yes | Yes  | Yes | Yes | Yes | Yes | Yes | Yes | Yes |

##### Supported actuators

| Library  | Kinematic Actuators | Force-controlled Actuators | Servos |
|:--------:|:-------------------:|:--------------------------:|:------:|
| DART     | Yes | Yes  | Yes |

##### Supported sensors

| Library  | Odometry | IMU | Collision | GPS | Monocular Cameras | Stereo Cameras | Depth Cameras | 2D Laser Scanners | 3D Laser Scanners |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|:------------:|:----------:|
| DART     | No | No | No | No | No | No | No | No | No |

### Benchmark

#### Problems

* [IFToMM](http://iftomm-multibody.org/benchmark/) - A tool for the international multibody dynamics community to propose, solve, and refer to a collection of benchmark problems.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) - Benchmark Problems for Multibody Dynamics (BPMD) Database.

#### Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015. ([pdf](https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf))
* S. Ivaldi et al. [Tools for simulating humanoid robot dynamics: a survey based on user feedback](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7041462). Humanoids 2014. ([pdf](http://arxiv.org/pdf/1402.7050.pdf))

## Other Libraries and Software

### Simulator

* [Gazebo](http://www.gazebosim.org/) A dynamic multi-robot simulator. ([bitbucket](https://bitbucket.org/osrf/gazebo))
* [MORSE](http://morse-simulator.github.io/) - The Modular OpenRobots Simulation Engine. ([github](https://github.com/morse-simulator/morse))
* [V-REP](http://www.coppeliarobotics.com/) - Virtual robot experimentation platform.

### Motion Planning

* [AIKIDO](https://github.com/personalrobotics/aikido) - A C++ library for solving robotic motion planning and decision making problems. ([github](https://github.com/personalrobotics/aikido))
* [OMPL](http://ompl.kavrakilab.org/) - The Open Motion Planning Library. ([bitbucket](https://bitbucket.org/ompl/ompl), [github](https://github.com/ompl/ompl))

### Collision Detection

* [FCL](https://github.com/flexible-collision-library/fcl) - Flexible Collision Library. ([github](https://github.com/flexible-collision-library/fcl))
* [libccd](https://github.com/danfis/libccd) - Library for collision detection between two convex shapes. ([github](https://github.com/danfis/libccd))

### Robot Model Description Format

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. ([bitbucket](https://bitbucket.org/osrf/sdformat))
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. ([github](https://github.com/ros/urdfdom))

## External Links

* [9 open source robotics projects](https://opensource.com/life/16/4/open-source-robotics-projects) by [Jason Baker](https://opensource.com/users/jason-baker)
* [Comparison of Rigid Body Dynamic Simulators for Robotic Simulation in Gazebo](http://www.osrfoundation.org/wordpress2/wp-content/uploads/2015/04/roscon2014_scpeters.pdf) by [Steven Peters](http://www.osrfoundation.org/team/steven-peters/) and [John Hsu](http://www.osrfoundation.org/team/john-hsu/)
* [Survey of multibody dynamics software](http://homepages.rpi.edu/~luy4/survey.html) maintained by [Ying Lu](http://homepages.rpi.edu/~luy4/)
* [Wikipedia: Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators)

## Contributing

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first. Also, please feel free to report any error.

## License

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
