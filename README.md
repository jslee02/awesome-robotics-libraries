# Awesome Robotics Libraries

## Multibody Dynamics

* [Bullet](http://bulletphysics.org/wordpress/) - Real-Time Physics Simulation ([github](https://github.com/bulletphysics/bullet3))
* [CHRONO::ENGINE](http://chronoengine.info/) - C++ library for multibody dynamics simulations. ([github](https://github.com/projectchrono/chrono))
* [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit. ([github](https://github.com/dartsim/dart.git))
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) - A planning, control, and analysis toolbox for nonlinear dynamical systems. ([github](https://github.com/RobotLocomotion/drake))
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - A library for dynamic simulation of multi-body systems in C++.
* [KDL](http://www.orocos.org/kdl) - Orocos Kinematics and Dynamics C++ library. ([github](https://github.com/orocos/orocos_kinematics_dynamics))
* [metapod](https://github.com/laas/metapod) - A template-based robot dynamics library. ([github](https://github.com/laas/metapod))
* [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation library written in C++. ([github](https://github.com/PositronicsLab/Moby))
* [morse](http://morse-simulator.github.io/) - The Modular OpenRobots Simulation Engine. ([github](https://github.com/morse-simulator/morse))
* [mrpt](http://www.mrpt.org/) - The Mobile Robot Programming Toolkit. ([github](https://github.com/MRPT/mrpt))
* [ODE](http://www.ode.org/) - An open source, high performance library for simulating rigid body dynamics. ([bitbucket](https://bitbucket.org/odedevs/ode))
* [OpenRAVE](http://www.openrave.org) - An environment for testing, developing, and deploying robotics motion planning algorithms. ([github](https://github.com/rdiankov/openrave))
* [pinocchio](https://github.com/stack-of-tasks/pinocchio) - Dynamic computations using Spatial Algebra. ([github](https://github.com/stack-of-tasks/pinocchio))
* [PositionBasedDynamics](https://github.com/janbender/PositionBasedDynamics) - A library for the physically-based simulation of rigid bodies, deformable solids and fluids.
* [RBDL](http://rbdl.bitbucket.org/) - Rigid Body Dynamics Library. ([bitbucket](https://bitbucket.org/rbdl/rbdl))
* [Robotics Library](http://www.roboticslibrary.org/) - A self-contained C++ library for robot kinematics, motion planning and control. ([github](https://github.com/roboticslibrary/rl))
* [RobWork](http://www.robwork.dk/apidoc/nightly/rw/index.html) - A collection of C++ libraries for simulation and control of robot systems.
* [siconos](http://siconos.gforge.inria.fr) - A software package for the modeling and simulation of nonsmooth dynamical systems. ([github](https://github.com/siconos/siconos))
* [Simbody](https://simtk.org/home/simbody/) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems. ([github](https://github.com/simbody/simbody.git))
* [yarp](http://www.yarp.it/) - Yet Another Robot Platform. ([github](https://github.com/robotology/yarp))

### Comparison

#### General Information

| Library  | Developer(s) | Latest Release  | License | Platform Supported |
| :------: | ---------- | ------------------  | ------- | ------------------ |
| Bullet | Erwin Coumans | 2.83 (2016-01-08)  | Zlib | Linux, MacOS, Windows |
| DART | Georgia Tech | 5.1.1 (2015-11-06)  | BSD | Linux, MacOS, Windows |
| Simbody | [Simtk.org](https://simtk.org/xml/index.xml) | 3.5.3 (2015-06-15)  | Apache 2.0 | Linux, MacOS, Windows |

#### Support

| Library  | Mailing List | API Documentation  | Public Forum/Help System | User Manual | Tutorial | Issue Tracker | Wiki |
| :------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: |
| DART | No | [Yes](http://dartsim.github.io/dart/) | No | No | [Yes](http://dart.readthedocs.org/en/release-5.1/) | [Yes](https://github.com/dartsim/dart/issues) | [Yes](https://github.com/dartsim/dart/wiki) |

#### Code Quality

| Library  | Lines of Code | Lines of Comments  | Coverage | Continuous Integration | Static Code Checker | Style Checker |
| :------: | :------: | :------: | :------: | :------: | :------: | :------: |
| DART  | 58k | 26k  | Unknown | Travis CI, Appveyor | None | None |

#### Simulation Features

##### Supported dynamics algorithms

| Library  | Inverse Dynamics | Forward Dynamics  | Hybrid Dynamics |
| :------: | :--------------: | :--------------:  | :-------------: |
| DART | Yes | Yes  | Yes |

##### Supported joint types

| Library  | Revolute Joint | Prismatic Joint  | Screw Joint | Universal Joint | Ball Joint | Euler Joint | Translational Joint | Planar Joint | Free Joint |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| DART | Yes | Yes  | Yes | Yes | Yes | Yes | Yes | Yes | Yes |

### Benchmark

#### Problems

* [IFToMM](http://iftomm-multibody.org/benchmark/) - A tool for the international multibody dynamics community to propose, solve, and refer to a collection of benchmark problems.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) - Benchmark Problems for Multibody Dynamics (BPMD) Database.

#### Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). (ICRA 2015)
* S. Ivaldi et al. [Tools for simulating humanoid robot dynamics: a survey based on user feedback](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7041462). (Humanoids 2014)

## Other Libraries and Software

### Simulator

* [Gazebo](http://www.gazebosim.org/) A dynamic multi-robot simulator. ([bitbucket](https://bitbucket.org/osrf/gazebo))
* [V-REP](http://www.coppeliarobotics.com/) - Virtual robot experimentation platform.

### Motion Planning

* [OMPL](http://ompl.kavrakilab.org/) - The Open Motion Planning Library. ([bitbucket](https://bitbucket.org/ompl/ompl), [github](https://github.com/ompl/ompl))

### Collision Detection

* [FCL](https://github.com/flexible-collision-library/fcl) - Flexible Collision Library. ([github](https://github.com/flexible-collision-library/fcl))

### Robot Model Description Format

* [SDF](http://sdformat.org/) - XML format that describes objects and environments for robot simulators, visualization, and control. ([bitbucket](https://bitbucket.org/osrf/sdformat))
* [urdf](http://wiki.ros.org/urdf) - XML format for representing a robot model. ([github](https://github.com/ros/urdfdom))

## Contributing

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first.

## License

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
