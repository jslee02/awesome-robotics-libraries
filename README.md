# Awesome Robotics Libraries

* [Bullet](http://bulletphysics.org/wordpress/) - Real-Time Physics Simulation ([github](https://github.com/bulletphysics/bullet3))
* [CHRONO::ENGINE](http://chronoengine.info/) - C++ library for multibody dynamics simulations. ([github](https://github.com/projectchrono/chrono))
* [DART](http://dartsim.github.io/) - Dynamic Animation and Robotics Toolkit. ([github](https://github.com/dartsim/dart.git))
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) - A planning, control, and analysis toolbox for nonlinear dynamical systems. ([github](https://github.com/RobotLocomotion/drake))
* [IBDS](http://www.interactive-graphics.de/index.php/downloads/12-ibds) - A library for dynamic simulation of multi-body systems in C++.
* [KDL](http://www.orocos.org/kdl) - Orocos Kinematics and Dynamics C++ library. ([github](https://github.com/orocos/orocos_kinematics_dynamics))
* [metapod](https://github.com/laas/metapod) - A template-based robot dynamics library.
* [Moby](http://physsim.sourceforge.net/index.html) - Multi-body dynamics simulation library written in C++. ([github](https://github.com/PositronicsLab/Moby))
* [morse](http://morse-simulator.github.io/) - The Modular OpenRobots Simulation Engine. ([github](https://github.com/morse-simulator/morse))
* [mrpt](http://www.mrpt.org/) - The Mobile Robot Programming Toolkit. ([github](https://github.com/MRPT/mrpt))
* [ODE](http://www.ode.org/) - An open source, high performance library for simulating rigid body dynamics. ([bitbucket](https://bitbucket.org/odedevs/ode))
* [pinocchio](https://github.com/stack-of-tasks/pinocchio) - Dynamic computations using Spatial Algebra.
* [PositionBasedDynamics](https://github.com/janbender/PositionBasedDynamics) - A library for the physically-based simulation of rigid bodies, deformable solids and fluids.
* [RBDL](http://rbdl.bitbucket.org/) - Rigid Body Dynamics Library. ([bitbucket](https://bitbucket.org/rbdl/rbdl))
* [Robotics Library](http://www.roboticslibrary.org/) - A self-contained C++ library for robot kinematics, motion planning and control. ([github](https://github.com/roboticslibrary/rl))
* [RobWork](http://www.robwork.dk/apidoc/nightly/rw/index.html) - A collection of C++ libraries for simulation and control of robot systems.
* [siconos](http://siconos.gforge.inria.fr) - A software package for the modeling and simulation of nonsmooth dynamical systems. ([github](https://github.com/siconos/siconos))
* [Simbody](https://simtk.org/home/simbody/) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems. ([github](https://github.com/simbody/simbody.git))
* [yarp](http://www.yarp.it/) - Yet Another Robot Platform. ([github](https://github.com/robotology/yarp))

## Comparison

### General Information

| Library  | Developers | Latest Release  | License | Platform Supported |
| :------: | ---------- | ------------------  | ------- | ------------------ |
| [DART](http://dartsim.github.io/) | Georgia Tech | 5.1.1 (2015-11-06)  | BSD | Linux, MacOS, Windows |
| [Simbody](https://simtk.org/home/simbody) | [Simtk.org](https://simtk.org/xml/index.xml) | 3.5.3 (2015-06-15)  | Apache 2.0 | Linux, MacOS, Windows |

### Support

| Library  | Mailing List | API Documentation  | Public Forum/Help System | User Manual | Tutorial | Issue Tracker | Wiki |
| :------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: |
| [DART](http://dartsim.github.io/) | No | [Yes](http://dartsim.github.io/dart/) | No | No | [Yes](http://dart.readthedocs.org/en/release-5.1/) | [Yes](https://github.com/dartsim/dart/issues) | [Yes](https://github.com/dartsim/dart/wiki) |

### Code Quality

| Library  | Lines of Code | Lines of Comments  | Coverage | Continuous Integration | Static Code Checker | Style Checker |
| :------: | :------: | :------: | :------: | :------: | :------: | :------: |
| [DART](http://dartsim.github.io/)  | 58k | 26k  | Unknown | Travis CI, Appveyor | None | None |

### Simulation Features

#### Supported dynamics algorithms

| Library  | Inverse Dynamics | Forward Dynamics  | Hybrid Dynamics |
| :------: | :--------------: | :--------------:  | :-------------: |
| [DART](http://dartsim.github.io/) | Y | Y  | Y |

#### Supported joint types

| Library  | Revolute Joint | Prismatic Joint  | Screw Joint | Universal Joint | Ball Joint | Euler Joint | Translational Joint | Planar Joint | Free Joint |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| [DART](http://dartsim.github.io/) | Yes | Yes  | Yes | Yes | Yes | Yes | Yes | Yes | Yes |

## Benchmark

### Problems

* [IFToMM](http://iftomm-multibody.org/benchmark/) - A tool for the international multibody dynamics community to propose, solve, and refer to a collection of benchmark problems.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) - Benchmark Problems for Multibody Dynamics (BPMD) Database.

### Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015.

## Contributing

Contributions are very welcome!
