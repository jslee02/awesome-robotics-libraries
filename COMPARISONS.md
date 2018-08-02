# Comparisons

#### Table of Contents
* [Multibody Dynamics Libraries](#multibody-dynamics-libraries)
* [Simulators](#simulators)

## [Multibody Dynamics Libraries](#awesome-robotics-libraries)

Comparisons inspired by [Wikipedia's Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators) page. 
> :warning: **Warning**: Most fields are missing. Any help would be appreciated.

### [Selected Libraries to Compare](#Comparisons)

* [Bullet](http://bulletphysics.org/wordpress/) ([github](https://github.com/bulletphysics/bullet3)) - Real-Time Physics Simulation
* [CHRONO::ENGINE](http://chronoengine.info/) ([github](https://github.com/projectchrono/chrono)) - C++ library for multibody dynamics simulations.
* [DART](http://dartsim.github.io/) ([github](https://github.com/dartsim/dart.git)) - Dynamic Animation and Robotics Toolkit.
* [Drake](http://drake002.csail.mit.edu/drake/sphinx/) ([github](https://github.com/RobotLocomotion/drake)) - A planning, control, and analysis toolbox for nonlinear dynamical systems.
* [ODE](http://www.ode.org/) ([bitbucket](https://bitbucket.org/odedevs/ode)) - An open source, high performance library for simulating rigid body dynamics.
* [Simbody](https://simtk.org/home/simbody/) ([github](https://github.com/simbody/simbody.git)) - A multibody dynamics/physics library for simulating articulated biomechanical/mechanical systems.

### General Information

| Library  | Developer(s) | Latest Release | Platform Supported | License |
|:--------:| ------------ | -------------- | ------------------ | ------- |
| chrono   | [University of Wisconsin-Madison](http://www.projectchrono.org/about/) | IntPoint1.2 (2016-02-27) | Linux, Windows | BSD 3-Clause |
| Bullet   | Erwin Coumans | 2.83 (2016-01-08) | Linux, MacOS, Windows | Zlib |
| DART     | Georgia Tech and CMU | [6.0.0 (2016-05-10)](https://github.com/dartsim/dart/releases/tag/v6.0.0) | Linux, MacOS, Windows | BSD 2-Clause |
| Drake    | MIT and TRI | 0.9.11 (2015-10-08) | Linux, MacOS, Windows | BSD 3-Clause |
| ODE      | [Russell Smith](http://www.q12.org/) | [0.14 (2015-12-18)](https://bitbucket.org/odedevs/ode/commits/tag/0.14) | Linux, MacOS, Windows | LGPL 2.1 or BSD 3-Clause |
| Simbody  | [Simtk.org](https://simtk.org/xml/index.xml) | 3.5.3 (2015-06-15) | Linux, MacOS, Windows | Apache 2.0 |
:star: updated Apr 19, 2016

### Technical Information

| Library  | Main Programming Language | External APIs | File Formats Support |
|:--------:|:-------------------------:|:-------------:|:--------------------:|
| chrono   | C++ | C++, Python | unknown |
| Bullet   | C++ | C++ | URDF |
| DART     | C++ | C++, Python | URDF, SDF, VSK, SKEL |
| Drake    | C++ and MATLAB | C++, Python | own json format |
| ODE      | C++ | C++ | unknown |
| Simbody  | C++ | C++ | URDF |

### Support

| Library  | Mailing List | API Documentation | Public Forum/Help System | User Manual | Tutorial | Issue Tracker | Wiki |
|:--------:|:------------:|:-----------------:|:------------------------:|:-----------:|:--------:|:-------------:|:----:|
| DART    | No | [Yes](http://dartsim.github.io/dart/) | No | No | [Yes](http://dart.readthedocs.org/en/release-5.1/) | [Yes](https://github.com/dartsim/dart/issues) | [Yes](https://github.com/dartsim/dart/wiki) |

### Code Quality

| Library  | Lines of Code | Lines of Comments | Coverage | Continuous Integration | Static Code Checker | Style Checker |
|:--------:|:-------------:|:-----------------:|:--------:|:----------------------:|:-------------------:|:-------------:|
| DART    | [75k](https://github.com/dartsim/dart/wiki/Project-Status) | [34k](https://github.com/dartsim/dart/wiki/Project-Status)  | [51%](https://coveralls.io/github/dartsim/dart) | Travis CI, Appveyor | None | None |

### Simulation Features

#### Families of Robots

| Library  | UGV (ground mobile robot) | UAV (aerial robots) | AUV (underwater robots) | Robotic Arms | Robotic Hands (grasping simulation) | Humanoid Robots | Human Avatars |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|
| DART     | Yes | No | No | Yes | Yes | Yes | No |

#### Supported Kinematics/Dynamics Algorithms

| Library  | Inverse Kinematics | Inverse Dynamics | Forward Dynamics | Hybrid Dynamics |
|:--------:|:------------------:|:----------------:|:----------------:|:---------------:|
| DART     | Yes | Yes | Yes  | Yes |

#### Supported Joint Types

| Library  | Revolute | Prismatic | Screw | Universal | [Cylindrical](https://en.wikipedia.org/wiki/Cylindrical_joint) | Ball | Euler | Translational | Planar | Free |
|:--------:|:--------:|:---------:|:-----:|:---------:|:------------:|:----:|:-----:|:-------------:|:------:|:----:|
| DART     | Yes | Yes  | Yes | Yes | [No](https://github.com/dartsim/dart/issues/721) | Yes | Yes | Yes | Yes | Yes |

#### Supported Actuators

| Library  | Kinematic Actuators | Force-controlled Actuators | Servos |
|:--------:|:-------------------:|:--------------------------:|:------:|
| DART     | Yes | Yes  | Yes |

#### Supported Sensors

| Library  | Odometry | IMU | Collision | GPS | Monocular Cameras | Stereo Cameras | Depth Cameras | 2D Laser Scanners | 3D Laser Scanners |
|:--------:|:--------------:|:---------------:|:-----------:|:---------------:|:----------:|:-----------:|:---------------------:|:------------:|:----------:|
| DART     | No | No | No | No | No | No | No | No | No |

### Benchmark

* [scpeters/benchmark](https://github.com/scpeters/benchmark) - Benchmark comparisons of rigid-body dynamics simulators
* [leggedrobotics/SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/) - [[github](https://github.com/leggedrobotics/SimBenchmark)]

#### Problems

* [IFToMM](http://iftomm-multibody.org/benchmark/) - A tool for the international multibody dynamics community to propose, solve, and refer to a collection of benchmark problems.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) - Benchmark Problems for Multibody Dynamics (BPMD) Database.

#### Papers

* M. Torres-Torriti et al. [Survey and comparative study of free simulation software for mobile robots](http://journals.cambridge.org/action/displayAbstract?fromPage=online&aid=10215708&fileId=S0263574714001866). Robotica 2016.
* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015. ([pdf](https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf))
* Y. Lu et al. [Comparison of Multibody Dynamics Solver Performance: Synthetic versus Realistic Data](http://proceedings.asmedigitalcollection.asme.org/proceeding.aspx?articleid=2483796). ASME IDETC/CIEC 2015. ([pdf](http://www.cs.rpi.edu/foswiki/pub/RoboticsWeb/LabPublications/LTasme2015.pdf))
* S. Ivaldi et al. [Tools for simulating humanoid robot dynamics: a survey based on user feedback](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7041462). Humanoids 2014. ([pdf](http://arxiv.org/pdf/1402.7050.pdf))

#### Articles

* [9 open source robotics projects](https://opensource.com/life/16/4/open-source-robotics-projects) by [Jason Baker](https://opensource.com/users/jason-baker)
* [Comparison of Rigid Body Dynamic Simulators for Robotic Simulation in Gazebo](http://www.osrfoundation.org/wordpress2/wp-content/uploads/2015/04/roscon2014_scpeters.pdf) by [Steven Peters](http://www.osrfoundation.org/team/steven-peters/) and [John Hsu](http://www.osrfoundation.org/team/john-hsu/)
* [Open Source Physics Engines](http://www.tapirgames.com/blog/open-source-physics-engines) by [Tapir Liu](https://twitter.com/TapirLiu)
* [Survey of multibody dynamics software](http://homepages.rpi.edu/~luy4/survey.html) maintained by [Ying Lu](http://homepages.rpi.edu/~luy4/)
* [Wikipedia: Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators)

## [Simulators](#comparisons)

* [Gazebo](http://www.gazebosim.org/) ([bitbucket](https://bitbucket.org/osrf/gazebo)) - A dynamic multi-robot simulator.
* [GraspIt!](http://graspit-simulator.github.io/) ([github](https://github.com/graspit-simulator/graspit)) - A simulator for grasping research that can accommodate arbitrary hand and robot designs developed by [the Columbia University Robotics Group](http://www.cs.columbia.edu/robotics/)
* [MORSE](http://morse-simulator.github.io/) ([github](https://github.com/morse-simulator/morse)) - The Modular OpenRobots Simulation Engine.
* [V-REP](http://www.coppeliarobotics.com/) - Virtual robot experimentation platform.

### General Information

| Library  | Developer(s) | Latest Release | Platform Supported | License | Github :star: |
|:--------:| ------------ | -------------- | ------------------ | ------- |:------------:|
| Gazebo   | [OSRF](http://www.osrfoundation.org/) | [7.1.0 (2016-04-08)](http://www.gazebosim.org/download) | Linux, Mac, Windows | Apache 2.0 | N/A |
| MORSE    | [LAAS-CNRS and many](http://www.openrobots.org/morse/doc/latest/credits.html) | 1.4 (2016-02-08) | Linux | BSD 3-Clause | 153 |
| V-REP    | [Coppelia Robotics](http://www.coppeliarobotics.com/contact.html) | [3.3.0 (2016-02-19)](http://www.coppeliarobotics.com/downloads.html) | Linux, Mac, Windows | [Commercial / Free educational](http://www.coppeliarobotics.com/licensing.html) | N/A |
:star: updated Apr 19, 2016

### Papers

* A. Staranowicz, G. L. Mariottini, [A Survey and Comparison of Commercial and
Open-Source Robotic Simulator Software](http://dl.acm.org/citation.cfm?id=2141689), PETRA 2011. ([pdf](http://ranger.uta.edu/~gianluca/papers/StMa_PETRA11.pdf))

## [Contributing](#comparison)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/master/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#comparison)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
