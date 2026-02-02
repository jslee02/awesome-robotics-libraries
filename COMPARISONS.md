# Comparisons

A companion to the main [Awesome Robotics Libraries](README.md) list, providing side-by-side comparisons of physics engines and dynamics libraries commonly used in robotics.

## Contents

* [Physics Engine Feature Matrix](#physics-engine-feature-matrix)
* [Robotics Dynamics Libraries](#robotics-dynamics-libraries)
* [Decision Guide](#decision-guide)
* [Benchmark Resources](#benchmark-resources)

## Physics Engine Feature Matrix

| Name | Rigid | Soft | Fluids | Contact | Differentiable | GPU | Language | License |
|------|:-----:|:----:|:------:|:-------:|:--------------:|:---:|----------|---------|
| [Bullet](https://pybullet.org/) | ✅ | ✅ | ❌ | Impulse, PGS | ❌ | ❌ | C++ | Zlib |
| [CHRONO](https://projectchrono.org/) | ✅ | ✅ | ✅ | DVI, SMC | ❌ | ✅ | C++ | BSD-3 |
| [DART](http://dartsim.github.io/) | ✅ | ✅ | ❌ | LCP, PGS | ❌ | ❌ | C++ | BSD-2 |
| [Drake](https://drake.mit.edu/) | ✅ | ⚠️ | ⚠️ | SAP, TAMSI | ✅ | ❌ | C++ | BSD-3 |
| [Flex](https://developer.nvidia.com/flex) | ✅ | ✅ | ✅ | Particle-based | ❌ | ✅ | C++ | — |
| [MuJoCo](https://mujoco.org/) | ✅ | ✅ | ❌ | Complementarity | ✅ (MJX) | ✅ (MJX) | C++ | Apache-2.0 |
| [Newton Dynamics](https://newtondynamics.com/) | ✅ | ❌ | ❌ | Impulse | ❌ | ❌ | C++ | Zlib |
| [nphysics](https://nphysics.org/) | ✅ | ❌ | ❌ | Impulse | ❌ | ❌ | Rust | BSD-3 |
| [ODE](https://ode.org/) | ✅ | ❌ | ❌ | LCP | ❌ | ❌ | C++ | LGPL/BSD |
| [PhysX](https://nvidia-omniverse.github.io/PhysX/) | ✅ | ✅ | ✅ | TGS, PGS | ❌ | ✅ | C++ | BSD-3 |
| [pinocchio](https://stack-of-tasks.github.io/pinocchio/) | ✅ | ❌ | ❌ | — | ✅ | ❌ | C++ | BSD-2 |
| [ReactPhysics3d](https://www.reactphysics3d.com/) | ✅ | ❌ | ❌ | Impulse | ❌ | ❌ | C++ | Zlib |
| [Simbody](https://simtk.org/home/simbody/) | ✅ | ❌ | ❌ | Complementarity | ❌ | ❌ | C++ | Apache-2.0 |
| [SOFA](https://www.sofa-framework.org/) | ✅ | ✅ | ❌ | Penalty, LM | ❌ | ✅ | C++ | LGPL-2.1 |
| [Tiny Diff. Sim.](https://github.com/erwincoumans/tiny-differentiable-simulator) | ✅ | ❌ | ❌ | Impulse | ✅ | ❌ | C++ | Apache-2.0 |

**Legend**: ✅ Supported · ❌ Not supported · ⚠️ Partial/limited support

## Robotics Dynamics Libraries

Libraries commonly used for robot dynamics computation in research and applications.

| Name | Forward Dyn. | Inverse Dyn. | IK | URDF | Python API | Analytical Derivatives | Active (2025+) |
|------|:------------:|:------------:|:--:|:----:|:----------:|:---------------------:|:--------------:|
| [pinocchio](https://github.com/stack-of-tasks/pinocchio) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| [DART](https://github.com/dartsim/dart) | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| [Drake](https://github.com/RobotLocomotion/drake) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | ✅ | ✅ | ❌ | ❌ | ✅ | ✅ | ✅ |
| [RBDL](https://github.com/rbdl/rbdl) | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| [Simbody](https://github.com/simbody/simbody) | ✅ | ✅ | ❌ | ✅ | ❌ | ❌ | ✅ |
| [idyntree](https://github.com/gbionics/idyntree) | ✅ | ✅ | ❌ | ✅ | ✅ | ❌ | ✅ |
| [KDL](https://github.com/orocos/orocos_kinematics_dynamics) | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ | ✅ |
| [RBDyn](https://github.com/jrl-umi3218/RBDyn) | ✅ | ✅ | ❌ | ❌ | ✅ | ❌ | ✅ |

## Decision Guide

**Which library should I use?**

* **Robotics research (manipulation, locomotion):** [pinocchio](https://github.com/stack-of-tasks/pinocchio), [Drake](https://github.com/RobotLocomotion/drake), or [MuJoCo](https://github.com/google-deepmind/mujoco) — all actively maintained with strong community support.
* **Reinforcement learning for robotics:** [MuJoCo](https://github.com/google-deepmind/mujoco) (with MJX for GPU), [Bullet/PyBullet](https://github.com/bulletphysics/bullet3), or [DART](https://github.com/dartsim/dart) via [gym-dart](https://github.com/DartEnv/dart-env).
* **Game-like real-time physics:** [Bullet](https://github.com/bulletphysics/bullet3), [PhysX](https://github.com/NVIDIA-Omniverse/PhysX), or [ReactPhysics3d](https://github.com/DanielChappuis/reactphysics3d).
* **Biomechanical simulation:** [Simbody](https://github.com/simbody/simbody) (the engine behind OpenSim).
* **Multi-physics (FEM, soft bodies, fluids):** [SOFA](https://github.com/sofa-framework/sofa) or [CHRONO](https://github.com/projectchrono/chrono).
* **Differentiable simulation:** [Drake](https://github.com/RobotLocomotion/drake), [MuJoCo MJX](https://github.com/google-deepmind/mujoco), or [Tiny Differentiable Simulator](https://github.com/erwincoumans/tiny-differentiable-simulator).
* **Lightweight rigid-body only:** [RBDL](https://github.com/rbdl/rbdl), [KDL](https://github.com/orocos/orocos_kinematics_dynamics), or [RBDyn](https://github.com/jrl-umi3218/RBDyn).
* **Rust ecosystem:** [nphysics](https://github.com/dimforge/nphysics) (note: limited maintenance since 2021; consider [Rapier](https://rapier.rs/) as successor).

## Benchmark Resources

### Benchmark Suites

* [scpeters/benchmark](https://github.com/scpeters/benchmark) — Benchmark comparisons of rigid-body dynamics simulators.
* [leggedrobotics/SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/) — Comprehensive benchmark for physics simulation in robotics. [[github](https://github.com/leggedrobotics/SimBenchmark)]
* [IFToMM](http://iftomm-multibody.org/benchmark/) — Benchmark problems from the international multibody dynamics community.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) — Benchmark Problems for Multibody Dynamics database.

### Key Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015. ([pdf](https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf))
* M. Torres-Torriti et al. [Survey and comparative study of free simulation software for mobile robots](http://journals.cambridge.org/action/displayAbstract?fromPage=online&aid=10215708&fileId=S0263574714001866). Robotica 2016.
* S. Ivaldi et al. [Tools for simulating humanoid robot dynamics: a survey based on user feedback](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7041462). Humanoids 2014. ([pdf](http://arxiv.org/pdf/1402.7050.pdf))
* Y. Lu et al. Comparison of Multibody Dynamics Solver Performance: Synthetic versus Realistic Data. ASME IDETC/CIEC 2015.

### Articles

* [Comparison of Rigid Body Dynamic Simulators for Robotic Simulation in Gazebo](http://www.osrfoundation.org/wordpress2/wp-content/uploads/2015/04/roscon2014_scpeters.pdf) — Steven Peters and John Hsu.
* [Wikipedia: Robotics simulator](https://en.wikipedia.org/wiki/Robotics_simulator#Simulators)

## [Contributing](#comparisons)

Contributions are very welcome! Please read the [contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md) first. Also, please feel free to report any error.

## [License](#comparisons)

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
