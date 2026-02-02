# Comparisons

A companion to the main [Awesome Robotics Libraries](README.md) list, providing side-by-side comparisons of physics engines and dynamics libraries commonly used in robotics.

> **Last updated**: February 2026. Feature data verified against source code at each library's HEAD commit.

## Contents

* [Physics Engine Feature Matrix](#physics-engine-feature-matrix)
* [Robotics Dynamics Libraries](#robotics-dynamics-libraries)
* [Decision Guide](#decision-guide)
* [Benchmark Resources](#benchmark-resources)

## Physics Engine Feature Matrix

| Name | Rigid | Soft | Fluids | Contact Solvers | Differentiable | GPU | Language | License |
|------|:-----:|:----:|:------:|-----------------|:--------------:|:---:|----------|---------|
| [Brax](https://github.com/google/brax) | ✅ | ❌ | ❌ | Impulse+Baumgarte, XPBD, Projected Gradient QP, MuJoCo | ✅ | ✅ | Python | Apache-2.0 |
| [Bullet](https://pybullet.org/) | ✅ | ✅ | ❌ | Sequential Impulse/PGS, Dantzig, Lemke, NNCG | ❌ | ⚠️ | C++ | Zlib |
| [CHRONO](https://projectchrono.org/) | ✅ | ✅ | ✅ | NSC (DVI), SMC (Hooke/Hertz/Flores) | ❌ | ✅ | C++ | BSD-3 |
| [DART](http://dartsim.github.io/) | ✅ | ✅ | ❌ | LCP: Dantzig, PGS, Lemke, + 12 more | ✅ | ❌ | C++ | BSD-2 |
| [Drake](https://drake.mit.edu/) | ✅ | ✅ | ❌ | SAP, TAMSI, Similar, Lagged; Hydroelastic | ✅ | ❌ | C++ | BSD-3 |
| [Flex](https://developer.nvidia.com/flex) | ✅ | ✅ | ✅ | PBD (Position-Based Dynamics) | ❌ | ✅ | C++ | Proprietary |
| [Genesis](https://genesis-world.readthedocs.io) | ✅ | ✅ | ✅ | Newton, CG; GJK+EPA / MPR | ✅ | ✅ | Python | Apache-2.0 |
| [MuJoCo](https://mujoco.org/) | ✅ | ✅ | ❌ | Newton, CG, PGS (convex) | ✅ | ✅ | C/C++ | Apache-2.0 |
| [Newton](https://newton-physics.github.io/newton/) | ✅ | ✅ | ✅ | MuJoCo (via Warp), XPBD, AVBD, penalty | ✅ | ✅ | Python | Apache-2.0 |
| [Newton Dynamics](https://newtondynamics.com/) | ✅ | ⚠️ | ⚠️ | Dantzig LCP | ❌ | ✅ | C++ | Zlib |
| [nphysics](https://nphysics.org/) | ✅ | ✅ | ❌ | Moreau-Jean (SOR-prox) | ❌ | ❌ | Rust | Apache-2.0 |
| [ODE](https://ode.org/) | ✅ | ❌ | ❌ | LCP Dantzig, PGS (QuickStep) | ❌ | ❌ | C/C++ | LGPL/BSD |
| [PhysX](https://nvidia-omniverse.github.io/PhysX/) | ✅ | ✅ | ✅ | PGS, TGS | ❌ | ✅ | C++ | BSD-3 |
| [pinocchio](https://stack-of-tasks.github.io/pinocchio/) | ⚠️ | ❌ | ❌ | Proximal, PGS, ADMM | ✅ | ❌ | C++ | BSD-2 |
| [ReactPhysics3d](https://www.reactphysics3d.com/) | ✅ | ❌ | ❌ | Sequential Impulses | ❌ | ❌ | C++ | Zlib |
| [Simbody](https://simtk.org/home/simbody/) | ✅ | ❌ | ❌ | Hunt-Crossley, Hertz, Elastic Foundation, PGS | ⚠️ | ❌ | C++ | Apache-2.0 |
| [SOFA](https://www.sofa-framework.org/) | ✅ | ✅ | ✅ | Penalty, LCP, Lagrange Multipliers, Augmented Lagrangian | ❌ | ✅ | C++ | LGPL-2.1 |
| [Tiny Diff. Sim.](https://github.com/erwincoumans/tiny-differentiable-simulator) | ✅ | ❌ | ❌ | MLCP (PGS), smooth spring-damper | ✅ | ✅ | C++ | Apache-2.0 |

**Legend**: ✅ Supported · ❌ Not supported · ⚠️ Partial/limited

**Notes**:
* **Brax**: Four physics pipelines — Spring (impulse), Positional (XPBD), Generalized (mass matrix QP), and MJX (delegates to MuJoCo). All written in JAX; differentiable via `jax.grad`. As of v0.13.0, only `brax/training` is actively maintained; physics users directed to MJX or MuJoCo Warp.
* **Bullet** GPU: OpenCL rigid-body pipeline exists but is experimental and not widely used.
* **DART** contact: Ships 15+ LCP solvers — Dantzig (default), PGS, Lemke, Baraff, Interior Point, MPRGP, and more. Collision backends: FCL, Bullet, ODE, built-in.
* **DART** differentiable: Analytical Lie-group Jacobians and their time derivatives (not AD).
* **Drake** differentiable: Full `AutoDiffXd` scalar type throughout `MultibodyPlant`. Soft bodies via FEM `DeformableVolume` (NeoHookean).
* **Flex**: Archived — last commit April 2021. GPU-only (CUDA/DX11/DX12), no CPU fallback. Superseded by PhysX 5 + NVIDIA Warp.
* **Genesis**: 6 custom Taichi-based solvers — Rigid (MuJoCo-inspired ABD), FEM (implicit Newton-PCG), MPM (elastic/liquid/sand/snow), SPH (DFSPH), PBD (cloth/elastic/liquid), Stable Fluid (Eulerian gas). Differentiable via Taichi autodiff + custom backward passes (rigid solver backward implemented, MPM/Tool officially supported). Claims 43M FPS (Franka arm, 30K parallel envs, RTX 4090). GPU via Taichi → CUDA/Vulkan/Metal.
* **MuJoCo** soft: Tendons, muscles, and deformable FEM bodies (`flexcomp` — St. Venant-Kirchhoff, 1D/2D/3D elements). Differentiable via finite-difference in C and full autodiff via MJX (JAX).
* **Newton**: Linux Foundation project (Disney Research, Google DeepMind, NVIDIA). 7 solver backends — Featherstone (CRBA), MuJoCo Warp, Semi-implicit Euler, XPBD, VBD/AVBD, Style3D (projective dynamics cloth), Implicit MPM. Differentiable via NVIDIA Warp's `wp.Tape()` reverse-mode AD + analytical IK Jacobians. Supports URDF, MJCF, and USD (with PhysX schema compatibility). Built-in GPU-batched IK (LM + L-BFGS). Active beta.
* **Newton Dynamics** soft: Deformable mesh API existed in v3.14 but was removed/commented out in v4. Fluids via CUDA SPH. The `MADEAPPS/newton-dynamics` repo is discontinued; active development continues at `JulioJerez/newton-dynamics`.
* **nphysics**: Passively maintained since July 2021. Officially superseded by [Rapier](https://rapier.rs/).
* **pinocchio** rigid: Provides dynamics algorithms (ABA, RNEA) but no built-in time-stepper — the user writes the integration loop. Has contact constraint solvers (Proximal, PGS, ADMM) with Coulomb friction cones since v3.
* **Simbody** differentiable: `SmoothSphereHalfSpaceForce` provides a C²-smooth contact model designed for gradient-based optimization.
* **SOFA** fluids: SPH via the [`SofaSphFluid`](https://github.com/sofa-framework/SofaSphFluid) plugin; Eulerian grid-based via the bundled `SofaEulerianFluid` plugin. GPU via bundled SofaCUDA (28 kernel files) and SofaOpenCL plugins.
* **Tiny Diff. Sim.**: Stale — last commit April 2023. GPU via CppAD code generation to CUDA kernels.

## Robotics Dynamics Libraries

Libraries commonly used for robot dynamics computation in research and applications.

| Name | Forward Dyn. | Inverse Dyn. | IK | URDF | Python API | Analytical Derivatives | Active (2025+) |
|------|:------------:|:------------:|:--:|:----:|:----------:|:---------------------:|:--------------:|
| [pinocchio](https://github.com/stack-of-tasks/pinocchio) | ✅ ABA | ✅ RNEA | ⚠️ | ✅ | ✅ | ✅ | ✅ |
| [DART](https://github.com/dartsim/dart) | ✅ ABA | ✅ RNEA | ✅ | ✅ | ✅ | ✅ | ✅ |
| [Drake](https://github.com/RobotLocomotion/drake) | ✅ ABA | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | ✅ | ✅ | ❌ | ✅ | ✅ | ⚠️ | ✅ |
| [Newton](https://github.com/newton-physics/newton) | ✅ CRBA | ✅ RNEA | ✅ | ✅ | ✅ | ⚠️ | ✅ |
| [Brax](https://github.com/google/brax) | ✅ | ✅ RNEA | ❌ | ❌ | ✅ | ❌ | ⚠️ |
| [RBDL](https://github.com/rbdl/rbdl) | ✅ ABA | ✅ RNEA | ✅ | ✅ | ✅ | ⚠️ | ⚠️ |
| [Simbody](https://github.com/simbody/simbody) | ✅ | ✅ | ✅ | ⚠️ | ❌ | ❌ | ✅ |
| [idyntree](https://github.com/robotology/idyntree) | ✅ ABA | ✅ RNEA | ✅ | ✅ | ✅ | ⚠️ | ✅ |
| [KDL](https://github.com/orocos/orocos_kinematics_dynamics) | ✅ | ✅ RNEA | ✅ | ❌ | ✅ | ❌ | ✅ |
| [RBDyn](https://github.com/jrl-umi3218/RBDyn) | ✅ CRBA | ✅ RNEA | ✅ | ✅ | ✅ | ⚠️ | ✅ |

**Notes**:
* **pinocchio** IK: Provides Jacobians, `integrate()`, and example IK loops — but no dedicated IK solver class. The companion project [LoIK](https://github.com/Simple-Robotics/LoIK) provides a dedicated solver. Analytical derivatives are the signature feature: closed-form derivatives of RNEA, ABA, constrained dynamics, kinematics, and centroidal dynamics (RSS 2018). Also supports CppAD and CasADi AD backends.
* **DART** IK: Extensive built-in IK — per-node `InverseKinematics`, `HierarchicalIK`, `WholeBodyIK`, `CompositeIK`, `IkFast` (OpenRAVE analytical), JacobianDLS, JacobianTranspose. Analytical derivatives: Lie-group Jacobians and time derivatives computed analytically. Also parses URDF, SDF, MJCF, and SKEL formats.
* **Drake** IK: `InverseKinematics`, `GlobalInverseKinematics` (mixed-integer), `DifferentialInverseKinematics`. Analytical derivatives via `AutoDiffXd` scalar templating. Also parses URDF, SDF, MJCF, and Drake Model Directives (`.dmd.yaml`).
* **MuJoCo** URDF: Native parser (auto-detects `<robot>` root element). IK: No built-in solver — provides Jacobians (`mj_jac`, `mj_jacBody`) for user-implemented IK. Derivatives: Finite-difference in C (`mjd_transitionFD`); full autodiff via MJX (JAX). Also native MJCF format.
* **Newton** FD: Featherstone CRBA (H·q̈ = τ − C, Cholesky). Also MuJoCo Warp backend for forward dynamics. ID: Full RNEA implementation. IK: GPU-batched LM + L-BFGS with analytical or autodiff Jacobians (selectable via `IKJacobianMode`). URDF + MJCF + USD parsers. Derivatives: Warp `wp.Tape()` autodiff + analytical IK Jacobians. Active beta (Linux Foundation project).
* **Brax** FD: Generalized pipeline solves `M·q̈ = -C + τ`; also Spring (impulse) and Positional (PBD) pipelines. ID: RNEA in generalized pipeline. IK: None. URDF: No parser — MJCF only (via `mujoco.MjModel`). Derivatives: Purely JAX autodiff (`jax.grad`). Active: Only `brax/training` maintained as of v0.13.0; physics users directed to MJX.
* **RBDL** derivatives: Via CasADi algorithmic differentiation backend (`rbdl-casadi`), not hand-coded. IK: Built-in damped least-squares with position/orientation/CoM constraints. Activity: Low — last commit June 2025.
* **Simbody** IK: `Assembler` framework with `Markers` (point-based) and `OrientationSensors` conditions. URDF: Example-only reader (Atlas demo), not a core library feature. Simbody's native API uses `MultibodyGraphMaker`.
* **idyntree** IK: Full NLP-based IK solver using IPOPT with quaternion/RPY parametrizations and CoM constraints. Derivatives: Experimental forward dynamics linearization (`ForwardDynamicsLinearization`, marked "HIGHLY EXPERIMENTAL"). Also parses SDF. Correct GitHub repo: [`robotology/idyntree`](https://github.com/robotology/idyntree).
* **KDL** IK: 7+ solver variants — Newton-Raphson, LMA, pseudoinverse, WDLS, nullspace optimization, and tree-based variants. Also provides the Vereshchagin hybrid dynamics solver. URDF: Requires external [`kdl_parser`](https://github.com/ros/kdl_parser) from ROS. Python: PyKDL via pybind11.
* **RBDyn** FD: CRBA-based (H·q̈ = τ − C with LDLT), not ABA. IK: Jacobian SVD solver with configurable damping. Derivatives: Inverse statics torque Jacobians (∂τ/∂q for static case) and IDIM for parameter identification. Built-in URDF read/write parser, no ROS dependency.

## Decision Guide

**Which library should I use?**

* **Robotics research (manipulation, locomotion):** [pinocchio](https://github.com/stack-of-tasks/pinocchio), [Drake](https://github.com/RobotLocomotion/drake), or [MuJoCo](https://github.com/google-deepmind/mujoco) — all actively maintained with strong community support and differentiable simulation.
* **Reinforcement learning for robotics:** [MuJoCo](https://github.com/google-deepmind/mujoco) (with MJX for GPU-accelerated batched simulation), [Isaac Lab](https://github.com/isaac-sim/IsaacLab) (GPU-parallel envs on Isaac Sim), [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) (43M FPS batched sim), or [Brax](https://github.com/google/brax) (JAX-native, training-focused).
* **Game-like real-time physics:** [Bullet](https://github.com/bulletphysics/bullet3), [PhysX](https://github.com/NVIDIA-Omniverse/PhysX), or [ReactPhysics3d](https://github.com/DanielChappuis/reactphysics3d).
* **Biomechanical simulation:** [Simbody](https://github.com/simbody/simbody) (the engine behind OpenSim).
* **Multi-physics (FEM, soft bodies, fluids):** [SOFA](https://github.com/sofa-framework/sofa), [CHRONO](https://github.com/projectchrono/chrono), [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) (6 unified solvers: rigid, FEM, MPM, SPH, PBD, Stable Fluid), or [Newton](https://github.com/newton-physics/newton) (FEM, MPM, cloth, XPBD on Warp).
* **Differentiable simulation:** [pinocchio](https://github.com/stack-of-tasks/pinocchio) (analytical derivatives), [Drake](https://github.com/RobotLocomotion/drake) (AutoDiffXd), [MuJoCo MJX](https://github.com/google-deepmind/mujoco) (JAX autodiff), [Newton](https://github.com/newton-physics/newton) (Warp autodiff), [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) (Taichi autodiff), or [Brax](https://github.com/google/brax) (JAX autodiff).
* **Lightweight rigid-body dynamics:** [RBDL](https://github.com/rbdl/rbdl), [KDL](https://github.com/orocos/orocos_kinematics_dynamics), or [RBDyn](https://github.com/jrl-umi3218/RBDyn).
* **Floating-base humanoid estimation/control:** [idyntree](https://github.com/robotology/idyntree) or [RBDyn](https://github.com/jrl-umi3218/RBDyn) (mc-rtc ecosystem).
* **GPU-accelerated robotics sim (Warp-based):** [Newton](https://github.com/newton-physics/newton) — Linux Foundation project with MuJoCo Warp backend, 7 solvers, GPU-batched IK.
* **Rust ecosystem:** [nphysics](https://github.com/dimforge/nphysics) (note: passively maintained since 2021; consider [Rapier](https://rapier.rs/) as its successor).

## Benchmark Resources

### Benchmark Suites

* [scpeters/benchmark](https://github.com/scpeters/benchmark) — Benchmark comparisons of rigid-body dynamics simulators.
* [leggedrobotics/SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/) — Comprehensive benchmark for physics simulation in robotics. [[github](https://github.com/leggedrobotics/SimBenchmark)]
* [IFToMM](http://iftomm-multibody.org/benchmark/) — Benchmark problems from the international multibody dynamics community.
* [BPMD](https://grasp.robotics.cs.rpi.edu/bpmd/) — Benchmark Problems for Multibody Dynamics database.

### Key Papers

* T. Erez et al. [Simulation tools for model-based robotics: comparison of Bullet, Havok, MuJoCo, ODE, and PhysX](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=7139807). ICRA 2015. ([pdf](https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf))
* J. Carpentier & N. Mansard. [Analytical derivatives of rigid body dynamics algorithms](https://hal.science/hal-01790971/). RSS 2018.
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
