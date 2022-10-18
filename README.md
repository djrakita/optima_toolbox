# optima_toolbox

[![Crates.io](https://img.shields.io/crates/v/optima.svg)](https://crates.io/crates/optima)

| [Documentation](https://djrakita.github.io/optima_toolbox/) | [API](https://docs.rs/optima/0.0.4/optima/) | 

## Development Update for Version 0.0.3 (8/10/2022)
- [Initial documentation](https://djrakita.github.io/optima_toolbox/) has been posted!  It's still a work in progress, but it's a good start.
- `OptimaIK` has been implemented.  This new IK solver heavily draws on *RelaxedIK* and *CollisionIK*, and also adds a "static" IK option.

## Introduction

Optima is an easy to set up and easy to use robotics toolbox.  Its primary use-case is robot motion generation (e.g., trajectory optimization, motion planning, optimization-based inverse kinematics, etc), though its underlying structures are general and can apply to many planning and optimization problem domains.  The core library is written in Rust, though high quality support for other targets such as Python and Webassembly are afforded via Rust's flexible compiler.  

Optima features an extensive suite of efficient and flexible robotics subroutines, including forward kinematics, jacobian computations, etc.  Optima also features implementations of state-of-the-art robotics algorithms related to planning, optimization, and proximity computations, such as RelaxedIK, CollisionIK, and Proxima (with more algorithms on the way).  These robotics features are designed to be easy to set up and access.  Models of many popular robots such as UR5, Franka Emika Panda, Kinova Gen3, etc., are included out-of-the-box and can be instantiated with a single line of code.  If your robot model is not included in our provided assets, a robot can be added to the library in just minutes.    

### High-level Features

- The underlying non-linear optimization engine has several backend options, such as the [Open Engine](https://alphaville.github.io/optimization-engine/docs/open-intro) and [NLopt](https://nlopt.readthedocs.io/en/latest/).  All algorithms support a non-linear objective function, non-linear equality constraints, and non-linear inequality constraints.  Most provided algorithms are local solvers, though several global optimization algorithms are provided via NLopt as well.  These optimization algorithm options can be swapped in and out by changing a single parameter, making testing and benchmarking over optimization algorithms easy to conduct.

- The library presents a powerful Rust trait called `OptimaTensorFunction`; any function that implements this trait can automatically compute derivatives (up to the fourth derivative) using a derivative composition graph that automatically selects either analytical derivatives, when provided, or finite-difference approximate derivatives, by default, through its traversal.  Thus, instead of depending on low-level automatic differentiation libraries that often fail on important subroutines, this approach provides a highly efficient and flexible way to perform derivatives for any implemented function.  High order derivatives are even computable on multi-dimensional functions as the library supports tensors of any arbitrary dimension.  For instance, a function \\( \mathit{f}: \mathbb{R}^{3 \times 5} \rightarrow {R}^{4 \times 4} \\) implemented as an `OptimaTensorFunction` can automatically provide derivatives \\( \frac{\partial f}{\partial \mathbf{X}} \in {R}^{4 \times 4 \times 3 \times 5} \\), \\( \frac{\partial^2 f}{\partial \mathbf{X}^2} \in {R}^{4 \times 4 \times 3 \times 5 \times 3 \times 5} \\), etc.

- Optima uses flexible transform and rotation implementations that make SE(3) computations a breeze.  For instance, any computations over transforms can use homogeneous matrices, implicit dual quaternions, a rotation matrix + vector, unit quaternion + vector, euler angles + vector, etc.  No matter what SE(3) representation is chosen, the library will automatically ensure that all conversions and computations are handled correctly under the hood.   

- In just a few lines of code, individual robot models can be combined to form a "robot set", allowing for easy planning and optimization over several robots all at once.  For instance, a UR5 (6DOF) can be easily combined with two Rethink Sawyer (7DOF) robots to form a robot set with 20 total DOF.  In addition, any robot in the library (including those in a robot set) can be easily supplemented with a mobile base of numerous types (e.g., floating base, planar base, etc.).  The extra degrees of freedom accompanying these mobile base options are automatically added to the robot (or robot set) model.  
   
### Sub-repositories

The Optima Toolbox is comprised of four primary sub-repositories:

1. [`optima`](https://github.com/djrakita/optima): This repository contains the core Rust code of the Optima Toolbox.  This code can be compiled to Rust, Python, and Web-assembly.    
1. [`optima_assets`](https://github.com/djrakita/optima_assets/tree/main): This repository contains asset files that are pre-packaged with the toolbox.  For instance, these assets include pre-processed robot models (in the [`optima_robots`](https://github.com/djrakita/optima_robots/tree/main) repository), as well as 3D mesh files that can be used to build up scenes around robot models.  
1. [`optima_js`](https://github.com/djrakita/optima_js/tree/main): This repository will contain utility code when using Optima in Javascript.  This folder is an empty place-holder right now; code will be added here over time.
1. [`optima_py`](https://github.com/djrakita/optima_py/tree/main): This repository will contain utility code when using Optima in Python.  This folder is an empty place-holder right now; code will be added here over time.



 
