# vmplib (VEX Motion Profiling Library)  

## About This Repository  

Welcome to **vmplib**, a lightweight C++ library for 2D motion profiling tailored to VEX robotics. This repo implements the methods described in the paper paper:

> *2D Motion Profiling for Competitive Robotics*.  
> [Read the PDF](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)

Use `vmplib` to generate 2d motion profiles for autonomous routines or simulations.

---

## Features  

### Current Implementations  
- **Cubic Bézier Path Generation**  
  Compute positions, derivatives, speeds, and curvature on any 2D Bézier spline.  
- **Arc-Length Parameterization**  
  Gaussian-quadrature and Newton–Raphson routines to map “distance traveled” ↔︎ spline parameter *t*.  
- **Trapezoidal & Keyframe Velocity Profiling**  
  Generate linear and angular velocity profiles with acceleration/deceleration phases or custom keyframes.  
- **RAMSETE-Style Feedback Simulation**  
  Simulate closed-loop following with a RAMSETE-inspired controller and first-order motor deadbands.  

---

## Usage Instructions  

### Prerequisites  
- **GNU Make**  
- **g++** (or `clang++`) with C++17 support  
- (Optional) Nix & `nix develop`

### Clone & Build  

```bash
git clone https://github.com/SerrialError/vmplib.git
cd vmplib
make          # builds bin/main
```
or with Nix Flakes
```bash
nix develop   # drop into shell with g++ & cmake
make
```

---

## Future Plans
- **PROS Modules**

- **CLI Tool for Path Import/Export**

- **Integration with Path-Jerry and other route planners**

Contributions, issues, and pull requests are welcome!
