# vmplib (VEX Motion Profiling Library)  

## About This Repository  

Welcome to **vmplib**, a lightweight C++ library for 2D motion profiling tailored to VEX robotics. This repo implements the methods described in our paper:

> SerrialError & ChatGPT (2024).  
> *2D Motion Profiling for Competitive Robotics*.  
> 📄 [Read the PDF](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)

Use `vmplib` to generate smooth, curvature-aware motion profiles (Bezier-based, trapezoidal, RAMSETE) for autonomous routines or simulations.

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
- **Example Binaries**  
  - `src/main.cpp` – simple “printVels” demo  
  - `src/2dmpsimulate.cpp` – full simulation/driving example  

### Planned Enhancements  
- **Pure Pursuit Integration**  
- **PROS/VEXcode API Bindings**  
- **Multi-segment & Rotational Profiling**  
- **Real-time VEX V5 Deployment Scripts**

---
## Development Process  

1. **Theory & Math**  
   Derived and documented in our paper: [2dmp.pdf](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)  
2. **API Design**  
   Exposed minimal structs (`Point`, `Pose`, `VelocityLayout`, etc.) and free functions in `bezier.hpp`.  
3. **Implementation**  
   All core code lives in `src/2dmpsimulate.cpp`. Example usage in `src/main.cpp`.  
4. **Build System**  
   Simple `Makefile` for GCC/CMake integration. Nix flakes for reproducible dev shells.

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
### Future Plans
- **PROS Modules**

- **CLI Tool for Path Import/Export**

- **Integration with Path-Jerry and other route planners**

Contributions, issues, and pull requests are welcome!
