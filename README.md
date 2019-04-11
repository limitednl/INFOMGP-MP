# Game Physics Project
We implemented a simple fluid simulation using SPH. 
The simulation uses 2k particles by default and is restricted to a 10x10x10 box.
Each particle has a mass value of 1.997.

The other aspects of the simulation are tweakable in the UI, which include gravity, fluid viscosity, the outside wall pressure, 
the coefficient of restitution, the gas constant, the environmental pressure, the time step, and flags that determine whether or not to use each feature.

For smoothing kernels we used the Poly6, Spiky, and Viscosity kernels outlined in [this](https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf) thesis.

## Installation
The framework we used has been repurposed from the practical skeleton and thus the installation instructions remain the same.

The skeleton uses the following dependencies: [libigl](http://libigl.github.io/libigl/), and consequently [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), for the representation and viewing of geometry, and [libccd](https://github.com/danfis/libccd) for collision detection. libigl viewer is using [dear imGui](https://github.com/ocornut/imgui) for the menu. Everything is bundled as either submodules, or just incorporated code within the environment, and you do not have to take care of any installation details. To get the library, use:

```bash
git clone --recursive https://github.com/avaxman/INFOMGP-Practical1.git
```

to compile the environment, go into the repository folder and enter in a terminal (macOS/Linux):

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

In windows, you need to use [cmake-gui](https://cmake.org/runningcmake/). Pressing twice ``configure`` and then ``generate`` will generate a Visual Studio solution in which you can work. The active soution should be ``practical1_bin``. *Note*: it only seems to work in 64-bit mode. 32-bit mode might give alignment errors.

## Literature Used
[Fluid Simulation Course Notes](https://www.cs.ubc.ca/~rbridson/fluidsimulation/fluids_notes.pdf)
[Fluid Simulationusing Smoothed ParticleHydrodynamics, Burak Ertekin](https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf)