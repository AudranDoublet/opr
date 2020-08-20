# Compile me

1. Install rustup: https://rustup.rs/
2. Run: rustup install nightly
3. In this directory, run: rustup override set nightly
4. Run cargo build --release

**Warning** You need ffmpeg installed to run the full pipeline (last step is the conversion from images to video)

# Command-line interface usage

**Example**
```bash
$ cargo run --release -- pipeline data/scenes/eiffel.yaml eiffelVideo
```

This will run the full pipeline and store a result video in `eiffelVideo/output.mp4`

For a complete documentation you can use:
```bash
$ cargo run --release -- --help
```

# Technical steps

Our program is composed of a few steps:
* Loading and preprocessing of objects (the time consuming part is computing a signed distance field for the mesh, which is used for collision handling; this part is cached)
* Processing the simulation
* Transformation of the particles into a mesh using a marching cube algorithm
* Rendering of the final scene using a raytracinbg algorithm
* Image to video conversion using ffmpeg

Rendering time for scene can be very high:
* we made some optimisations, but it's far from being perfect; mainly, some parts could be computed on a GPU
* we use photorealistic algorithms (SPH, marching cube and raytracing) which are greedy enough by nature

# Functionalities

All the listed functionalities can be configured in the scene YAML.

We implemented different functionalities in our simulation algorithm, including:
* a pressure solver named Divergence Free SPH (DFSPH); another pressure solver can be easily implemented
* an high viscosity solver (based on `A Physically Consistent Implicit Viscosity Solver for SPH Fluids`; 2018; Weiler and al.)
* a drag force (based on `Approximate Air-Fluid Interactions for SPH`; 2017; Glisser and al.)
* an elasticity force (based on `An Implicit SPH Formulation for Incompressible Linearly Elastic Solids`; 2018; Peer and al.)
* a surface tension force (based on `Versatile Surface Tension and Adhesion for SPH Fluids`; 2013; Akinci and al.)
* a vorticity force (based on `Turbulent Micropolar SPH Fluids with Foam`; 2017; Bender and al.)
* multiphase fluids interactions
* fluid-solid interactions (based on `Volume Maps: An Implicit Boundary Representation for SPH`; 2019; Bender and al.)
* basic solid object engine (using center of mass, inertia tensor and mass as physical properties for objects)
* spray, bubble and foam simulation (based on `Unified spray, foam and air bubbles for particle-based fluids`; 2012; Ihmsen and al.)
* simulation FPS

We also implemented animation functionalities 
* Animated camera
* Animated solid objects (to make, for example, a motor)
* Animated fluid emitters
* Animated wind
* Possibility to add particles using a predefined shape (cube or a mesh)
* Fixed particles, used for example in dancing armadillo scene (cf demonstration video)

Animated objects supports:
* Modification of position, velocity, acceleration, rotation, angular velocity, angular acceleration (and a 'look at' functionnality)
* each of this variables can be set
* each of this variables can be fixed for a certain time
* each of this variables can have a linear progression
* each of this variables can have a progression using a bezier curve
* for motions (linear progression and curve) and _fade in_ and _fade out_ effect can be added
* the system handle animation loops

We implemented functionalities for the marching cube step:
* linear intensity interpolation
* anisotropic kernel for intensity, enabling a more realistic meshing (based on `Reconstructing surfaces of particle-based fluids using anisotropic kernels`; 2013; Yu and Turk)

There are also rendering options:
* custom skybox
* custom fluid/solids materials for rendering (allowing diffusion, specular, reflection, refraction, beer law)
* adding an object that does not exist in the simulation
* rendering FPS (allowing slow/fast motion when rendering and simulation FPS are different)

As it was a school project with a limited time, we didn't have the time to implement some of the features we wanted:
* solid-solid interactions (an approach is implemented, but doesn't works well)
* spray and foam rendering

A documentation for scenes configuration doesn't exists, but the scenes in `data/scenes` folder are exhaustive.
