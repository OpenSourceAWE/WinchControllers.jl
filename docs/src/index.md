```@meta
CurrentModule = WinchControllers
```

# WinchControllers
Documentation for the package [WinchControllers](https://github.com/aenarete/WinchControllers.jl).

This package is part of Julia Kite Power Tools, which consist of the following packages:

![Julia Kite Power Tools](kite_power_tools.png)

## Goals of this package
The goal of this package is to provide controllers for winches that consist of a motor/generator connected to a drum (with or without gearbox). On the drum is a tether that is connected to a load or a kite. Currently operation in air is assumed, but the package could also be extended for winches connected to under-water cables. While the main use case of the author are airborne wind energy systems, I am open to add features needed for other use cases.

**Implemented features:**
- lower force control (assure that there is always a minimal cable tension)
- upper force control (keep the maximal force limited)
- reel-out speed control proportional to the square root of the force (other relationships can easily be added)
- control of asynchronous motors/ generators
- speed control
- auto-tuning of the controller

**Planned features**
- support of torque controlled winches
- length control (position control)
- integration of a quasi-steady tether model

## Installation
Install [Julia 1.10](https://ufechner7.github.io/2024/08/09/installing-julia-with-juliaup.html) or later, if you haven't already. On Linux, make sure that Python3 and Matplotlib are installed:
```
sudo apt install python3-matplotlib
```
Before installing this software it is suggested to create a new project, for example like this:
```bash
mkdir test
cd test
julia --project="."
```
Then add WinchControllers from  Julia's package manager, by typing:
```julia
using Pkg
pkg"add WinchControllers"
``` 
at the Julia prompt. You can run the unit tests with the command (careful, can take 60 min):
```julia
pkg"test WinchControllers"
```

## Provides
- a set of generic control components, see [Generic Components](@ref)
- a winch controller [WinchController](@ref), that limits the upper and lower force and controls the speed as function of the force
- a winch controller settings struct [WCSettings](@ref) for the settings
- [Utility Functions and Macros](@ref)

## See also
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=Fechner+wind&pageSize=50&ordering=rating&descending=true) for the scientific background of this code
- The meta-package  [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl)
- the package [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
- the packages [WinchModels](https://github.com/aenarete/WinchModels.jl) and [KitePodModels](https://github.com/aenarete/KitePodModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the packages [KiteControllers](https://github.com/aenarete/KiteControllers.jl) and [KiteViewers](https://github.com/aenarete/KiteViewers.jl)

Author: Uwe Fechner (uwe.fechner.msc@gmail.com)
