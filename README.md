# WinchControllers

[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://opensourceawe.github.io/WinchControllers.jl/dev)
[![Build Status](https://github.com/opensourceawe/WinchControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/opensourceawe/WinchControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/opensourceawe/WinchControllers.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/opensourceawe/WinchControllers.jl)
[![Aqua QA](https://raw.githubusercontent.com/JuliaTesting/Aqua.jl/master/badge.svg)](https://github.com/JuliaTesting/Aqua.jl)

Discrete controllers for Winches.

This package is part of Julia Kite Power Tools, which consists of the following packages:
<p align="center"><img src="./docs/kite_power_tools.png" width="500" /></p>

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
<details>
  <summary>Installation of Julia</summary>

If you do not have Julia installed yet, please read [Installation](https://github.com/aenarete/KiteSimulators.jl/blob/main/docs/Installation.md).

</details>

<details>
  <summary>Installation as package</summary>

### Installation of WinchControllers as package

It is suggested to use a local Julia environment. You can create it with:
```bash
mkdir myproject
cd myproject
julia --project=.
```
(don't forget typing the dot at the end), and then, on the Julia prompt enter:
```julia
using Pkg
pkg"add WinchControllers#main"
```
You can run the tests with:
```julia
using Pkg
pkg"test WinchControllers"
```
To add the examples and install the packages needed by the examples, run:
```julia
using WinchControllers
WinchControllers.install_examples()
exit()
```
</details>

<details>
  <summary>Installation using git</summary>

### Installation of WinchControllers using git

In most cases -- if you want to modify, tune and understand kite controllers -- it is better
to check out this project from git. You can do this with:
```bash
git clone https://github.com/opensourceawe/WinchControllers.jl.git
cd WinchControllers.jl
git checkout v0.5.0
```
For the checkout command, use the tag of the latest version.
</details>

### Provides
- a set of generic control components, see [Generic Components](https://opensourceawe.github.io/WinchControllers.jl/dev/components/#Generic-Components)
- a winch controller [WinchController](https://opensourceawe.github.io/WinchControllers.jl/dev/winchcontroller/), that limits the upper and lower force and controls the speed as function of the force
- a winch controller settings struct [WCSettings](https://opensourceawe.github.io/WinchControllers.jl/dev/winchcontroller/#WinchControllers.WCSettings) for the settings
- [Utility Functions and Macros](https://opensourceawe.github.io/WinchControllers.jl/dev/functions/)

### WinchController - Usage -
The WinchController combines the three controllers, mentioned above.
It can be operated in two modes of operation:
- speed control
- power production

In speed control mode it requires a set speed as input. Upper and lower force limits
are respected.
In power production mode it does not require any input but the measured tether force.
Output is the set speed of the asynchronous motor.

For a usage example look at the script [test_winchcontroller.jl](./test/test_winchcontroller.jl) .

## Licence
This project is licensed under the MIT License. Please see the below WAIVER in association with the license.

## WAIVER
Technische Universiteit Delft hereby disclaims all copyright interest in the package “KiteController.jl” (controllers for airborne wind energy systems) written by the Author(s).

Prof.dr. H.G.C. (Henri) Werij, Dean of Aerospace Engineering

## Scientific background
[A Methodology for the Design of Kite-Power Control Systems](https://research.tudelft.nl/en/publications/a-methodology-for-the-design-of-kite-power-control-systems)

## Donations
If you like this software, please consider donating to https://gofund.me/df0ae77d .

## Related
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=wind+Fechner&pageSize=50&ordering=rating&descending=true) for the scientific background of this code
- The meta package [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl) which contains all packages from Julia Kite Power Tools.
- the packages [KiteModels](https://github.com/ufechner7/KiteModels.jl) and [WinchModels](https://github.com/aenarete/WinchModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the packages [KiteViewers](https://github.com/aenarete/KiteViewers.jl) and [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
