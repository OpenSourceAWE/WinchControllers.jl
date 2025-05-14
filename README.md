# WinchControllers

[![Build Status](https://github.com/ufechner7/WinchControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/ufechner7/WinchControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)

Discrete controllers for Winches.

This package is part of Julia Kite Power Tools, which consists of the following packages:
<p align="center"><img src="./docs/kite_power_tools.png" width="500" /></p>

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
git clone https://github.com/aenarete/WinchControllers.jl.git
cd WinchControllers.jl
git checkout v0.1.0
```
For the checkout command, use the tag of the latest version.
</details>