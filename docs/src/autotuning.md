```@meta
CurrentModule = WinchControllers
```

# Autotuning

## Introduction
The winch controller has 34 parameters (see [Winchcontroller Settings](@ref)). To determine and optimize all of them manually is time consuming and error prone, in particular if the physical system is changing. Therefore some form of automated tuning is desirable.

## Methodology
When the [Performance Indicators](@ref) are defined, an optimizer can be used to determine the controller parameters. The controller is using three PID controllers and a mixer. The following parameters are tuned automatically:
```julia
    wcs.i_speed  # the speed controller gain
    wcs.p_speed  # the speed controller proportional gain
    wcs.t_blend  # the blending time for switching between controllers
    wcs.pf_low   # the lower force controller gain
    wcs.if_low   # the lower force controller proportional gain
    wcs.pf_high  # the upper force controller gain
    wcs.if_high  # the upper force controller integral gain
    wcs.df_high  # the differential gain of the upper force controller
```
The global, blackbox optimizer package NOMAD is used for the optimization process, together with a test case that mimics extreme wind conditions.

## Example

