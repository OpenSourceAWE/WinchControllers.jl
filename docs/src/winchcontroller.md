```@meta
CurrentModule = WinchControllers
```
## Introduction
For a kite power system, the reel-out speed of the winch must be controlled such that the maximal tether force is never exceeded, while the reel-out speed should be optimized for maximal power over the full cycle at wind speeds below rated wind speed. To keep the kite controllable, also a minimal tether force limit has to be kept. Depending on the mode of operation, one of the following three controllers is used:

## Enum WinchControllerState
```@docs
WinchControllerState
```

## CalcVSetIn
```@docs
CalcVSetIn
CalcVSetIn(wcs::WCSettings)
set_vset_pc(cvi::CalcVSetIn, v_set_pc, force)
calc_output(cvi::CalcVSetIn)
on_timer(cvi::CalcVSetIn)
```

## SpeedController
```@docs
SpeedController
SpeedController(wcs::WCSettings)
set_inactive(sc::SpeedController, inactive::Bool)
set_v_act(sc::SpeedController, v_act)
set_v_set(sc::SpeedController, v_set)
set_v_set_in(sc::SpeedController, v_set_in)
set_tracking(sc::SpeedController, tracking)
get_v_set_out(sc::SpeedController)
get_v_err(sc::SpeedController)
on_timer(sc::SpeedController)
```

## AbstractForceController
```@docs
set_v_act(fc::AFC, v_act)
set_force(fc::AFC, force)
set_reset(fc::AFC, reset)
set_f_set(fc::AFC, f_set)
set_v_sw(fc::AFC, v_sw)
set_tracking(fc::AFC, tracking)
get_v_set_out(fc::AFC)
get_f_err(fc::AFC)
```

## LowerForceController
```@docs
LowerForceController
LowerForceController(wcs::WCSettings)
get_f_set_low(lfc::LowerForceController)
on_timer(lfc::LowerForceController)
```

## UpperForceController
```@docs
UpperForceController
UpperForceController(wcs::WCSettings)
get_f_set_upper(ufc::UpperForceController)
on_timer(ufc::UpperForceController)
```

## WinchController
```@docs
WinchController
WinchController(wcs::WCSettings)
calc_v_set(wc::WinchController, v_act, force, f_low; v_set_pc=nothing)
get_set_force(wc::WinchController)
get_state(wc::WinchController)
get_status(wc::WinchController)
get_v_err(wc::WinchController)
on_timer(wc::WinchController)
```

## Controller Settings
```@docs
WCSettings
calc_vro
```

## Logger
```@docs
WCLogger
WCLogger(duration, dt)
log(logger::WCLogger; params...)
f_err
v_err
gamma
```

## Winch
Only used for testing the [WinchController](@ref).

```@docs
Winch
Winch(wcs::WCSettings, set::Settings)
set_v_set(w::Winch, v_set)
set_force(w::Winch, force)
get_speed(w::Winch)
get_acc(w::Winch)
on_timer(w::Winch)
```