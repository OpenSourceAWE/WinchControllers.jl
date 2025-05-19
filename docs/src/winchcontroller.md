```@meta
CurrentModule = WinchControllers
```

## CalcVSetIn
```@docs
CalcVSetIn
CalcVSetIn(wcs::WCSettings)
set_vset_pc(cvi::CalcVSetIn, v_set_pc, force)
calc_output(cvi::CalcVSetIn)
on_timer(cvi::CalcVSetIn)
```

## Speed Controller
```@docs
SpeedController
SpeedController(wcs::WCSettings)
set_inactive(sc::SpeedController, inactive::Bool)
set_v_act(sc::SpeedController, v_act)
set_v_set(sc::SpeedController, v_set)
set_v_set_in(sc::SpeedController, v_set_in)
set_tracking(sc::SpeedController, tracking)
get_v_set_out(sc::SpeedController)
on_timer(sc::SpeedController)
```

## Winch Controller
```@docs
WinchController
```

## Controller Settings
```@docs
WCSettings
calc_vro
```

## Winch
Only used for testing the [WinchController](@ref).

```@docs
Winch
Winch(wcs::WCSettings, set::Settings)
set_v_set
set_force
get_speed(w::Winch)
get_acc(w::Winch)
on_timer(w::Winch)
```