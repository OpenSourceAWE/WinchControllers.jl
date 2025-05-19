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
```