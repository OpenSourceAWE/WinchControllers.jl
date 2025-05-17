```@meta
CurrentModule = WinchControllers
```
## Integrator
```@docs
Integrator
Integrator(dt, i=1.0, x0=0.0)
reset(int::Integrator, x0=0.0)
calc_output(int::Integrator, input)
on_timer(int::Integrator)
```

## UnitDelay
```@docs
UnitDelay
reset(ud::UnitDelay)
calc_output(ud::UnitDelay, input)
on_timer(ud::UnitDelay)
```

## CalcVSetIn
```@docs
set_vset_pc(cvi::CalcVSetIn, v_set_pc, force)
calc_output(cvi::CalcVSetIn)
```