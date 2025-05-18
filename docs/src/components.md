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

## RateLimiter
```@docs
RateLimiter
RateLimiter(dt, limit=1.0, x0=0.0)
reset(rl::RateLimiter, x0=0.0)
calc_output(rl::RateLimiter, input)
on_timer(rl::RateLimiter)
```

## Mixer_2CH
```@docs
Mixer_2CH
```

## Mixer_3CH
```@docs
Mixer_3CH
```