```@meta
CurrentModule = WinchControllers
```
# Generic Components

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
Mixer_2CH(dt, t_blend)
select_b(m2::Mixer_2CH, select_b::Bool)
calc_output(m2::Mixer_2CH, input_a, input_b)
```

## Mixer_3CH
```@docs
Mixer_3CH
get_state(m3::Mixer_3CH)
select_b(m3::Mixer_3CH, select_b::Bool)
select_c(m3::Mixer_3CH, select_c::Bool)
calc_output(m3::Mixer_3CH, input_a, input_b, input_c)
```