"""
A collection of generic control components

Components:

- Integrator
- UnitDelay
- RateLimiter
- Mixer_2CH    two channel mixer
- Mixer_3CH    three channel mixer
"""

"""
    mutable struct Integrator

Discrete integrator with external reset.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct Integrator @deftype Float64
    dt                  # timestep [s]
    i           = 1.0   # integration constant
    output      = 0.0
    last_output = 0.0
end

"""
    function Integrator(dt, i=1.0, x0=0.0)

Constructor for discrete integrator with external reset.

# Parameters
- dt: time-step [s]
- i:  integration constant
- x0: initial and last output

# Returns
- a new struct of type `Integrator`
"""
function  Integrator(dt, i=1.0, x0=0.0)
    Integrator(dt, i, x0, x0)
end

"""
    reset(int::Integrator, x0=0.0)

Reset the integrator `int` to the value `x0`.

# Parameters
- int::Integrator: An integrator struct
- x0: default value =0.0; initial and last output

# Returns
- nothing
"""
function reset(int::Integrator, x0=0.0)
    int.output = x0
    int.last_output = x0
    nothing
end

"""
    calc_output(int::Integrator, input)

Calculate and return the output without updating `last_output`.

# Parameters

- int::Integrator: An integrator struct
- input: The input value

# Returns
- the output value
"""
function calc_output(int::Integrator, input)
    int.output = int.last_output + input * int.i * int.dt
end

"""
    on_timer(int::Integrator)

Update the field `last_output`. Must be called once per time-step.

# Parameters

- int::Integrator: An integrator struct

# Returns
- nothing
"""
function on_timer(int::Integrator)
    int.last_output = int.output
    nothing
end

"""
    mutable struct UnitDelay

UnitDelay, delay the input signal by one time step.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct UnitDelay @deftype Float64
    last_output = 0
    last_input = 0
end

"""
    calc_output(ud::UnitDelay, input)

Calculate and return the output and update the `last_input`, but not the `last_output`.

# Parameters
- int::UnitDelay: A `UnitDelay` struct
- input: The input value

# Returns
- the last output
"""
function calc_output(ud::UnitDelay, input)
    ud.last_input = input
    ud.last_output
end

"""
    on_timer(ud::UnitDelay)

Update the field `last_output`. Must be called once per time-step.

# Parameters

- ud::UnitDelay: A UnitDelay struct

# Returns
- nothing
"""
function on_timer(ud::UnitDelay)
    ud.last_output = ud.last_input
    nothing
end

"""
    reset(ud::UnitDelay)

Reset the `last_input` and `last_output` of the struct ud to zero.

# Parameters
- ud::UnitDelay: A `UnitDelay` struct

# Returns
- nothing
"""
function reset(ud::UnitDelay)
    ud.last_input = 0.0
    ud.last_output = 0.0
    nothing
end

"""
    mutable struct RateLimiter

Limit the rate of change of the output signal (return value of calc_output) to ± limit.
Unit of limit: 1/s

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct RateLimiter @deftype Float64
    dt = 0.05
    limit = 1
    output = 0
    last_output = 0
end

"""
    RateLimiter(dt, limit=1.0, x0=0.0)

## Parameters
- dt: the time-step [s]
- limit: the rate limit, default: 1.0 [1/s]
- x0: the initial output, default: 0.0

## Returns
- a struct of type RateLimiter
"""
function RateLimiter(dt, limit=1.0, x0=0.0)
    RateLimiter(dt, limit, x0, x0)
end

"""
    reset(rl::RateLimiter, x0=0.0)

Reset the `output` and `last_output` of the struct ud to `x0`.

# Parameters
- rl::RateLimiter: A `RateLimiter` struct
- x0: the initial value of the output signal

# Returns
- nothing
"""
function reset(rl::RateLimiter, x0=0.0)
    rl.output = x0
    rl.last_output = x0
end

"""
    calc_output(rl::RateLimiter, input)

Calculate and return the output, but not the `last_output`.

# Parameters
- rl::RateLimiter: A `RateLimiter` struct
- input: The input value

# Returns
- the new output
"""
function calc_output(rl::RateLimiter, input)
    if input - rl.last_output > rl.limit * rl.dt
        rl.output = rl.last_output + rl.limit * rl.dt
    elseif input - rl.last_output < -rl.limit * rl.dt
        rl.output = rl.last_output - rl.limit * rl.dt
    else
        rl.output = input
    end
    rl.output
end

"""
    on_timer(rl::RateLimiter)

Update the field `last_output`. Must be called once per time-step.

# Parameters
- rl::RateLimiter: A `RateLimiter` struct

# Returns
- nothing
"""
function on_timer(rl::RateLimiter)
    rl.last_output = rl.output
    nothing
end

"""
    mutable struct Mixer_2CH

Mix two analog inputs. Implements the following block diagram: ![mixer_2ch](assets/mixer_2ch.png).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct Mixer_2CH @deftype Float64
    dt
    t_blend = 1.0
    factor_b = 0
    select_b::Bool = false
end

"""
    Mixer_2CH(dt, t_blend)

Constructor for Mixer_2CH, a component for mixing two analog inputs.

## Parameters
- dt: the time-step [s]
- `t_blend`: the time for blending over from one channel to the other [s]

## Returns
- a struct of type Mixer_2CH
"""
function Mixer_2CH(dt, t_blend)
    Mixer_2CH(dt, t_blend, 0, false)
end

"""
    select_b(m2::Mixer_2CH, select_b::Bool)

Select input a or b.

## Parameters
- m2::Mixer_2CH: the two-channel mixer
- select_b: if true, select channel b, otherwise select channel a

## Returns
- nothing
"""
function select_b(m2::Mixer_2CH, select_b::Bool)
    m2.select_b = select_b
    nothing
end

function on_timer(m2::Mixer_2CH)
    if m2.select_b
        integrator_in = 1.0 / m2.t_blend
    else
        integrator_in = -1.0 / m2.t_blend
    end
    m2.factor_b += integrator_in * m2.dt
    m2.factor_b = saturate(m2.factor_b, 0, 1)
end

"""
    calc_output(m2::Mixer_2CH, input_a, input_b)

Calculate and return the output of the two channel mixer.

# Parameters

- m2::Mixer_2CH: a two channel mixer component
- input_a: input value of channel A
- input_b: input value of channel B

# Returns
- the output value
"""
function calc_output(m2::Mixer_2CH, input_a, input_b)
    input_b * m2.factor_b + input_a * (1.0 - m2.factor_b)
end

"""
    mutable struct Mixer_3CH

Mix three analog inputs. Implements the following block diagram: ![mixer_3ch](assets/mixer_3ch.png).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct Mixer_3CH @deftype Float64
    dt
    t_blend = 1.0
    factor_b = 0
    factor_c = 0
    select_b::Bool = false
    select_c::Bool = false
end

function Mixer_3CH(dt, t_blend = 1.0)
    Mixer_3CH(dt, t_blend, 0, 0, false, false)
end

function on_timer(m3::Mixer_3CH)
    # calc output of integrator b
    if m3.select_b
        integrator_b_in = 1.0 / m3.t_blend
    else
        integrator_b_in = -1.0 / m3.t_blend
    end
    m3.factor_b += integrator_b_in * m3.dt
    m3.factor_b = saturate(m3.factor_b, 0, 1)
    # calc output of integrator c
    if m3.select_c
        integrator_c_in = 1.0 / m3.t_blend
    else
        integrator_c_in = -1.0 / m3.t_blend
    end
    m3.factor_c += integrator_c_in * m3.dt
    m3.factor_c = saturate(m3.factor_c, 0, 1)  
    nothing
end

"""
    select_b(m3::Mixer_3CH, select_b::Bool)

Make input b the active input.

To make channel a the active input call:
```julia
select_b(m3, false)
select_c(m3, false)
```

## Parameters
- m3::Mixer_3CH: the three-channel mixer
- select_b: if true, select channel b

## Returns
- nothing
"""
function select_b(m3::Mixer_3CH, select_b::Bool)
    m3.select_b = select_b
    if select_b
        select_c(m3, false)
    end
end

"""
    select_c(m3::Mixer_3CH, select_c::Bool)

Make input c the active input.

## Parameters
- m3::Mixer_3CH: the three-channel mixer
- select_c: if true, select channel c

## Returns
- nothing
"""
function select_c(m3::Mixer_3CH, select_c::Bool)
    m3.select_c = select_c
    if select_c
        select_b(m3, false)
    end
    nothing
end

"""
    calc_output(m3::Mixer_3CH, input_a, input_b, input_c)

Calculate and return the output of the three channel mixer.

# Parameters
- m3::Mixer_3CH: a three channel mixer component
- input_a: input value of channel A
- input_b: input value of channel B
- input_c: input value of channel C

# Returns
- the output value
"""
function calc_output(m3::Mixer_3CH, input_a, input_b, input_c)
    input_b * m3.factor_b + input_c * m3.factor_c + input_a * (1.0 - m3.factor_b - m3.factor_c)
end

"""
    function get_state(m3::Mixer_3CH)

Return the controller state as integer.

Returns:
- wcsLowerForceControl = 0 # input b selected 
- wcsSpeedControl = 1      # input a selected
- wcsUpperForceControl = 2 # input c selected
"""
function get_state(m3::Mixer_3CH)
    Int(! m3.select_b && ! m3.select_c) + 2 * Int(m3.select_c)
end
