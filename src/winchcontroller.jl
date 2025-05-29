# Winch winch controller component, implemented as described in the PhD thesis of Uwe Fechner. 
"""
    @enum WinchControllerState

The three values that tell us which sub-controller is active.
- wcsLowerForceLimit = 0
- wcsSpeedControl = 1
- wcsUpperForceLimit = 2
"""
@enum WinchControllerState wcsLowerForceLimit=0 wcsSpeedControl wcsUpperForceLimit

"""
    mutable struct WinchController

Basic winch controller. Works in one of the three modes `wcsLowerForceLimit`, `wcsSpeedControl` and
`wcsUpperForceLimit`.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct WinchController @deftype Float64
    wcs::WCSettings
    time = 0
    v_set_pc::Union{Float64, Nothing} = nothing # last set value from the position controller (can be nothing)
    v_set_in = 0.0  # input of the speed controller
    v_set_out = 0.0 # output of the speed controller
    v_set_ufc = 0.0 # output of the upper force controller
    v_set_lfc = 0.0 # output of the lower force controller
    v_set = 0.0     # output of the winch controller, going to the motor-controller/ model
    v_act = 0.0     # actual, measured speed
    force = 0.0     # actual, measured force
    calc::CalcVSetIn = CalcVSetIn(wcs)
    mix3::Mixer_3CH  = Mixer_3CH(wcs.dt, wcs.t_blend)
    sc::SpeedController = SpeedController(wcs)
    lfc::LowerForceController = LowerForceController(wcs)
    ufc::UpperForceController = UpperForceController(wcs)
end

"""
    WinchController(wcs::WCSettings)

Constructor for a WinchController, based on the winch controller settings.

## Parameters
- wcs::[WCSettings](@ref): the winch controller settings struct

## Returns
- a struct of type [WinchController](@ref)
"""
function WinchController(wcs::WCSettings)
    wc = WinchController(wcs=wcs)
    set_f_set(wc.lfc, wcs.f_low)
    set_reset(wc.lfc, true)
    set_v_sw(wc.lfc, -1.0)
    set_f_set(wc.ufc, wcs.f_high)
    set_v_sw(wc.ufc, calc_vro(wcs, wc.ufc.f_set))
    set_reset(wc.ufc, true)
    set_reset(wc.ufc, false)
    wc
end

"""
    calc_v_set(wc::WinchController, v_act, force, f_low; v_set_pc=nothing)

Calculate the set velocity (`v_set`) for the winch.

# Arguments
- `wc::WinchController`: The winch controller instance.
- `v_act`: The actual velocity of the winch.
- `force`: The measured or estimated force on the winch.
- `f_low`: The lower force threshold.
- `v_set_pc`: (optional) Precomputed or externally provided set velocity. Defaults to `nothing`.

# Returns
- The calculated set velocity for the winch.

# Notes     
- The function logic depend on the relationship between the actual force and the lower force threshold.
- If `v_set_pc` is provided, it overrides the computed set velocity.
"""
function calc_v_set(wc::WinchController, v_act, force, f_low, v_set_pc=nothing)
    set_f_set(wc.lfc, f_low)
    wc.v_act = v_act
    wc.force = force
    set_vset_pc(wc.calc, v_set_pc, wc.force)
    v_set_in = calc_output(wc.calc)
    # set the inputs of sc
    set_v_set_in(wc.sc, v_set_in)
    set_v_act(wc.sc, v_act)
    if wc.time <= wc.wcs.t_startup
        reset = true
    else
        reset = false
    end
    # set the inputs of lfc and ufc    
    set_reset(wc.lfc, reset)
    set_v_sw(wc.lfc, calc_vro(wc.wcs, wc.lfc.f_set) * 1.05)
    set_v_sw(wc.ufc, calc_vro(wc.wcs, wc.ufc.f_set) * 0.95)
    set_v_act(wc.lfc, v_act)
    set_v_act(wc.ufc, v_act)
    # set tracking and force for all controllers
    set_tracking(wc.sc, wc.v_set)
    set_tracking(wc.lfc, wc.v_set)
    set_tracking(wc.ufc, wc.v_set)
    set_force(wc.lfc, force)
    set_force(wc.ufc, force)
    # activate or deactivate the speed controller
    set_inactive(wc.sc, (wc.lfc.active || wc.ufc.active) && isnothing(v_set_pc)) 
    if ! isnothing(v_set_pc)
        wc.lfc.active=false
        wc.ufc.active=false
    end   
    # calculate the output, using the mixer
    select_b(wc.mix3, wc.lfc.active)
    select_c(wc.mix3, wc.ufc.active)
    v_set_out_A = get_v_set_out(wc.sc)
    v_set_out_B = get_v_set_out(wc.lfc)
    v_set_out_C = get_v_set_out(wc.ufc)
    wc.v_set = calc_output(wc.mix3, v_set_out_A, v_set_out_B, v_set_out_C)
    wc.v_set_out = v_set_out_A # for logging, store the output of the speed controller
    wc.v_set
end

"""
    on_timer(wc::WinchController)

Callback function that is triggered periodically by a timer event. This function is responsible for handling time-based 
updates or actions for the given `WinchController` instance `wc`.

# Arguments
- `wc::WinchController`: The winch controller instance to be updated.

# Returns
- Nothing. This function is called for its side effects.

"""
function on_timer(wc::WinchController)
    wc.time += wc.wcs.dt
    on_timer(wc.calc)
    on_timer(wc.sc)
    on_timer(wc.lfc)
    on_timer(wc.ufc)
    on_timer(wc.mix3)    
end

"""
    get_state(wc::WinchController) -> @enum WinchControllerState

Returns the current state of the given `WinchController` instance `wc`. The returned value typically represents the operational state or status of the winch controller, such as position, speed, or error status.

# Arguments
- `wc::WinchController`: The winch controller object whose state is to be retrieved.

# Returns
- @enum [WinchControllerState](@ref)
"""
function get_state(wc::WinchController)
    get_state(wc.mix3)
end

"""
    get_set_force(wc::WinchController)

Returns the set force value of the `WinchController` instance `wc`.

# Arguments
- `wc::WinchController`: The winch controller object for which the set force is to be retrieved.

# Returns
- The set force value, or `nothing` if the state is not `wcsLowerForceLimit` or `wcsUpperForceLimit`.

"""
function get_set_force(wc::WinchController)
    state = get_state(wc)
    if state == Int(wcsLowerForceLimit)
        return wc.lfc.f_set 
    elseif state == Int(wcsUpperForceLimit)
        return wc.ufc.f_set
    else
        return nothing
    end
end

"""
    get_status(wc::WinchController)

Retrieve the current status of the given `WinchController` instance for logging and debugging purposes.

# Arguments
- `wc::WinchController`: The winch controller object whose status is to be retrieved.

# Returns
- The current status of the winch controller, an array containing:
    - `reset`: Boolean indicating if the controller is in reset state.
    - `active`: Boolean indicating if the controller is active.
    - `force`: The current set force value or zero if not set.
    - `f_set`: The set force value.
    - `v_set_out`: The output velocity set by the speed controller.
    - `v_set_ufc`: The output velocity set by the upper force controller.
    - `v_set_lfc`: The output velocity set by the lower force controller.
"""
function get_status(wc::WinchController)
    f_set = get_set_force(wc)
    if isnothing(f_set)
        f_set = 0.0
    end
    result = [false, false, 0.0, 0.0, 0.0, 0.0, 0.0]
    result[1] = wc.ufc.reset
    result[2] = wc.ufc.active
    result[3] = wc.ufc.force
    result[4] = f_set
    result[5] = wc.sc.v_set_out
    result[6] = wc.lfc.v_set_out
    result[7] = wc.ufc.v_set_out
    result
end

"""
    get_v_err(wc::WinchController)

Compute and return the velocity error for the given `WinchController` instance `sc`.

# Arguments
- wc::[WinchController](@ref): The winch controller object for which the velocity error is to be calculated.

# Returns
- The velocity error `v_err` [m/s]. 
  If the speed controller is inactive, it returns `NaN`.
"""
function get_v_err(wc::WinchController)
    get_v_err(wc.sc)
end

function get_f_err(wc::WinchController)
    state = get_state(wc)
    if state == Int(wcsUpperForceLimit)
        return wc.ufc.f_err
    elseif state == Int(wcsLowerForceLimit) 
        return wc.lfc.f_err
    else
        return NaN
    end
end

function get_v_set(wc::WinchController)
    state = get_state(wc)
    if state == Int(wcsSpeedControl)
        return wc.v_set
    else
        return NaN
    end
end

function get_v_set_in(wc::WinchController)
    state = get_state(wc)
    if state == Int(wcsSpeedControl)
        return wc.sc.v_set_in
    else
        return NaN
    end
end
