"""
    mutable struct WinchController

Winch controller using Model Predictive Control (MPC) for speed and force control, incorporating wind speed as a disturbance.

# Fields
- `wcs::WCSettings`: Winch controller settings.
- `set::Settings`: System settings loaded from configuration.
- `winch::TorqueControlledMachine`: Winch model.
- `model::NonLinModel`: Nonlinear model for control.
- `plant::NonLinModel`: Nonlinear plant model for simulation.
- `mpc::LinMPC`: Linear MPC controller.
- `time::Float64`: Current simulation time.
- `v_act::Float64`: Actual reel-out speed (m/s).
- `force::Float64`: Actual tether force (N).
- `v_set::Float64`: Set velocity output to the winch (m/s).
- `v_set_in::Float64`: Input set velocity to the controller (m/s).
- `kv::Float64`: Velocity-force ratio setpoint.
"""
@with_kw mutable struct WinchController @deftype Float64
    wcs::WCSettings
    set::Settings
    winch::TorqueControlledMachine
    model::NonLinModel
    plant::NonLinModel
    mpc::LinMPC
    time = 0.0
    v_act = 0.0
    force = 0.0
    v_set = 0.0
    v_set_in = 0.0
    kv = wcs.kv
end

"""
    WinchController(wcs::WCSettings, set::Settings)

Constructor for a WinchController using MPC.

# Arguments
- `wcs::WCSettings`: Winch controller settings.
- `set::Settings`: System settings.

# Returns
- A `WinchController` instance.
"""
function WinchController(wcs::WCSettings, set::Settings)
    winch = TorqueControlledMachine(set)
    
    # Simplified kite force model
    calc_force(v_wind, v_ro) = (v_wind - v_ro)^2 * 4000.0 / 16.0
    
    # State and output equations
    function f!(dx, x, u, d, _)
        v = x[1]
        τ = u[1]
        v_wind = d[1]
        F = calc_force(v_wind, v)
        a = calc_acceleration(winch, v, F; set_torque=τ)
        dx[1] = a
        nothing
    end
    
    function h!(y, x, d, _)
        v = x[1]
        v_wind = d[1]
        y[1] = v
        y[2] = calc_force(v_wind, v)
        y[3] = v / sqrt(y[2] + 1e-8)
        nothing
    end
    
    # Model parameters
    nu, nx, ny, nd, Ts = 1, 1, 3, 1, wcs.dt
    vu, vx, vy, vd = ["\$τ\$ (Nm)"], ["v (m/s)"], ["v (m/s)", "F (N)", "kv (-)"], ["v_wind (m/s)"]
    model = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd); u=vu, x=vx, y=vy, d=vd)
    plant = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd); u=vu, x=vx, y=vy, d=vd)
    
    # Linearized model and Kalman filter
    linmodel = linearize(model, x=[0], u=[0], d=[0])
    estim = KalmanFilter(linmodel; i_ym=[1], σQ=[0.05], σR=[0.5], nint_u=[1], σQint_u=[0.1], nint_ym=[1])
    
    # MPC setup
    Hp, Hc, Mwt, Nwt = 20, 2, [0.0, 0.0, 1e5], [0.01]
    umin, umax = [-wcs.f_high*set.drum_radius/set.gear_ratio], [wcs.f_high*set.drum_radius/set.gear_ratio]
    ymin, ymax = [-Inf, wcs.f_low, -Inf], [Inf, wcs.f_high, Inf]
    mpc = LinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt=1e6)
    mpc = setconstraint!(mpc; umin, umax, ymin, ymax)
    
    WinchController(wcs=wcs, set=set, winch=winch, model=model, plant=plant, mpc=mpc)
end

"""
    calc_v_set(wc::WinchController, v_act, force, v_wind)

Calculate the set velocity for the winch using MPC.

# Arguments
- `wc::WinchController`: The winch controller instance.
- `v_act`: Actual reel-out speed (m/s).
- `force`: Measured tether force (N).
- `v_wind`: Wind speed disturbance (m/s).

# Returns
- The calculated set velocity (m/s).
"""
function calc_τ_set(wc::WinchController, v_act, force, v_wind)
    wc.v_act = v_act
    wc.force = force
    d = [v_wind]
    y = [v_act, force, v_act / sqrt(force + 1e-8)]
    ry = [sqrt(force) * wc.kv, force, wc.kv]
    
    # Update state and compute control input
    x̂ = preparestate!(wc.mpc, y[[1]], d)
    u = moveinput!(wc.mpc, ry, d)
    
    # Update linearized model
    linmodel = linearize(wc.model; u, x=x̂[1], d)
    setmodel!(wc.mpc, linmodel)
    
    # Update state
    updatestate!(wc.mpc, u, y[[1]], d)
    updatestate!(wc.plant, u, d)
    
    wc.v_set = u[1] / wc.set.drum_radius * wc.set.gear_ratio
    wc.v_set_in = ry[1]
    wc.v_set
end

"""
    on_timer(wc::WinchController)

Update the controller state based on a timer event.

# Arguments
- `wc::WinchController`: The winch controller instance.
"""
function on_timer(wc::WinchController)
    wc.time += wc.wcs.dt
end

"""
    get_state(wc::WinchController)

Get the current state of the winch controller.

# Returns
- `WinchControllerState`: The current state (e.g., wcsSpeedControl).
"""
function get_state(wc::WinchController)
    wc.state
end

"""
    get_status(wc::WinchController)

Retrieve the controller status for logging.

# Returns
- Array containing: reset (false), active (true), force, f_set (0.0), v_set_out, v_set_lfc (0.0), v_set_ufc (0.0).
"""
function get_status(wc::WinchController)
    [false, true, wc.force, 0.0, wc.v_set, 0.0, 0.0]
end

"""
    get_v_err(wc::WinchController)

Compute the velocity error.

# Returns
- Velocity error (m/s) or NaN if not applicable.
"""
function get_v_err(wc::WinchController)
    abs(wc.v_set_in - wc.v_act)
end

"""
    get_f_err(wc::WinchController)

Compute the force error.

# Returns
- Force error (N) or NaN if not applicable.
"""
function get_f_err(wc::WinchController)
    NaN
end

"""
    get_v_set(wc::WinchController)

Get the set velocity.

# Returns
- Set velocity (m/s) or NaN if not applicable.
"""
function get_v_set(wc::WinchController)
    wc.v_set
end

"""
    get_v_set_in(wc::WinchController)

Get the input set velocity.

# Returns
- Input set velocity (m/s) or NaN if not applicable.
"""
function get_v_set_in(wc::WinchController)
    wc.v_set_in
end

"""
    sim_adapt!(wc::WinchController, lg::WCLogger, V_WIND::Vector{Float64})

Simulate the winch controller with adaptive MPC.

# Arguments
- `wc::WinchController`: The winch controller instance.
- `lg::WCLogger`: Logger for simulation data.
- `V_WIND`: Vector of wind speed disturbances.

# Returns
- `SimResult`: Simulation results containing input, output, and state data.
"""
function sim_adapt!(wc::WinchController, lg::WCLogger, V_WIND::Vector{Float64})
    N = length(lg)
    U_data = zeros(wc.plant.nu, N)
    Y_data = zeros(wc.plant.ny, N)
    D_data = zeros(wc.plant.nd, N)
    Ry_data = zeros(wc.plant.ny, N)
    X̂_data = zeros(wc.mpc.estim.nx̂, N)
    X_data = zeros(wc.plant.nx, N)
    
    setstate!(wc.plant, [0.0])
    initstate!(wc.mpc, [0], wc.plant([0])[1], [0])
    setstate!(wc.mpc, zeros(wc.mpc.estim.nx̂))
    
    for i = 1:N
        v_wind = V_WIND[i]
        v_act = get_speed(wc.winch)
        force = calc_force(v_wind, v_act)
        set_force(wc.winch, force)
        
        v_set = calc_v_set(wc, v_act, force, v_wind)
        set_v_set(wc.winch, v_set)
        
        on_timer(wc.winch)
        on_timer(wc)
        
        status = get_status(wc)
        U_data[:,i] = [v_set * wc.set.drum_radius / wc.set.gear_ratio]
        Y_data[:,i] = [v_act, force, v_act / sqrt(force + 1e-8)]
        D_data[:,i] = [v_wind]
        Ry_data[:,i] = [sqrt(force) * wc.kv, force, wc.kv]
        X̂_data[:,i] = wc.mpc.estim.x̂0
        X_data[:,i] = wc.plant.x0
        
        log(lg; v_ro=v_act, acc=get_acc(wc.winch), state=Int(get_state(wc)), reset=status[1], active=status[2],
            force=status[3], jerk=wc.winch.jerk, f_set=status[4], f_err=get_f_err(wc), v_err=get_v_err(wc),
            v_set=get_v_set(wc), v_set_out=v_set, v_set_in=get_v_set_in(wc))
    end
    
    SimResult(wc.mpc, U_data, Y_data, D_data; Ry_data, X̂_data, X_data)
end
