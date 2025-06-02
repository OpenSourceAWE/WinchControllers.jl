# Linearize the closed loop system consisting of the winch, kite and upper force controller.

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, WinchModels, KiteUtils, ControlPlots, ControlSystemsBase, FiniteDiff, RobustAndOptimalControl
import FiniteDiff: finite_difference_jacobian

if isfile("data/system_tuned.yaml")
    set = load_settings("system_tuned.yaml")
else
    set = load_settings("system.yaml")
end
wcs = WCSettings(dt=0.02)
update(wcs)
wcs.test = true

winch = Winch(wcs, set)

@info "Testing stability of the upper force controller (UFC) with the winch and kite system."

# find equilibrium speed
function find_equilibrium_speed(winch, set_speed, force, n=10000)
    last_v_act = 0.0
    for v_set in range(0.0, 2*set_speed, n)
        lim_speed = minimum([v_set, set_speed])
        set_force(winch, force)
        set_v_set(winch, lim_speed)
        v_act = get_speed(winch)
        on_timer(winch)
        if v_set > 0 && abs(v_act - last_v_act) < 1e-6
            return v_act
        end
        last_v_act = v_act
    end
    set_v_set(winch, set_speed)
    on_timer(winch)
    @error "Failed to find equilibrium speed"
end

function motor_dynamics(x, u)
    # x: state vector, e.g., [v_act]
    # u: input vector, e.g., [v_set, force]
    v_act = x[1]
    v_set, force = u[1], u[2]
    acc = calc_acceleration(winch.wm, v_act, force; set_speed = v_set)
    return [acc]
end

function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

# create the upper force controller
# Tf​ can typically be chosen as Ti/NT for a PI controller and Td/N for a PID controller, and N is commonly in the range 2 to 20. 
# Td = wcs.df_high / wcs.pf_high Kd/Kp
function upper_force_controller(wcs)
    # wcs: WCSettings object
    Td = wcs.df_high / wcs.pf_high
    N = wcs.nf_high
    Tf = Td / N
    C = pid(wcs.pf_high, wcs.if_high, wcs.df_high; form=:parallel, filter_order=1, state_space=true, Tf)
    return C
end

function system_dynamics(x, u)
    # x: state vector, e.g., [v_act]
    # u: input vector, e.g., [v_set, v_wind]
    v_act = x[1]
    v_set, v_wind = u[1], u[2]
    force = calc_force(v_wind, v_act)
    acc = calc_acceleration(winch.wm, v_act, force; set_speed = v_set)
    return [acc]
end

function linearize(winch, v_set, v_wind)
    force = calc_force(v_wind, v_set)
    v_act = find_equilibrium_speed(winch, v_set, force)
    x0 = [v_act]          # State at operating point
    u0 = [v_set, v_wind]  # Input at operating point
    A = finite_difference_jacobian(x -> system_dynamics(x, u0), x0)
    B = finite_difference_jacobian(u -> system_dynamics(x0, u), u0)
    C = [1.0]
    D = [0.0 0.0]
    force = calc_force(v_wind, v_act)
    siso_sys = ss(A, B[:, 1], C, D[:, 1]) * force/v_act
end

function open_loop_system(winch, v_set, v_wind)
    # Create the open loop system with the upper force controller
    C = upper_force_controller(winch.wcs)
    sys = linearize(winch, v_set, v_wind)
    # sys_open = feedback(C * sys, 1.0; sign=-1)
    return C * sys
end

function margins()
    margins = []
    for v_wind in range(1, 9, length=9)
        local v_set, dm
        v_set = 0.57*v_wind
        sys = open_loop_system(winch, v_set, v_wind)
        dm = diskmargin(sys)
        push!(margins, dm.margin)
    end
    min_margin = minimum(margins)
    if min_margin < 0.3
        @error "System is unstable with a minimum margin of: $min_margin"
    elseif min_margin < 0.5
        @warn "System is marginally stable with a minimum margin of: $min_margin"
    else
        @info "System is stable with a minimum margin of: $(round(min_margin, digits=2))"
    end
    return margins
end
margins()

for v_wind in range(7.5, 9, length=3)
    global sys
    local v_set
    v_set = 0.57*v_wind
    sys = open_loop_system(winch, v_set, v_wind)
    bode_plot(sys; from=0.76, to=2.85, title="System with UFC, v_wind=7.5..9 m/s")
end
nothing
