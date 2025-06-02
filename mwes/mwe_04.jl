# Linearize the winch
# input: set_speed
# output: speed
# Same as mwe_02.jl, but using FiniteDiff.jl for Jacobian calculation

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, WinchModels, KiteUtils, ControlPlots, ControlSystemsBase, FiniteDiff
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

function linearize(winch, v_set, force)
    v_act = find_equilibrium_speed(winch, v_set, force)
    x0 = [v_act]         # State at operating point
    u0 = [v_set, force]  # Input at operating point
    A = finite_difference_jacobian(x -> motor_dynamics(x, u0), x0)
    B = finite_difference_jacobian(u -> motor_dynamics(x0, u), u0)
    C = [1.0]
    D = [0.0 0.0]
    siso_sys = ss(A, B[:, 1], C, D[:, 1])
end

v_set = 4.0
# for force in range(300.0, 3800.0, length=10)
#     @info "Linearizing for force: $force N"
#     sys_new = linearize(winch, v_set, force)
#     # @info "System: $sys_new"
#     # @info "Eigenvalues: $(eigvals(sys_new))"
#     bode_plot(sys_new; from=0.76, to=2.85, title="Linearized Winch, F=300..3800 N")
# end

sys_new = linearize(winch, v_set, 1000.0)
bode_plot(sys_new; from=0.76, to=2.85, title="Linearized Winch, F=1000 N")
