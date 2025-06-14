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
function find_equilibrium_speed(winch, set_speed, force, n=2000)
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
end

function motor_dynamics(x, u)
    # x: state vector, e.g., [v_act]
    # u: input vector, e.g., [v_set, force]
    v_act = x[1]
    v_set, force = u[1], u[2]
    acc = calc_acceleration(winch.wm, v_act, force; set_speed = v_set)
    return [acc]
end

v_set = 4.0
force = 4000.0
v_act = find_equilibrium_speed(winch, v_set, force)

x0 = [v_act]         # State at operating point
u0 = [v_set, force]  # Input at operating point

# System matrix A = ∂f/∂x at operating point
@time A = finite_difference_jacobian(x -> motor_dynamics(x, u0), x0)

# Input matrix B = ∂f/∂u at operating point
B = finite_difference_jacobian(u -> motor_dynamics(x0, u), u0)

C = [1.0]
D = [0.0 0.0]

sys = ss(A, B, C, D)
sys_new = ss(A, B[:, 1], C, D[:, 1])

bode_plot(sys_new; from=0.76, to=2.85, title="Linearized Winch, F=$force N")
