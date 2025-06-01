# Linearize the winch
# input: set_speed
# output: speed

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, WinchModels, KiteUtils, ControlPlots, ControlSystemsBase

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
    TIME = Float64[]
    V_SET = Float64[]
    V_ACT = Float64[]
    time = 0.0
    last_v_act = 0.0
    # slow start
    for v_set in range(0.0, 2*set_speed, n)
        lim_speed = minimum([v_set, set_speed])
        set_force(winch, force)
        set_v_set(winch, lim_speed)
        v_act = get_speed(winch)
        push!(V_SET, lim_speed)  
        push!(V_ACT, v_act)      
        on_timer(winch)
        push!(TIME, time)
        time += wcs.dt
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

v_set = 8.0
force = 2000.0
v_act = find_equilibrium_speed(winch, v_set, force)

x0 = [v_act]         # State at operating point
u0 = [v_set, force]  # Input at operating point

function finite_jacobian(f, x; ϵ=sqrt(eps(eltype(x))))
    n = length(x)
    m = length(f(x))
    J = zeros(m, n)
    for i in 1:n
        Δx = zeros(n)
        Δx[i] = ϵ
        J[:, i] = (f(x + Δx) - f(x - Δx)) ./ (2ϵ)
    end
    J
end

A = finite_jacobian(x -> motor_dynamics(x, u0), x0)
B = finite_jacobian(u -> motor_dynamics(x0, u), u0)
C = [1.0]
D = [0.0 0.0]

sys = ss(A, B, C, D)

# 1. Extract B matrices
B1 = B[:, 1]   # First input column (active)

# 2. Create new system with only the first input
sys_new = ss(A, B1, C, D[:, 1])

bode_plot(sys_new; title="Bode Plot of Linearized Winch System")
