# Determine the steady-state speed of a winch
# input: set_speed
# output: speed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, KiteUtils, ControlPlots

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
function find_equilibrium_speed(winch, set_speed, force, n=100)
    TIME = Float64[]
    V_SET = Float64[]
    V_ACT = Float64[]
    time = 0.0
    # slow start
    for v_set in range(0.0, set_speed, n)
        set_force(winch, force)
        set_v_set(winch, v_set)
        v_act = get_speed(winch)
        push!(V_SET, v_set)  
        push!(V_ACT, v_act)      
        on_timer(winch)
        push!(TIME, time)
        time += wcs.dt
    end
    set_v_set(winch, set_speed)
    on_timer(winch)
    return TIME, V_SET, V_ACT
end

TIME, V_SET, V_ACT = find_equilibrium_speed(winch, 5.0, 1000.0)
plot(TIME, [V_SET, V_ACT])