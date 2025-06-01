# Linearize the winch
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

v_set = 8.0
force = 1000.0
v_act = find_equilibrium_speed(winch, v_set, force)