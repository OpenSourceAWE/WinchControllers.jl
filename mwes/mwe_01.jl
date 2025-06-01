# Determine the steady-state speed of a winch
# input: set_speed
# output: speed

using WinchControllers, KiteUtils

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
    V_SET = zeros(Float64, n)
    # slow start
    for speed in range(0.0, set_speed, n)
        set_force(winch, force)
        set_v_set(winch, speed)
        v_act = get_speed(winch)
        push!(V_SET, v_set)        
        on_timer(winch)
    end
    set_v_set(winch, set_speed)
    on_timer(winch)
    return V_SET
end