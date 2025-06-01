# Linearize the winch
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
function find_equilibrium_speed(winch, force)
    set_speed = 0.0
    for _ in 1:1000
        v_act = get_speed(winch)
        set_force(winch, force)
        set_v_set(winch, set_speed)
        
        on_timer(winch)
        
        # check if the speed is close enough to the desired equilibrium
        if abs(get_speed(winch) - set_speed) < 0.01
            break
        end
    end
    set_v_set(winch, set_speed)
    on_timer(winch)
    return get_speed(winch)
end