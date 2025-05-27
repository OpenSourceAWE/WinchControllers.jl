# Calculate the pulling force of the kite as function of the reel-out speed and the wind speed in the
# direction of the tether at the height of the kite. Most simplified model, massless, constant L/D,
# constant elevation angle.
function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

function speed_controller_step!(pid, winch, i, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    set_v_act(pid, v_ro)
    v_set_out = get_v_set_out(pid)
    ACC_SET[i] = (v_set_out - last_v_set_out) / pid.wcs.dt
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(pid)
    last_v_set_out = v_set_out
end

function speed_controller_step2!(pid, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    # calc v_set_in
    set_vset_pc(calc, nothing, last_force[])
    v_set_in = calc_output(calc)
    set_v_set_in(pid, v_set_in)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, 0.5 * force + 0.5 * last_force[])
    set_v_act(pid, v_ro)
    v_set_out = get_v_set_out(pid)
    ACC_SET[i] = (v_set_out - last_v_set_out) / pid.wcs.dt
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid)
    last_force[] = force
    last_v_set_out = v_set_out
end

function speed_controller_step3!(pid1, sc, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, 
                                 FORCE, V_SET_OUT, V_SET_OUT_B, STATE, V_ERR, F_ERR)
    # calc v_set_in of the speed controller
    set_vset_pc(calc, nothing, last_force[])
    v_set_in = calc_output(calc)
    set_v_set_in(pid1, v_set_in)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    # calculate v_set_out_A from the speed controller
    set_v_act(pid1, v_ro)
    v_set_out_A = get_v_set_out(pid1)
    V_ERR[i] = get_v_error(pid1)
    # calculate v_set_out_B from the force controller
    set_force(sc, last_force[])
    if i * wcs.dt <= wcs.t_startup
        reset = true
    else
        reset = false
    end
    set_reset(sc, reset)
    v_sw = calc_vro(wcs, sc.f_set) #* 1.05
    set_v_sw(sc, v_sw)
    set_v_act(sc, v_ro)
    set_tracking(sc, v_set_out_A)
    set_force(sc, force)
    v_set_out_B = get_v_set_out(sc)
    F_ERR[i] = get_f_err(sc)
    if ! sc.active
        F_ERR[i] = NaN
    end
    set_tracking(pid1, v_set_out_B)
    select_b(mix2, sc.active)
    set_inactive(pid1, sc.active)
    STATE[i] = sc.active
    V_SET_OUT_B[i] = v_set_out_B
    v_set_out = calc_output(mix2, v_set_out_A, v_set_out_B)
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid1)
    on_timer(sc)
    on_timer(mix2)
    last_force[] = force
    last_v_set_out[] = v_set_out            
end

function speed_controller_step4!(pid1, sc, ufc, mix3, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, 
         V_RO, ACC, FORCE, V_SET_OUT, STATE, V_ERR, F_ERR, V_SET)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    # calc v_set_in of the speed controller
    set_vset_pc(calc, nothing, force)
    v_set_in = calc_output(calc)
    set_v_set_in(pid1, v_set_in)

    set_force(winch, force)
    # calculate v_set_out_A from the speed controller
    set_v_act(pid1, v_ro)
    v_set_out_A = get_v_set_out(pid1)
    V_ERR[i] = get_v_error(pid1)
    # calculate v_set_out_B from the force controller
    set_force(sc, last_force[])
    if i * wcs.dt <= wcs.t_startup
        reset = true
    else
        reset = false
    end
    set_reset(sc, reset)
    # set v_switch (the speed, when the force controllers shall be turned off)
    v_sw = calc_vro(wcs, sc.f_set) * 1.05
    set_v_sw(sc, v_sw)
    v_sw = calc_vro(wcs, ufc.f_set) * 0.95
    set_v_sw(ufc, v_sw)

    set_v_act(sc, v_ro)
    set_v_act(ufc, v_ro)
    
    set_tracking(pid1, last_v_set_out[])
    set_tracking(sc, last_v_set_out[])
    set_tracking(ufc, last_v_set_out[])

    set_force(sc, force)
    set_force(ufc, force)

    v_set_out_B = get_v_set_out(sc)
    v_set_out_C = get_v_set_out(ufc)
    F_ERR[i] = get_f_err(sc) + get_f_err(ufc)

    select_b(mix3, sc.active)
    select_c(mix3, ufc.active)
    set_inactive(pid1, sc.active || ufc.active)
    STATE[i] = get_state(mix3)
    if STATE[i] == 1
        F_ERR[i] = NaN
        V_SET[i] = v_set_in
    else
        V_SET[i] = NaN
    end

    v_set_out = calc_output(mix3, v_set_out_A, v_set_out_B, v_set_out_C)
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid1)
    on_timer(sc)
    on_timer(ufc)
    on_timer(mix3)
    last_force[] = force
    last_v_set_out[] = v_set_out            
end
