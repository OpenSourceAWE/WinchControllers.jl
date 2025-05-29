# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using Timers, Statistics; tic()

# Test the speed controller in combination with the controller for the lower and upper force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test2.png
using WinchControllers, ControlPlots, KiteUtils

set = deepcopy(load_settings("system.yaml"))
wcs = WCSettings(dt=0.02)
update(wcs)
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
wcs.pf_high = 1.44e-4*1.6*0.5
wcs.i_speed = 4.0

DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

include("test_utilities.jl")

STARTUP = get_startup(wcs, SAMPLES)    
V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, SAMPLES)
V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
RESET, ACTIVE, F_SET = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
STATE = zeros(Int64, SAMPLES)
# create and initialize winch controller 
wc = WinchController(wcs)
winch = WinchControllers.Winch(wcs, set)
f_low = wcs.f_low

for i in 1:SAMPLES
    local force
    # model
    v_wind = V_WIND[i]

    v_act = get_speed(winch)
    force = calc_force(v_wind, v_act)
    set_force(winch, force)

    # controller
    v_set = calc_v_set(wc, v_act, force, f_low)
    
    # update model
    set_v_set(winch, v_set)
    
    on_timer(winch)
    on_timer(wc)

    # logging
    acc   = get_acc(winch)
    state = get_state(wc)
    status = get_status(wc)
    ACC[i] = acc 
    STATE[i] = state
    V_RO[i] = v_act
    RESET[i] = status[1]
    ACTIVE[i] = status[2]
    FORCE[i] = status[3]
    F_SET[i] = status[4]
    V_SET_OUT[i] = v_set
    if state in [0,2]
        F_ERR[i] = FORCE[i] - F_SET[i]
        V_ERR[i] = 0.0
    else
        V_ERR[i] = V_RO[i] - v_set
        F_ERR[i] = 0.0
    end
end


if @isdefined __TEST__
    dt = wcs.dt
    @test V_WIND[Int64(2/0.02)+1] ≈ 9.0 rtol=1e-4
    @test V_WIND[Int64(4/0.02)+1] ≈ 0.0 atol=1e-4
    @test V_WIND[Int64(6/0.02)+1] ≈ 9.0 rtol=1e-4
    @test mean(FORCE) ≈ 1783.2979123136008 rtol=1e-4
    @test maximum(FORCE) < 4295
    @test minimum(FORCE[10:end]) > 10.0
    @test STATE[Int64(0.5/dt)] == 0
    @test STATE[Int64(1.0/dt)] == 1
    @test STATE[Int64(2.0/dt)] == 2
    @test STATE[Int64(3.0/dt)] == 1
    @test STATE[Int64(4.0/dt)] == 0
    @test mean(F_ERR) ≈ 27.65 rtol=0.1
    @test maximum(F_ERR) < 495
    @test minimum(F_ERR) > -400
    @test mean(V_ERR) ≈ 0.0125 rtol=0.1
    @test maximum(V_ERR) < 0.12
    @test minimum(V_ERR) > -0.13


else
    # plot the results  
    p1=plotx(TIME, V_WIND, [V_RO, V_SET_OUT], F_ERR*0.001, V_ERR, ACC, FORCE*0.001, STATE,
      title="Winch controller test, all controllers active",
      ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
      labels=["v_wind", ["v_reel_out", "v_set_out"]],
      fig="test_winchcontroller",)

    display(p1)
    toc()
    println("Max iterations needed: $(wcs.iter)")
end

