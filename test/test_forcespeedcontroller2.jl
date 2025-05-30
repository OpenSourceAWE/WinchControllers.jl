# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# Test the speed controller in combination with the controller for the lower and upper force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test2.png
using WinchControllers, KiteUtils, ControlPlots, LinearAlgebra, Statistics

set = deepcopy(load_settings("system.yaml"))

wcs = WCSettings(dt=0.02)
update(wcs)
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
# wcs.t_blend = 0.25
# wcs.pf_low = 1.44e-4*0.5
wcs.pf_high = 1.44e-4*1.6*0.5
# wcs.kt_speed = 10


DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 
BENCHMARK = false

include("test_utilities.jl")

function f_err1(set, f_err_)
    f_max = set.max_force
    1/f_max * maximum(norm.(filter(!isnan, f_err_)))
end

rms(x) = norm(x) / sqrt(length(x))

function v_err1(v_err_, v_set)
    v_mean =  mean(norm.(filter(!isnan, v_set)))
    1/v_mean * rms(filter(!isnan, v_err_))
end

function gamma1(set, f_err_, v_err_, v_set)
    1 - 0.5(f_err1(set, f_err_) + v_err1(v_err_, v_set))
end

STARTUP = get_startup(wcs, SAMPLES)    
V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, SAMPLES)
V_RO, V_SET, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
STATE = zeros(Int64, SAMPLES)
# create and initialize speed controller 
pid1 = SpeedController(wcs)
set_tracking(pid1, 0.0)
set_inactive(pid1, false)
set_v_set_in(pid1, 4.0)
# set_v_set(pid1, -0.5)
# create and initialize lower force controller
sc = LowerForceController(wcs)
set_f_set(sc, wcs.f_low)
set_tracking(sc, 0.0)
set_reset(sc, true)
set_v_sw(sc, -1.0)
# create the mixer for the output of the two controllers
mix2 = Mixer_2CH(wcs.dt, wcs.t_blend)
# create winch model and unit delay and the v_set_in calculator and mixer
winch = Winch(wcs, set)
delay = UnitDelay()
calc = CalcVSetIn(wcs)
# create and initialize upper force controller
ufc = UpperForceController(wcs) 
set_v_sw(ufc, calc_vro(wcs, ufc.f_set))
set_reset(ufc, true)
set_reset(ufc, false)
# create the mixer for the output of the two controllers
mix3 = Mixer_3CH(wcs.dt, wcs.t_blend)
last_force = Ref(0.0)
last_v_set_out = Ref(0.0)

for i in 1:SAMPLES
    speed_controller_step4!(pid1, sc, ufc, mix3, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, 
                            ACC, FORCE, V_SET_OUT, STATE, V_ERR, F_ERR, V_SET)
end

p1=plotx(TIME, V_WIND, [V_RO, V_SET_OUT], F_ERR*0.001, V_ERR, ACC, FORCE*0.001, STATE,
      title="Winch controller test, all controllers active",
      ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
      labels=["v_wind", ["v_reel_out", "v_set_out"]],
      ysize=10,
      fig="test_forcespeed_2")

display(p1)

toc()

println("Max iterations needed: $(wcs.iter)")
println("Performance of force controllers: $(round(100*(1-f_err1(set, F_ERR)), digits=2)) %")
println("Performance of speed controller: $(round(100*(1-v_err1(V_ERR, V_SET)), digits=2)) %")
println("Combined performance γ: $(round(100*gamma1(set, F_ERR, V_ERR, V_SET), digits=2)) %")    