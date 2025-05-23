# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# Test the speed controller in combination with the controller for the lower and upper force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test2.png
using WinchControllers, KiteUtils, ControlPlots

set = deepcopy(load_settings("system.yaml"))

wcs = WCSettings(dt=0.02)
update(wcs)
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
#wcs.pf_low = 1.44e-4*0.5
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

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
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
    speed_controller_step4!(pid1, sc, ufc, mix3, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, FORCE, V_SET_OUT, STATE, V_ERR, F_ERR)
end

p1=plotx(TIME, V_WIND, [V_RO, V_SET_OUT], F_ERR*0.001, V_ERR, ACC, FORCE*0.001, STATE,
      title="Winch controller test, only lower-force and speed controller active",
      ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
      labels=["v_wind", ["v_reel_out", "v_set_out"]],
      fig="test_forcespeed_1a")

display(p1)

toc()

println("Max iterations needed: $(wcs.iter)")
    