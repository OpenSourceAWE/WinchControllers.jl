using WinchControllers, ModelPredictiveControl, WinchModels, KiteUtils, Plots

set = load_settings("system.yaml")
wcs = WCSettings(dt=0.1)
winch = TorqueControlledMachine(set)

# Calculate the pulling force of the kite as function of the reel-out speed and the wind speed in the
# direction of the tether at the height of the kite. Most simplified model, massless, constant L/D,
# constant elevation angle.
function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

# define the simulation parameters
DURATION   = 10.0
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

# create the logger
lg = WCLogger(DURATION, wcs.dt, set.max_force, wcs.max_acc, wcs.damage_factor, wcs.jerk_factor)

STARTUP = get_startup(wcs, length(lg))    
V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, length(lg))

function f(x, u, _ , p)
    τ = u[1]
    v = x[1]
    v_wind = p[1]
    F = calc_force(v_wind, v)
    α = calc_acceleration(winch, v, F; set_torque=τ)
    return [α]
end
function h(x, _ , p) 
    v_wind = p[1]
    v = x[1]
    return [v, calc_force(v_wind, v)] # [°]
end

p_model = [3.0]
nu, nx, ny, Ts = 1, 1, 2, wcs.dt
vu, vx, vy = ["\$τ\$ (Nm)"], ["v (m/s)"], ["v (m/s)", "F (N)"]
model = setname!(NonLinModel(f, h, Ts, nu, nx, ny; p=p_model); u=vu, x=vx, y=vy)

α=0.01; σQ=[0.05]; σR=[0.5, 0.5]; nint_u=[1]; σQint_u=[0.1]
estim = UnscentedKalmanFilter(model; α, σQ, σR, nint_u, nint_ym=[1,1], σQint_u)

N = 100
p_plant = copy(p_model)
p_plant[1] = p_model[1]
plant = setname!(NonLinModel(f, h, Ts, nu, nx, ny; p=p_plant); u=vu, x=vx, y=vy)
# res = sim!(estim, N, [-10], plant=plant, y_noise=[0.1])
# plot(res, plotu=false, plotxwithx̂=true)

Hp, Hc, Mwt, Nwt = 20, 2, [0.5, 0.0], [0.1]
nmpc = NonLinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt=Inf)
umin, umax = [-100.0], [+100.0]
ymin, ymax = [-Inf, wcs.f_low], [Inf, wcs.f_high]
nmpc = setconstraint!(nmpc; umin, umax, ymin, ymax)

@time res_ry = sim!(nmpc, N, [1.0, 0.0], plant=plant, x_0=[0], x̂_0=[0, 0, 0, 0])
plot(res_ry)
