using WinchControllers, ModelPredictiveControl, WinchModels, KiteUtils, Plots

set = load_settings("system.yaml")
wcs = WCSettings(dt=0.02)
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
N = length(lg)

# 1. Include Wind Speed in the Model
function f!(dx, x, u, d, _)
    v = x[1]
    τ = u[1]
    v_wind = d[1] # disturbance
    F = calc_force(v_wind, v)
    a = calc_acceleration(winch, v, F; set_torque=τ)
    dx[1] = a
    nothing
end
function h!(y, x, d, _)
    v = x[1]
    v_wind = d[1] # disturbance
    y[1] = v
    y[2] = calc_force(v_wind, v)
    nothing
end

# nu, nx, ny, Ts = 1, 1, 2, wcs.dt # original
nu, nx, ny, nd, Ts = 1, 1, 2, 1, wcs.dt # disturbance
vu, vx, vy = ["\$τ\$ (Nm)"], ["v (m/s)"], ["v (m/s)", "F (N)"]
vd = ["v_wind (m/s)"]
model = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd); u=vu, x=vx, y=vy, d=vd)
plant = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd); u=vu, x=vx, y=vy, d=vd)

linmodel = linearize(model, x=[0], u=[0], d=[0])
α=0.01; σQ=[0.05]; σR=[0.5, 0.5]; nint_u=[0]; σQint_u=[0.1]; nint_ym=[0,0]
kf = KalmanFilter(linmodel; σQ, σR, nint_u, nint_ym)

Hp, Hc, Mwt, Nwt = 20, 20, [0.5, 0.0], [0.1]
umin, umax = [-100.0], [+100.0]
ymin, ymax = [-Inf, wcs.f_low], [Inf, wcs.f_high]
mpc = LinMPC(kf; Hp, Hc, Mwt, Nwt, Cwt=1e3)
mpc = setconstraint!(mpc; umin, umax, ymin, ymax)

function sim_adapt!(mpc, nonlinmodel, N, ry, plant, x_0, x̂_0, y_step=[0, 0])
    U_data, Y_data, D_data, Ry_data, X̂_data, X_data = 
        zeros(plant.nu, N), zeros(plant.ny, N), zeros(plant.nd, N), zeros(plant.ny, N), zeros(mpc.estim.nx̂, N), zeros(plant.nx, N)
    setstate!(plant, x_0)
    initstate!(mpc, [0], plant([0]), [0])
    setstate!(mpc, x̂_0)
    for i = 1:N
        d = [V_WIND[i]]
        y = plant(d) + y_step # disturbance
        ry[1] = sqrt(y[2]) * wcs.kv
        # model.p[1] = V_WIND[i] # remove
        x̂ = preparestate!(mpc, y, d)
        u = moveinput!(mpc, ry, d)
        linmodel = linearize(nonlinmodel; u, x=x̂[1], d) # disturbance
        setmodel!(mpc, linmodel)
        U_data[:,i], Y_data[:,i], D_data[:,i], Ry_data[:,i], X̂_data[:,i], X_data[:,i] = u, y, d, ry, x̂, plant.x0
        updatestate!(mpc, u, y, d) # update mpc state estimate
        updatestate!(plant, u, d)  # update plant simulator
    end
    res = SimResult(mpc, U_data, Y_data, D_data; Ry_data, X̂_data, X_data)
    return res
end

ry = [1.0, 0.0]
x_0 = [0.0]
x̂_0 = zeros(1)
@time res_slin = sim_adapt!(mpc, model, N, ry, plant, x_0, x̂_0)
plot(res_slin; plotd=false, plotxwithx̂=true)
