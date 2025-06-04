using WinchControllers, ModelPredictiveControl, WinchModels, KiteUtils, Plots

set = load_settings("system.yaml")
wcs = WCSettings(dt=0.01; f_low=10.0)
winch = TorqueControlledMachine(set)
model_set = deepcopy(set)
model_set.drum_radius *= 1.1
model_winch = TorqueControlledMachine(model_set)

# Calculate the pulling force of the kite as function of the reel-out speed and the wind speed in the
# direction of the tether at the height of the kite. Most simplified model, massless, constant L/D,
# constant elevation angle.
function calc_force(v_wind, v_ro)
    if v_wind > v_ro
        (v_wind - v_ro)^2 * 4000.0 / 16.0
    else
        -(v_wind - v_ro)^2 * 4000.0 / 16.0
    end
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

function f!(dx, x, u, d, p)
    winch = p[1]
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
    y[3] = v / sqrt(abs(y[2]) + 1e-8)
    nothing
end

# nu, nx, ny, Ts = 1, 1, 2, wcs.dt # original
nu, nx, ny, nd, Ts = 1, 1, 3, 1, wcs.dt # disturbance
vu, vx, vy = ["\$τ\$ (Nm)"], ["v (m/s)"], ["v (m/s)", "F (N)", "kv (-)"]
vd = ["v_wind (m/s)"]
model = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd, p=[model_winch]); u=vu, x=vx, y=vy, d=vd)
plant = setname!(NonLinModel(f!, h!, Ts, nu, nx, ny, nd, p=[winch]); u=vu, x=vx, y=vy, d=vd)

linmodel = linearize(model, x=[0], u=[0], d=[0])
α=0.01; σQ=[0.05]; σR=[0.5, 0.5]; nint_u=[1]; σQint_u=[0.1]; nint_ym=[1, 1]
i_ym = [1,2]
estim = KalmanFilter(linmodel; i_ym, σQ, σR, nint_u, nint_ym)

Hp, Hc, Mwt, Nwt = 20, 2, [0.0, 0.0, 1e5], [0.01]
umin, umax = [-wcs.f_high*set.drum_radius/set.gear_ratio], [+wcs.f_high*set.drum_radius/set.gear_ratio]
ymin, ymax = [-Inf, wcs.f_low, -Inf], [Inf, wcs.f_high, Inf]
mpc = LinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt=1e7)
mpc = setconstraint!(mpc; umin, umax, ymin, ymax)

function sim_adapt!(mpc, nonlinmodel, N, ry, plant, x_0, x̂_0, y_step=[0,0,0])
    U_data, Y_data, Ŷ_data, D_data, Ry_data, X̂_data, X_data = 
        zeros(plant.nu, N), zeros(plant.ny, N), zeros(plant.ny, N), zeros(plant.nd, N), zeros(plant.ny, N), zeros(mpc.estim.nx̂, N), zeros(plant.nx, N)
    setstate!(plant, x_0)
    initstate!(mpc, [0], plant([0])[i_ym], [0])
    setstate!(mpc, x̂_0)
    for i = 1:N
        d = [V_WIND[i]]
        y = plant(d) + y_step # disturbance
        ym = copy(y)[i_ym]
        # ym[2] = 0.0
        ry[1] = sqrt(abs(y[2])) * wcs.kv
        x̂ = preparestate!(mpc, ym, d)
        u = moveinput!(mpc, ry, d)
        linmodel = linearize(nonlinmodel; u, x=x̂[1], d) # disturbance
        setmodel!(mpc, linmodel)
        U_data[:,i], Y_data[:,i], Ŷ_data[:,i], D_data[:,i], Ry_data[:,i], X̂_data[:,i], X_data[:,i] = u, y, mpc.ŷ, d, ry, x̂, plant.x0
        updatestate!(mpc, u, ym, d) # update mpc state estimate
        updatestate!(plant, u, d)  # update plant simulator
    end
    res = SimResult(mpc, U_data, Y_data, D_data; Ry_data, Ŷ_data, X̂_data, X_data)
    return res
end

ry = [0.0, 0.0, wcs.kv]
x_0 = [0.0]
x̂_0 = zeros(estim.nx̂)
@time res = sim_adapt!(mpc, model, N, ry, plant, x_0, x̂_0)
@show maximum(res.Y_data[2,:]) - wcs.f_high
@show minimum(res.Y_data[2,:])
mpc.weights.M_Hp[1,1] = 1.0
plot(res; plotd=false, plotx=false, plotxwithx̂=false, ploty=true, plotŷ=true, plotry=true)