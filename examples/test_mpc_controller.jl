using ModelPredictiveControl, WinchModels

set = load_settings("system.yaml")
winch = TorqueControlledMachine(set)

function f(x, u, _ , p)
    τ = u[1]
    ω = x[1]
    α = calc_acceleration(winch, )
 
    return [α]
end
h(x, _ , _ ) = x # [°]
p_model = [9.8, 0.4, 1.2, 0.3]
nu, nx, ny, Ts = 1, 2, 1, 0.1
vu, vx, vy = ["\$τ\$ (Nm)"], ["\$θ\$ (rad)", "\$ω\$ (rad/s)"], ["\$θ\$ (°)"]
model = setname!(NonLinModel(f, h, Ts, nu, nx, ny; p=p_model); u=vu, x=vx, y=vy)