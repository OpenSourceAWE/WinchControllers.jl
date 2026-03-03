# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using WinchControllers, ControlPlots

PERIOD_TIME = 0.05

m2 = Mixer_2CH(PERIOD_TIME, 1.0)
x = ones(10)
y = 2*x
out = zeros(10)
for i in eachindex(x)
    out[i] = calc_output(m2, x[i], y[i])
    select_b(m2, i > 2)
    on_timer(m2)
end
p = plot(1:10, [x, y, out]; labels=["input_a", "input_b", "output"])
display(p)

