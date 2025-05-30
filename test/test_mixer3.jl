# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using WinchControllers, ControlPlots
"""
Test the three channel mixer, using two sinus signals of different frequencies
and a random noise signal.
"""

DURATION = 10.0
PERIOD_TIME = 0.05
F1   = 0.5 # test frequency one
F2   = 0.25 # test frequency one
SAMPLES = Int(DURATION / PERIOD_TIME + 1)
TIME = range(0.0, DURATION, SAMPLES)
T_BLEND = 0.25 # blending time of the mixers in seconds
T1   = 3.0 # time one
T2   = 6.0
T3   = 9.0

SIN1 = sin.(TIME * F1 * (2*pi)) * 2
SIN2 = sin.(TIME * F2 * (2*pi))
NOISE = (rand(SAMPLES).-0.5) * 2

out = zeros(SAMPLES)
factor_b = zeros(SAMPLES)
factor_c = zeros(SAMPLES)
mix3 = Mixer_3CH(PERIOD_TIME, T_BLEND)

function time1(sample)
    return sample * PERIOD_TIME
end

for i in 1:SAMPLES
    if time1(i) ≈ T1
        select_b(mix3, true)
    elseif time1(i) ≈ T2
        select_c(mix3, true)
    elseif time1(i) ≈ T3
        select_c(mix3, false)        
    end
    factor_b[i] = mix3.factor_b
    factor_c[i] = mix3.factor_c
    out[i] = calc_output(mix3, SIN1[i], SIN2[i], NOISE[i])
    on_timer(mix3)
end
p=plot(TIME, [out, factor_b, factor_c]; labels=["output", "factor_b", "factor_c"])
display(p)

