```@meta
CurrentModule = WinchControllers
```

# Performance Indicators
The force error, the speed error and the control effort should be taken into account.

## Relative force error
The relative force error is the maximum of the norm of the force errors, based on all samples where one of the force controllers was active, divided by the nominal winch force $F_\mathrm{max}$, as formula:

$F_\mathrm{err} = \frac{1}{F_\mathrm{max}}~{ \max |F-F_\mathrm{set}|}$

The reason to use the maximum here is that it must be avoided that the force goes below zero and also that it exceeds the maximum tether force.

## Relative velocity error
The relative velocity error is the root mean square error (RMSE) of the velocity, based on all samples where one of the force controllers was active, divided by the average winch speed $\bar{v}$, as formula:

$v_\mathrm{err} = \frac{1}{\bar{v}}~\sqrt{\frac{1}{n} \sum_{i=1}^{n} (v-v_\mathrm{set})^2}$
with
$\bar{v} = \frac{1}{n} \sum_{i=1}^{n} \lvert v_\mathrm{set} \rvert$

For now, it is assumed that the maximal reel-in and reel-out speed of the winch are identical:

$v_\mathrm{max} = v_\mathrm{ro,max} = v_\mathrm{ri,max}$

The reason to use the mean square error for the velocity is that in this mode of operation the error only reduces the power output. Short peak errors are usually not a problem.

## Combined performance
The combined performance indicator $\gamma$ in the range of 0..1 is defined as

$\gamma = 1 - \frac{1}{2}(F_\mathrm{err} + v_\mathrm{err})~$.

## TODO
- Take the control effort into account. This could be the average power needed to accelerate and to slow down the winch with respect to the average mechanical power at lower end of the tether.
- Quantify the robustness. This could be done by linearizing the system and checking gain and phase margin (for example), or by varying the model parameters (e.g. inertia of the drum) and checking if the combined performance stays above a required minimum. Possible robustness requirements:  
    - the controller must be robust for a model uncertainty of $\pm 15\%$ inertia
    - it must be robust against $\pm 5\%$ drum radius
    - it must be robust against $20ms$ delay in the force control loop
