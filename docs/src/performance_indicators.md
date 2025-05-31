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

The reason to use the mean square error for the velocity is that in this mode of operation the error only reduces the power output. Short peak errors are usually not a problem.

## Damage
While the performance shall be maximized, the damage shall be minimized. Here, a simple
damage model is presented,

$\epsilon = \Delta~\left((1-\zeta)\left(\frac{\max |a|}{a_\mathrm{max}}\right)^2~+~\zeta \left(\frac{\mathrm{rms}(j)}{a_\mathrm{max}^2}\right)^4\right)$,

where $a$ is the actual acceleration, $a_{max}$ the specified, maximal acceleration, $\mathrm{rms}(j)$ the root-mean-square of the jerk, the derivative of the acceleration, $\Delta$ the damage that occurs at $a=a_\mathrm{max}$ and $\zeta$ the jerk factor. The default values used are $\Delta=0.05$ and $\zeta = 0.9$, but in the end this value needs to be determined based on the specification of the Winch. Control-codesign can be used to determine this parameter.

Using the forth potence of the jerk has proven to be effective in suppressing any oscillations and reducing overshoot of the controller while keeping it fast and stable.

## Combined performance
The combined performance indicator $\gamma$ in the range of 0..1 is defined as

$\gamma = 1 - \frac{1}{2.5}(1.5F_\mathrm{err} + v_\mathrm{err})~-~\epsilon~$,

the average of the error of the speed and the force controllers minus the damage caused by the acceleration of the winch. The force error has a slightly higher weight, because it is more critical for the controller performance.

## TODO
- Quantify the robustness. This could be done by linearizing the system and checking gain and phase margin (for example), or by varying the model parameters (e.g. inertia of the drum) and checking if the combined performance stays above a required minimum. Possible robustness requirements:  
    - the controller must be robust for a model uncertainty of $\pm 15\%$ inertia
    - it must be robust against $\pm 5\%$ drum radius
    - it must be robust against $20ms$ delay in the force control loop
