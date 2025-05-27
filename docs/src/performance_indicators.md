```@meta
CurrentModule = WinchControllers
```

# Performance Indicators

The force error, the speed error and the control effort should be taken into account.

## Relative force error
The relative force error is the root mean square error (RMSE) of the force, based on all samples where one of the force controllers was active, divided by the nominal winch force $F_{max}$, as formula:

$F_{err} = \frac{1}{F_{max}}~\sqrt{\frac{1}{n} \sum_{i=1}^{n} (F-F_{set})^2}$

## Relative velocity error
The relative velocity error is the root mean square error (RMSE) of the velocity, based on all samples where one of the force controllers was active, divided by the nominal winch speed $v_{max}$, as formula:

$v_{err} = \frac{1}{v_{max}}~\sqrt{\frac{1}{n} \sum_{i=1}^{n} (v-v_{set})^2}$

For now, it is assumed that the maximal reel-in and reel-out speed of the winch are identical:

$v_{max} = v_{ro\_max} = v_{ri\_max}$

## Combined performance
The combined performance indicator $\gamma$ in the range of 0..1 is defined as

$\gamma = 1 - \frac{1}{2}(F_{err} + v_{err})~$.

## TODO
Take the control effort into account. This could be the average power needed to accelerate and to slow down the winch with respect to either the nominal winch power, or with respect to the average mechanical power at lower end of the tether.

