"""
A collection of control functions for discrete control

Functions:

- saturate
- limit 
- merge_angles
- moving_average

Implemented as described in the PhD thesis of Uwe Fechner.
"""

""" 
    saturate(value, min_, max_)

Calculate a saturated value, that stays within the given limits. 

## Parameters:
- value: the input value
- min_: the lower value to which it shall be clamped
- max_: the upper value to which it shall be clamped

## Returns:
- the clamped value

## Remark:
- consider using the function `clamp` or the macro [`@limit`](@ref) instead
"""
function saturate(value, min_, max_)
    result = value
    if result > max_
        result = max_
    elseif result < min_
        result = min_
    end
    result
end

"""
    macro limit(name, minmax, max=nothing)

Limit the value of a variable. 

## Parameters:
- name: the name of the scalar variable that shall be limited
- minmax: if max is provided, this is the lower value to which the variable is clamped, otherwise it is the upper value
- max: the upper value to which to limit the provided variable or nothing

## Usage:
- @limit x 1 4 # limits the value to the range    1 .. 4,  modifies x
- @limit x 10  # limits the value to the range -inf .. 10, modifies x

"""
macro limit(name, minmax, max=nothing)
    if isnothing(max)
        max = minmax
        min = :(typemin($minmax))
    else
        min = minmax
    end

    return esc( :($name = clamp($name, $min, $max)) )
end

"""
    merge_angles(alpha, beta, factor_beta)

Calculate the weighted average of two angles. The weight of beta,
factor_beta must be between 0 and 1.
"""
function merge_angles(alpha, beta, factor_beta)
    x1 = sin(alpha)
    y1 = cos(alpha)
    x2 = sin(beta)
    y2 = cos(beta)
    x = x1 * (1.0 - factor_beta) + x2 * factor_beta
    y = y1 * (1.0 - factor_beta) + y2 * factor_beta
    atan(x, y)
end

# calculate the moving average of a vector over a window
function moving_average(data, window)
    local result
    if length(data) <= window
        result = mean(data)
    else
        result = mean(data[length(data)-window:end])
    end
    result
end