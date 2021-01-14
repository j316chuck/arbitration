%% Defines a custom colormapping for values in the range [min_val, max_val]
%   Returns a RGB color vector for the specified value val under this
%   colormap.
function [val_color, cmap] = custom_colormap(val, min_val, max_val)
    % TODO: right now we have fixed colors.
    low_color = [0.706, 0.016, 0.150]; 
    up_color = [0.230, 0.299, 0.754];

    % Blending between lower and upper value (normalized to [0,1])
    alpha = (val - min_val) / (max_val - min_val);

    val_color = alpha * up_color + (1-alpha) * low_color;
    
    % Return optionally the custom colomap object.
    cmap = [linspace(low_color(1), up_color(1), 100)', ...
            linspace(low_color(2), up_color(2), 100)', ...
            linspace(low_color(3), up_color(3), 100)'];
end