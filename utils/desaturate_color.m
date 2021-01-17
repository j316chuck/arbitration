%% Desaturates color by f percent 
function curr_color = desaturate_color(color, f)
    L = 0.3*color(1) + 0.6*color(2) + 0.1*color(3);
    curr_color(1) = color(1) + f * (L - color(1));
    curr_color(2) = color(2) + f * (L - color(2));
    curr_color(3) = color(3) + f * (L - color(3));
end