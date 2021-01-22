function f = get_alpha_blend_function(name)
    f1 = @(x) x;
    f2 = @(x) (exp(2*x) - 1); 
    f3 = @(x) exp(x) - 1; 
    f4 = @(x) (exp(0.5*x) - 1); 
    f5 = @(x) x .^ 2;
    f6 = @(x) x .^ 3; 
    f7 = @(x) x .^ (3/5); 
    f8 = @(x) x .^ (1/2); 
    f9 = @(x) x .^ (2/5); 
    f10 = @(x) x .^ (1/3); 
    f11 = @(x) x .^ (1/4); 
    c1 = @(x) min(max(f1(x), 0), 1); 
    c2 = @(x) min(max(f2(x), 0), 1); 
    c3 = @(x) min(max(f3(x), 0), 1); 
    c4 = @(x) min(max(f4(x), 0), 1); 
    c5 = @(x) min(max(f5(x) .* (x >= 0), 0), 1); 
    c6 = @(x) min(max(f6(x), 0), 1); 
    c7 = @(x) f7(min(max(x, 0), 1)); 
    c8 = @(x) f8(min(max(x, 0), 1));     
    c9 = @(x) f9(min(max(x, 0), 1)); 
    c10 = @(x) f10(min(max(x, 0), 1));    
    c11 = @(x) f11(min(max(x, 0), 1)); 

    if strcmp(name, "x")
        f = c1; 
    elseif strcmp(name, "exp_2x")
        f = c2; 
    elseif strcmp(name, "exp_x")
        f = c3; 
    elseif strcmp(name, "exp_0.5x")
        f = c4; 
    elseif strcmp(name, "x2") 
        f = c5; 
    elseif strcmp(name, "x3")
        f = c6; 
    elseif strcmp(name, "x_3-5")
        f = c7; 
    elseif strcmp(name, "x_1-2")
        f = c8; 
    elseif strcmp(name, "x_2-5")
        f = c9;     
    elseif strcmp(name, "x_1-3")
        f = c10;     
    elseif strcmp(name, "x_1-4")
        f = c11;     
    else
        warning("No proper blending function"); 
    end
end 