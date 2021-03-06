function plot_alpha_functions()
    figure(1); 
    clf; 
    hold on;
    x = -1:0.01:2;
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
    
    plot(x, c1(x), x, c2(x), x, c3(x), x, c4(x), x, c5(x), x, c6(x), ...
        x, c7(x), x, c8(x), x, c9(x), x, c10(x), x, c11(x)); 
    ylim([-0.5, 1.5]); 
    xlim([-1, 2]); 
    xlabel('V(x) safety score'); 
    ylabel('alpha for original plan');
    title('Alpha functions'); 
    repo = what("arbitration"); 
    legend('x', 'e^{2x}-1', 'e^x-1', 'e^{0.5x}-1', 'x^2', 'x^3', ...
        'x^{3/5}', 'x^{1/2}', 'x^{2/5}', 'x^{1/3}', 'x^{1/4}'); 
    savefig(strcat(repo.path, '/data/alpha_functions.fig')); 
end 