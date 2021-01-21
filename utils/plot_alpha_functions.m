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
    c1 = @(x) min(max(f1(x), 0), 1); 
    c2 = @(x) min(max(f2(x), 0), 1); 
    c3 = @(x) min(max(f3(x), 0), 1); 
    c4 = @(x) min(max(f4(x), 0), 1); 
    c5 = @(x) min(max(f5(x) .* (x >= 0), 0), 1); 
    c6 = @(x) min(max(f6(x), 0), 1); 
    
    plot(x, c1(x), x, c2(x), x, c3(x), x, c4(x), x, c5(x), x, c6(x)); 
    ylim([-0.5, 1.5]); 
    xlim([-1, 2]); 
    xlabel('V(x) safety score'); 
    ylabel('alpha for original plan');
    title('Alpha functions'); 
    repo = what("arbitration"); 
    legend('x', 'e^{2x}-1', 'e^x-1', 'e^{0.5x}-1', 'x^2', 'x^3'); 
    savefig(strcat(repo.path, '/data/alpha_functions.fig')); 
end 