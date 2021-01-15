repo = what('arbitration');
% params = default_hyperparams(); 
% exp = load_exp(params); 
% exp.brs_planner.solve_brs_avoid(exp.obstacle)
% brs_planner = exp.brs_planner; 
% filename = strcat(repo.path, '/data/brs_planner.mat'); 
% save(filename, 'brs_planner');

load(filename, 'brs_planner'); 
thetas = brs_planner.grid.vs{3};

n = length(thetas);
m = ceil(n / 4); 
figure(3);
clf;
set(gcf, 'Position',  [100, 100, 3000, 3000])
for t=1:n
    theta = thetas(t); 
    [g2d, data2d] = proj(brs_planner.grid, brs_planner.valueFun, [0 0 1], [theta]); 
    x = g2d.xs{1};
    y = g2d.xs{2};
    x1 = []; x2 = []; y1 = []; y2 = []; u1 = []; v1 = [];
    dt = 0.3;
    for i = 1:numel(x)
        state = [x(i), y(i), theta]; 
        u = brs_planner.get_avoid_u(state'); 
        new_state = brs_planner.use_avoid_control(state); 
        x1 = [x1, x(i)]; 
        y1 = [y1, y(i)];
        x2 = [x2, new_state(1)]; 
        y2 = [y2, new_state(2)]; 
        u1 = [u1, new_state(1) - x(i)]; 
        v1 = [v1, new_state(2) - y(i)];
    end
    subplot(m, 4, t);
    hold on;
    title(sprintf('Theta %f', theta));
    xlabel('x(m)');
    ylabel('y(m)');
    quiver(x1, y1, u1, v1, 0, 'DisplayName', 'optimal control');
    contour(x, y, data2d, 'DisplayName', 'value fun', 'color', '#CC1FCB');
    legend('Location', 'NorthWest');
    savefig('./brs_planner_avoid_u.fig')
end 

