function plot_traj(xs, ys, ths, color, name)
    s = scatter(xs, ys, 15, 'black', 'filled', 'DisplayName', name);
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.1;
end