%// Create a quiver3 as we normally would (could also be 2D quiver)

x = 1:10;
y = 1:10;
th = linspace(-pi, pi, 100);
[X,Y] = meshgrid(x, y);
Z = zeros(size(X));
U = reshape(cos(th), size(X));
V = reshape(sin(th), size(Y));
W = sqrt(X.^2 + Y.^2);

q = quiver(X, Y, U, V);

%// Compute the magnitude of the vectors
mags = sqrt(X.^2 + Y.^2);
mags = mags(:);
mags = [ones(50, 1)', zeros(50, 1)']';
colormap('winter');
set_quiver_colors(q, mags)
q.HandleVisibility = 'off';
q.ShowArrowHead = 'on';
q.AutoScale = 'on';
q.AutoScaleFactor = 0.3;

figure(10)
clf  
x = [1, 2, 3, 4];
y = x + 1;
ths = [0, pi/2, pi, 3*pi/2];
probs = [0, 1, 0.5, 0.75];
hold on
plot_traj_probs(x, y, ths, probs, 'chuck');
legend

function set_quiver_colors(q, probs)
    %// Get the current colormap
    currentColormap = colormap(gca);

    %// Now determine the color to make each arrow using a colormap
    [~, ~, ind] = histcounts(probs, size(currentColormap, 1));

    %// Now map this to a colormap to get RGB
    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
    cmap(:,:,4) = 255;
    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);

    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
    set(q.Head, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'

    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
    set(q.Tail, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:2,:,:), [], 4).');
end 

function plot_traj(xs, ys, ths, color, name)
    s = scatter(xs, ys, color, 'filled', 'DisplayName', name);
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.HandleVisibility = 'off';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.3;
end

function plot_traj_probs(xs, ys, ths, probs, name)
%     rgb_indexes = int32(probs * 255) + 1; 
%     currentColormap = colormap(gca);
%     M = currentColormap(rgb_indexes, :); 
%     s = scatter(xs, ys, [], M, 'filled', 'DisplayName', name);
    q = quiver(xs, ys, cos(ths), sin(ths));
    set_quiver_colors(q, probs); 
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.DisplayName = name;
    q.AutoScaleFactor = 0.3;
end
