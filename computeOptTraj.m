function [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
    % [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
    %   Computes the optimal trajectories given the optimal value function
    %   represented by (g, data), associated time stamps tau, dynamics given in
    %   dynSys.
    %
    % Inputs:
    %   g, data - grid and value function
    %   tau     - time stamp (must be the same length as size of last dimension of
    %                         data)
    %   dynSys  - dynamical system object for which the optimal path is to be
    %             computed
    %   extraArgs
    %     .uMode        - specifies whether the control u aims to minimize or
    %                     maximize the value function
    %     .visualize    - set to true to visualize results
    %     .fig_num:   List if you want to plot on a specific figure number
    %     .projDim      - set the dimensions that should be projected away when
    %                     visualizing
    %     .fig_filename - specifies the file name for saving the visualizations
    tauLength = length(tau);
    uMode = 'min';
    visualize = true;
    
    if isfield(extraArgs, 'optTrajDt')
      optTrajDt = extraArgs.optTrajDt;
    end

    clns = repmat({':'}, 1, g.dim);

    if any(diff(tau)) < 0
      error('Time stamps must be in ascending order!')
    end

    % Initialize trajectory
    traj = zeros(5, 1);
    tEarliest = 1;
    traj_idx = 1;
    t = 0;
    traj_tau = [];
    while 1
      % Determine the earliest time that the current state is in the reachable set
      % Binary search
      upper = tauLength;
      lower = tEarliest;

      tEarliest = find_earliest_BRS_ind(g, data, dynSys.x, upper, lower);

      % BRS at current time
      BRS_at_t = data(clns{:}, tEarliest);

      if tEarliest == tauLength
        % Trajectory has entered the target
        % TODO: change stop condition
        break
      end

      % Update trajectory
      grad = computeGradients(g, BRS_at_t);
      deriv_at_x = eval_u(g, grad, dynSys.x);
      u = dynSys.optCtrl(tau(tEarliest), dynSys.x, deriv_at_x, uMode);
      traj(:, traj_idx) = [dynSys.x(:); u(:)];
      traj_tau(end + 1) = t;
      % Visualize BRS corresponding to current trajectory point
      if visualize
        scatter(traj(1, traj_idx), traj(2, traj_idx), 50, 'green', 'o', 'filled');
        hold on
        hideDim = [0, 0, 1];
        pDim = 3;
        [g2D, data2D] = proj(g, BRS_at_t, hideDim, traj(pDim, traj_idx));
        visSetIm(g2D, data2D);
        tStr = sprintf('t = %.3f; tEarliest = %.3f', t, tau(tEarliest));
        title(tStr)
        drawnow
        hold off
      end
      
      traj_idx = traj_idx + 1;
      dynSys.updateState(u, optTrajDt, dynSys.x);
      t = t + optTrajDt;
    end
end