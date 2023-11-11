function example_lyapunov()

    % Example quaternion-based system dynamics (replace this with your system)
    dynamics = @(q) [-0.5*q(2); -0.5*q(1); 0.5*q(4); -0.5*q(3)];

    % Choose a quaternion for initial conditions
    q0 = [1; 0; 0; 0];

    % Time vector
    tspan = 0:0.01:10;

    % Integrate the system
    [t, Q] = ode45(@(t, q) dynamics(q), tspan, q0);
    Q

    % Compute Lyapunov function values
    V = zeros(size(Q, 1), 1);
    for i = 1:size(Q, 1)
        V(i) = norm(quaternion_log(Q(i, :)))^2
    end

    % Plot results
    figure;
    subplot(2, 1, 1);
    plot(t, Q);
    title('Quaternion Dynamics');

    subplot(2, 1, 2);
    plot(t, V);
    title('Lyapunov Function (\|log(q)\|^2)');

end