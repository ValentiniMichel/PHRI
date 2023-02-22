function [f_x, f_dx, f_ddx, xkk, xk1k, Pkk, Pk1k] = KF(x, t, ss)
    % Initialize parameters
    Ts = mean(diff(t));
    %Matrix from the slide 
    A = [1 Ts Ts^2/2; 
         0 1 Ts; 
         0 0 1];

    B = [Ts^3/6 Ts^2/2 Ts]';

    C = [1 0 0];

    % Variance matrix
    % Model variance 
    Q = (B*B')*10000000;
    % Noise Variance
    R = var(x);
    
    P0 = diag([1,1,1]);
    x0 = [0 0 0]';
    % Initial Conditions
    xkk(:,1) = x0;
    Pkk(:,:,1) = P0;
    if ss == true
        % Get P_inf by solving the Riccati equation
        [Pk1k,~,~] = idare(A.', C.', Q, R);
        %calculate K 
        K = Pk1k*C.'*pinv(C*Pk1k*C.' + R); % Steady state Kalman gain
    end
    % Recursive Kalman filter
    for k = 2:size(x,1)
        if ss == false
            % Kalman gain
            K = Pkk(:,:,k-1)*C.'*inv(C*Pkk(:,:,k-1)*C.'+R);
            Pkk(:,:,k) = A*(Pkk(:,:,k-1) - K*C*Pkk(:,:,k-1))*A.' + Q;
        end
        % Update current estimations and update previous estimations
        xkk(:,k) = A*xkk(:,k-1) + K*(x(k,1)-C*xkk(:,k-1));
    end
    f_x = xkk(1,:);
    f_dx = xkk(2,:);
    f_ddx = xkk(3,:);
end