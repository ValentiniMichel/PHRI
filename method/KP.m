% This function uses Kalman filtering to estimate velocity and acceleration
% from noisy position measures.

function [p_x, p_dx, p_ddx, xf, Pf] = KP(x, t, ss)
    % Initialize parameters
    Ts = mean(diff(t));
    A = [ 1 Ts Ts^2/2; 
           0 1 Ts; 
           0 0 1];
    B = [Ts^3/6 Ts^2/2 Ts]';
    C = [1 0 0];
    Q = (B*B')*10000000;
    R = var(x);
    P0 = diag([1,1,1]);
    x0 = [0 0 0]';
    % First iteration
    xkk(:,1) = x0;
    Pkk(:,:,1) = P0;
    %the difference about KF and KP is the P
    if ss == true % Steady state
        % Get P_inf by solving the Riccati equation
        [Pk1k,~,~] = idare(A.', C.', Q, R);
        K = A*Pk1k*C.'*inv(C*Pk1k*C.' + R); % Steady state Kalman gain
    end
    % Recursive Kalman predictor
    for k = 2:size(x,1)
        if ss == false
            K = A*Pkk(:,:,k-1)*C.'*inv(C*Pkk(:,:,k-1)*C.'+R);
            
            Pkk(:,:,k) = A*Pkk(:,:,k-1)*A.'-K*C*Pkk(:,:,k-1)*A.'+Q;
        end
        xkk(:,k) = A*xkk(:,k-1) + K*(x(k,1)-C*xkk(:,k-1));
    end
    p_x = xkk(1,:);
    p_dx = xkk(2,:);
    p_ddx = xkk(3,:);
end