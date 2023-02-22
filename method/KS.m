% This function uses Kalman smoothing to estimate velocity and acceleration
% from noisy position measures.


function [s_x, s_dx, s_ddx] = KS(x)
    % Initialize parameters
    Ts = 0.001;

    A = [ 1 Ts Ts^2/2; 
           0 1 Ts; 
           0 0 1];

    % Forward step - Standard Kalman filtering
    %[~,~,~,xfkk,xfk1k,Pfkk,Pfk1k] = KF(x,t,false);

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
    xfkk(:,1) = x0;
    Pfkk(:,:,1) = P0;
    xfk1k(:,1) = A*x0;
    Pfk1k(:,:,1) = A*P0+Q;
    % Recursive Kalman filter
    for k = 2:size(x,1)
        % Kalman gain
        K = Pfk1k(:,:,k-1)*C.'*inv(C*Pfk1k(:,:,k-1)*C.'+R);
        % Update current estimations
        xfkk(:,k) = xfk1k(:,k-1) + K*(x(k,1)-C*xfk1k(:,k-1));
        Pfkk(:,:,k) = Pfk1k(:,:,k-1) - K*C*Pfk1k(:,:,k-1);
        % Update previous estimations
        xfk1k(:,k) = A*xfkk(:,k);
        Pfk1k(:,:,k) = A*Pfkk(:,:,k)*A.' + Q;
    end


    % First iteration
    xskN = zeros(3,size(x,1));
    PskN = zeros(3,3,size(x,1));
    xskN(:,size(x,1)) = xfkk(:,size(x,1));
    PskN(:,:,size(x,1)) = Pfkk(:,:,size(x,1));
    % Backward step - Smoothing
    for k = size(x,1)-1:-1:1
        K = Pfkk(:,:,k)*A.'*inv(Pfk1k(:,:,k));
        xskN(:,k) = xfkk(:,k) + K*(xskN(:,k+1) - xfk1k(:,k));
        PskN(:,:,k) = Pfkk(:,:,k) + K*(PskN(:,:,k+1) - Pfk1k(:,:,k));
    end
    s_x = xskN(1,:);
    s_dx = xskN(2,:);
    s_ddx = xskN(3,:);
end