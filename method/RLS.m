% RLS
% This function uses Recursive Least Squares to estimate the parameters of 
% a DC motor model, given the vector X of angular acceleration and velocity,
% the vector Y of voltages and the forgetting factor lambda.

function [y_hat, beta_hat] = RLS(X, Y, lambda)    
    % Initial values
    x = X(1,:);
    y = Y(1);
    %quanto velocemente converge la regressione
    P = 0.1*eye(2);
    beta_hat = P*x.'*y;
    y_hat(1) = x*beta_hat;
    % Recursive Least Squares
    for k = 2:size(Y,1)
        x = X(k,:);
        y = Y(k);
        e = y - x*beta_hat;
        P = (P - (P*x.'*x*P)/(lambda + x*P*x.'))/lambda;
        K = P*x.';
        beta_hat = beta_hat + K*e;
        y_hat(k) = x*beta_hat;
    end
end