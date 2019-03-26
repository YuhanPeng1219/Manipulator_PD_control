%% *********************** Problem 1 **************************
% initial input
syms t a1 a2 
syms theta1 theta2 real
q1 = [0; 0]; q2 = [a1; 0];
w = 1; w_hat = [0,-1;1,0];
I = eye(2);
g02 = [I,[a1+a2;0];0,0,1];

% compute xigma hat
v1 = w_hat * q1; v2 = w_hat * q2;
xi1_hat = [w_hat, -v1;0,0,0]; xi2_hat = [w_hat, -v2;0,0,0];

% compute matrix exponentia 
e_xi1theta1 = simplify(expm(xi1_hat*theta1));e_xi1theta2 = simplify(expm(xi2_hat*theta2));

% Transformation matrix C3 relative to C0
e_xitheta = simplify(e_xi1theta1 * e_xi1theta2 * g02)
