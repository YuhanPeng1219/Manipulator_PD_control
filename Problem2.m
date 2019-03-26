%% ***************************** Question a *******************************
% initial input
syms m1 m2 g a1 a2 q1(t) q2(t) q1dot(t) q2dot(t)
P = m1*g*a1*sin(q1)+m2*g*(a1*sin(q1)+a2*sin(q1+q2));
T = 0.5*m1*a1^2*q1dot^2+0.5*m2*((a1^2+2*a1*a2*cos(q2)+a2^2)...
    *q1dot^2+2*(a2^2+a1*a2*cos(q2))*q1dot*q2dot+a2^2*q2dot);
L = T-P;
% compute derivative
partialL_theta = simplify([functionalDerivative(L, q1); ...
                           functionalDerivative(L, q2)]);
partialL_thetadot = simplify([functionalDerivative(L, q1dot); ...
                              functionalDerivative(L, q2dot)]);
% compute tau
diff_t = simplify(diff(partialL_thetadot, t));
diff_t = subs(diff_t, diff(q2, t), q2dot);
diff_t = subs(diff_t, diff(q1, t), q1dot);      
tau = simplify(diff_t - partialL_theta);
%% ***************************** Question d *******************************
M = [m1*a1^2+m2*(a1^2+a2^2+2*a1*a2*cos(q2)), m2*(a2^2+a1*a2*cos(q2));
     m2*(a2^2+a1*a2*cos(q2)), m2*a2^2];
C = [-m2*a1*a2*sin(q2)*q2dot, -m2*a1*a2*sin(q2)*q1dot-m2*a1*a2*sin(q2)*q2dot;
     m2*a1*a2*sin(q2)*q1dot, 0];
N = [m1*g*a1*cos(q1)+m2*g*(a1*cos(q1)+a2*cos(q2));
     m2*g*a2*cos(q1+q2)];
Mdot = diff(M, t);
Mdot = subs(Mdot, diff(q2, t), q2dot);
Test = Mdot - 2*C;
M2C_test = transpose(Test)+Test
