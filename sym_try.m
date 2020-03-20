syms a a_prime h hk t alpha beta;

origin = [0,0,0];
origin_mobile = [0,0,t,1];
u11 = [a,0,0];
s12_mobile = [a_prime, hk,h];
u21 = [0,a,0];
s22_mobile = [hk,a_prime, h];

fixed_T_mobile = [cos(beta),0,sin(beta),0;
                  sin(alpha)*sin(beta) cos(alpha) -sin(alpha)*cos(beta) 0;
                  -cos(alpha)*sin(beta) sin(alpha) cos(alpha)*cos(beta) t;
                    0,0,0,1];

s12 = zeros(3,1); 
s22 = zeros(3,1); 
a11 =  a_prime*fixed_T_mobile(1,1) + hk*fixed_T_mobile(1,2) + h*fixed_T_mobile(1,3) + fixed_T_mobile(1,4) - a;
a12 =  a_prime*fixed_T_mobile(2,1) + hk*fixed_T_mobile(2,2) + h*fixed_T_mobile(2,3) + fixed_T_mobile(2,4);
a13 =  a_prime*fixed_T_mobile(3,1) + hk*fixed_T_mobile(3,2) + h*fixed_T_mobile(3,3) + fixed_T_mobile(3,4);
a21 =  hk*fixed_T_mobile(1,1) + a_prime*fixed_T_mobile(1,2) + h*fixed_T_mobile(1,3) + fixed_T_mobile(1,4);
a22 =  hk*fixed_T_mobile(2,1) + a_prime*fixed_T_mobile(2,2) + h*fixed_T_mobile(2,3) + fixed_T_mobile(2,4) - a;
a23 =  hk*fixed_T_mobile(3,1) + a_prime*fixed_T_mobile(3,2) + h*fixed_T_mobile(3,3) + fixed_T_mobile(3,4);

closed_rho1 = s12 - [a,0,0,1];
closed_rho2 = s22 - [0,a,0,1];

J11 = diff(sqrt(a11^2 + a12^2 + a13^2),alpha);
J12 = diff(sqrt(a11^2 + a12^2 + a13^2),beta);
J21 = diff(sqrt(a21^2 + a22^2 + a23^2),alpha);
J22 = diff(sqrt(a21^2 + a22^2 + a23^2),beta);
