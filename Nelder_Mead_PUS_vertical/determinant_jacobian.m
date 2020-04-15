close all
clear all
clc


syms theta delta a a_prime h t offset

    hk = 0;
     a1 = cos(delta)*a_prime+sin(delta)*h;
     a2 = sin(theta)*sin(delta)*a_prime-sin(theta)*cos(delta)*h;
     a3 = -cos(theta)*sin(delta)*a_prime+cos(theta)*cos(delta)*h+t;
     
     b1 = sin(delta)*h;
     b2 = cos(theta)*a_prime-sin(theta)*cos(delta)*h;
     b3 = sin(theta)*a_prime+cos(theta)*cos(delta)*h+t;

%     rho1 = -cos(theta)*sin(delta)*a_prime+cos(theta)*cos(delta)*h+t + sqrt(a^2 - (sin(theta)*sin(delta)*a_prime-sin(theta)*cos(delta)*h)^2 - (offset - (cos(delta)*a_prime+sin(delta)*h))^2);
%     rho2 = -cos(theta)*sin(delta)*a_prime+cos(theta)*cos(delta)*h+t + sqrt(a^2 - (cos(delta)*a_prime+sin(delta)*h)^2 - (offset - (sin(theta)*sin(delta)*a_prime-sin(theta)*cos(delta)*h))^2);
    rho1 = a3 + sqrt(a^2 - a2^2 - (offset - a1)^2);
    rho2 = b3 + sqrt(a^2 - b1^2 - (offset - b2)^2);
    J11 = diff(rho1,theta);
    J12 = diff(rho1,delta);
    J21 = diff(rho2,theta);
    J22 = diff(rho2,delta);
    
    jac_det = J11*J22 - J12*J21;    