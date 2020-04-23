function [rho1,rho2] = cad_rho(a,a_prime,h,t,alpha,beta)
    [rho1,rho2] = get_rho(a,a_prime,h,t,alpha,beta);
    rho1 = rho1*364.31;
    rho2 = rho2*364.31;
end