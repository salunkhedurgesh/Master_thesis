clear all;
close all;
clc

h = 0.66; hk= 0; a = 1; a_prime = 2; t = 3;

%     hk =0; %For simplicity in the beginning, I am not involving hk
%     i = 0; %iteration keeper
%     act_range = [t+h-0.5, t+h+0.5]; %Actuator stroke length, "t+h" is assumed to be the mid-position of the actuator
%     quality = 0; %initializing the value to calculate quality for each loop
%     c_qual = 0; %initializing the value to calculate cumulative quality 
min_v = 1;
    for alpha = -1 : 0.1 :1
        for beta = -1 :0.1 :1
            %Singularity check
            val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
            val_den = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2)*sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2);
            
            plot3(alpha,beta, (val/val_den), '*b');
            hold on;
           if abs(val/val_den) < min_v
               min_v = abs(val/val_den);
           end
        end
    end