close all
clear all
clc

z = peaks(300);
surfl(z);
colormap copper
shading interp
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
view(3)
figure()
contour(z,16);
set(gca,'FontSize',14, 'FontName', 'CMU Serif')