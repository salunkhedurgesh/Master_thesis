%Some hierarchy optimization stuff
clear all;
close all
clc

Points = [1.194547,0.572612,0.238112,3.792359;
          0.683145,0.585041,0.188619,3.168192;
          0.896380,1.064090,0.075951,3.828311;
          0.532371,0.658285,-0.039942,2.955950;
          0.385192,1.051424,0.221648,3.401879;
          0.656764,0.730217,0.295060,2.273902;
          0.802561,0.554481,0.140402,3.907355;
          0.627915,0.627803,0.188482,2.895103;
          0.263446,0.408718,0.049611,3.291708;
          0.521865,0.694649,0.067045,2.397183];
      
Points_eval = [40319;40283;31712;40123;30488;28121;40311;40243;40323;29972];

for i = 1:length(Points_eval)
    Points(i,3) = abs(Points(i,3));
    para_weight = (Points(i,1) + Points(i,2))*(Points(i,3)+Points(i,4))*(Points(i,1)/Points(i,2));
    para_weight_off = ((Points(i,1) + Points(i,2))*(Points(i,3)+Points(i,4))*(Points(i,1)/Points(i,2)))+3;
    para_add(i) = Points(i,1)+Points(i,2)+Points(i,3)+Points(i,4);
    Points_reval(i) = Points_eval(i)/para_weight;  
    Points_reval_off(i) = Points_eval(i)/para_weight_off;  
end
