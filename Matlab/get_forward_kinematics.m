function [T, L4] = get_forward_kinematics(thetas)
% link_1 
alpha_0 = 0;
a_0 = 0;
d_1 = 345 / 1000;
d_1 = 0;
% link_2 
alpha_1 = -90;
a_1 = 20 / 1000;
d_2 = 0;
% link_3 
alpha_2 = 0;
a_2 = 260 / 1000;
d_3 = 0;
% link_4 
alpha_3 = -90;
a_3 = 20 / 1000;
d_4 = 260 / 1000;
% link_5 
alpha_4 = +90;
a_4 = 0;
d_5 = 0;
% link_6 
alpha_5 = -90;
a_5 = 0;
d_6 = 0;
       T1 = [cos((pi*thetas(1))/180), -sin((pi*thetas(1))/180), 0, a_0  ; cos((pi*alpha_0 )/180)*sin((pi*thetas(1))/180), cos((pi*alpha_0 )/180)*cos((pi*thetas(1))/180), -sin((pi*alpha_0 )/180), -d_1*sin((pi*alpha_0 )/180) ;sin((pi*alpha_0 )/180)*sin((pi*thetas(1))/180), sin((pi*alpha_0)/180)*cos((pi*thetas(1))/180),  cos((pi*alpha_0)/180),  d_1*cos((pi*alpha_0)/180) ;0,0,0,1];
       T2 = [cos((pi*thetas(2))/180), -sin((pi*thetas(2))/180), 0, a_1  ; cos((pi*alpha_1 )/180)*sin((pi*thetas(2))/180), cos((pi*alpha_1 )/180)*cos((pi*thetas(2))/180), -sin((pi*alpha_1 )/180), -d_2*sin((pi*alpha_1 )/180) ;sin((pi*alpha_1 )/180)*sin((pi*thetas(2))/180), sin((pi*alpha_1)/180)*cos((pi*thetas(2))/180),  cos((pi*alpha_1)/180),  d_2*cos((pi*alpha_1)/180) ;0,0,0,1];
       T3 = [cos((pi*thetas(3))/180), -sin((pi*thetas(3))/180), 0, a_2  ; cos((pi*alpha_2 )/180)*sin((pi*thetas(3))/180), cos((pi*alpha_2 )/180)*cos((pi*thetas(3))/180), -sin((pi*alpha_2 )/180), -d_3*sin((pi*alpha_2 )/180) ;sin((pi*alpha_2 )/180)*sin((pi*thetas(3))/180), sin((pi*alpha_2)/180)*cos((pi*thetas(3))/180),  cos((pi*alpha_2)/180),  d_3*cos((pi*alpha_2)/180) ;0,0,0,1];
       T4 = [cos((pi*thetas(4))/180), -sin((pi*thetas(4))/180), 0, a_3  ; cos((pi*alpha_3 )/180)*sin((pi*thetas(4))/180), cos((pi*alpha_3 )/180)*cos((pi*thetas(4))/180), -sin((pi*alpha_3 )/180), -d_4*sin((pi*alpha_3 )/180) ;sin((pi*alpha_3 )/180)*sin((pi*thetas(4))/180), sin((pi*alpha_3)/180)*cos((pi*thetas(4))/180),  cos((pi*alpha_3)/180),  d_4*cos((pi*alpha_3)/180) ;0,0,0,1];
       T5 = [cos((pi*thetas(5))/180), -sin((pi*thetas(5))/180), 0, a_4  ; cos((pi*alpha_4 )/180)*sin((pi*thetas(5))/180), cos((pi*alpha_4 )/180)*cos((pi*thetas(5))/180), -sin((pi*alpha_4 )/180), -d_5*sin((pi*alpha_4 )/180) ;sin((pi*alpha_4 )/180)*sin((pi*thetas(5))/180), sin((pi*alpha_4)/180)*cos((pi*thetas(5))/180),  cos((pi*alpha_4)/180),  d_5*cos((pi*alpha_4)/180) ;0,0,0,1];
       T6 = [cos((pi*thetas(6))/180), -sin((pi*thetas(6))/180), 0, a_5  ; cos((pi*alpha_5 )/180)*sin((pi*thetas(6))/180), cos((pi*alpha_5 )/180)*cos((pi*thetas(6))/180), -sin((pi*alpha_5 )/180), -d_6*sin((pi*alpha_5 )/180) ;sin((pi*alpha_5 )/180)*sin((pi*thetas(6))/180), sin((pi*alpha_5)/180)*cos((pi*thetas(6))/180),  cos((pi*alpha_5)/180),  d_6*cos((pi*alpha_5)/180) ;0,0,0,1];
       L2 = T1 * T2;
       L3 = L2 * T3;
       L4 = L3 * T4;
       L5 = L4 * T5;
       T = L5 * T6;
end
