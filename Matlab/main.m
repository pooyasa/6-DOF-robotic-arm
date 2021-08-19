% Robot Mass: 26.5 Kg
% Robot total volume: 0.010996184 m^3
% Robot Density:   2409.9 Kg/m^3


m_1 = 3.77;
m_2 = 4.30;
m_3 = 1.86;
m_4 = 1.26;
m_5 = 0.64;
m_6 = 0.02;

I_1 = diag([16105757	18340813	23038929])  / 1000000;
I_2 = diag([18369961	20919228	26277824])  / 1000000;
I_3 = diag([3047738	5563810	5699532]) / 1000000 ;
I_4 = diag([2431980	3307978	4364563]) / 1000000 ;
I_5 = diag([576504	791542	863926]) / 1000000 ;
I_6 = diag([3259	3259	6078]) / 1000000 ;

PC_1 = [-3.47; 0; 79.91] / 1000;
PC_2 = [160; 10; 0] / 1000;
PC_3 = [46.73; 70.54; 0] / 1000;
PC_4 = [39.57; 0; -61.14;] / 1000;
PC_5 = [0; 0; 6.5] / 1000;
PC_6 = [0; 0; 0] / 1000;

%% Set the joint angles and limits
A1_limits = [- 170 , 170];
A2_limits = [- 80  , 140];
A3_limits = [- 110 , 155];
A4_limits = [- 175 , 175];
A5_limits = [- 120 , 120];
A6_limits = [- 350 , 350];

angles = zeros(3,7);
angles (:, 1) = [0, 1,2];
angles (:, 2) = [0, 0,0] / 180 * 3.14;
angles (:, 3) = [0, 22.5,45] / 180 * 3.14;
angles (:, 4) = ([0, 22.5,45]) / 180 * 3.14;
angles (:, 5) = [0, 0,0] / 180 * 3.14;
angles (:, 6) = [0, 0,0] / 180 * 3.14;
angles (:, 7) = [0, 90, 360] / 180 * 3.14;
time_steps = size(angles); time_steps = time_steps(1);
%% Denavit Hartenberg Parameters in degree and mm
% link_1 
alpha_0 = 0;
a_0 = 0;
d_1 = 345 / 1000;
d_1 = 0;
% link_2 
alpha_1 = -pi/2;
a_1 = 20 / 1000;
d_2 = 0;
% link_3 
alpha_2 = 0;
a_2 = 260 / 1000;
d_3 = 0;
% link_4 
alpha_3 = -pi/2;
a_3 = 20 / 1000;
d_4 = 260 / 1000;
% link_5 
alpha_4 = pi/2;
a_4 = 0;
d_5 = 0;
% link_6 
alpha_5 = -pi/2;
a_5 = 0;
d_6 = 0;
%% Calculate Transformation matrix for each link
syms alpha theta a d 
Rx = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0 ; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
Rz = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
Dx = [1 0 0 a;0 1 0 0 ; 0 0 1 0; 0 0 0 1];
Dz = [1 0 0 0;0 1 0 0 ; 0 0 1 d; 0 0 0 1];
T = Rx * Dx * Rz * Dz;

T1 = subs(T, [alpha, theta, a , d], [sym('alpha_0'), sym('theta_1'), sym('a_0'),sym('d_1')]);
T2 = subs(T, [alpha, theta, a , d], [sym('alpha_1'), sym('theta_2'), sym('a_1'),sym('d_2')]);
T3 = subs(T, [alpha, theta, a , d], [sym('alpha_2'), sym('theta_3'), sym('a_2'),sym('d_3')]);
T4 = subs(T, [alpha, theta, a , d], [sym('alpha_3'), sym('theta_4'), sym('a_3'),sym('d_4')]);
T5 = subs(T, [alpha, theta, a , d], [sym('alpha_4'), sym('theta_5'), sym('a_4'),sym('d_5')]);
T6 = subs(T, [alpha, theta, a , d], [sym('alpha_5'), sym('theta_6'), sym('a_5'),sym('d_6')]);
 
T = T1 * T2 * T3 * T4 * T5 * T6;
%% Calculating joints positions and orientations
A1 = A1_limits(1):2:A1_limits(2);   %Swivel Base Rotation
A2 = A2_limits(1):2:A2_limits(2);   %Lower Arm Rotation
A3 = A3_limits(1):2:A3_limits(2);   %Upper Arm Rotation
A4 = A4_limits(1):2:A4_limits(2);   %Arm Roll
A5 = A5_limits(1):2:A5_limits(2);   %Wrist Bend
A6 = A6_limits(1):2:A6_limits(2);   %Tool Flange

A1 = [0];
%A2 = [0];
%A3 = [0];
A4 = [0];
A5 = [0];
A6 = [0];

A2 = A2 - 90;
A3 = A3 - 90;

alpha_0 = 0;
alpha_1 = -180/2;
alpha_2 = 0;
alpha_3 = -180/2;
alpha_4 = 180/2;
alpha_5 = -180/2;

end_effector_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );
end_effector_orientation = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 , 3);
link_0_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );
link_1_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );
link_2_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );
link_3_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );
link_4_coordinates = zeros(length(A1) * length(A2) * length(A3) *length(A4) * length(A5) * length(A6), 3 );


counter = 1;
for i = 1:length(A1)
   for j = 1:length(A2)
        for k = 1:length(A3)
            for l = 1:length(A4)
                for m = 1:length(A5)
                    for n = 1:length(A6)
                       T1 = [cos((pi*A1(i))/180), -sin((pi*A1(i))/180), 0, a_0  ; cos((pi*alpha_0 )/180)*sin((pi*A1(i))/180), cos((pi*alpha_0 )/180)*cos((pi*A1(i))/180), -sin((pi*alpha_0 )/180), -d_1*sin((pi*alpha_0 )/180) ;sin((pi*alpha_0 )/180)*sin((pi*A1(i))/180), sin((pi*alpha_0)/180)*cos((pi*A1(i))/180),  cos((pi*alpha_0)/180),  d_1*cos((pi*alpha_0)/180) ;0,0,0,1];
                       T2 = [cos((pi*A2(j))/180), -sin((pi*A2(j))/180), 0, a_1  ; cos((pi*alpha_1 )/180)*sin((pi*A2(j))/180), cos((pi*alpha_1 )/180)*cos((pi*A2(j))/180), -sin((pi*alpha_1 )/180), -d_2*sin((pi*alpha_1 )/180) ;sin((pi*alpha_1 )/180)*sin((pi*A2(j))/180), sin((pi*alpha_1)/180)*cos((pi*A2(j))/180),  cos((pi*alpha_1)/180),  d_2*cos((pi*alpha_1)/180) ;0,0,0,1];
                       T3 = [cos((pi*A3(k))/180), -sin((pi*A3(k))/180), 0, a_2  ; cos((pi*alpha_2 )/180)*sin((pi*A3(k))/180), cos((pi*alpha_2 )/180)*cos((pi*A3(k))/180), -sin((pi*alpha_2 )/180), -d_3*sin((pi*alpha_2 )/180) ;sin((pi*alpha_2 )/180)*sin((pi*A3(k))/180), sin((pi*alpha_2)/180)*cos((pi*A3(k))/180),  cos((pi*alpha_2)/180),  d_3*cos((pi*alpha_2)/180) ;0,0,0,1];
                       T4 = [cos((pi*A4(l))/180), -sin((pi*A4(l))/180), 0, a_3  ; cos((pi*alpha_3 )/180)*sin((pi*A4(l))/180), cos((pi*alpha_3 )/180)*cos((pi*A4(l))/180), -sin((pi*alpha_3 )/180), -d_4*sin((pi*alpha_3 )/180) ;sin((pi*alpha_3 )/180)*sin((pi*A4(l))/180), sin((pi*alpha_3)/180)*cos((pi*A4(l))/180),  cos((pi*alpha_3)/180),  d_4*cos((pi*alpha_3)/180) ;0,0,0,1];
                       T5 = [cos((pi*A5(m))/180), -sin((pi*A5(m))/180), 0, a_4  ; cos((pi*alpha_4 )/180)*sin((pi*A5(m))/180), cos((pi*alpha_4 )/180)*cos((pi*A5(m))/180), -sin((pi*alpha_4 )/180), -d_5*sin((pi*alpha_4 )/180) ;sin((pi*alpha_4 )/180)*sin((pi*A5(m))/180), sin((pi*alpha_4)/180)*cos((pi*A5(m))/180),  cos((pi*alpha_4)/180),  d_5*cos((pi*alpha_4)/180) ;0,0,0,1];
                       T6 = [cos((pi*A6(n))/180), -sin((pi*A6(n))/180), 0, a_5  ; cos((pi*alpha_5 )/180)*sin((pi*A6(n))/180), cos((pi*alpha_5 )/180)*cos((pi*A6(n))/180), -sin((pi*alpha_5 )/180), -d_6*sin((pi*alpha_5 )/180) ;sin((pi*alpha_5 )/180)*sin((pi*A6(n))/180), sin((pi*alpha_5)/180)*cos((pi*A6(n))/180),  cos((pi*alpha_5)/180),  d_6*cos((pi*alpha_5)/180) ;0,0,0,1];
                       L2 = T1 * T2;
                       L3 = L2 * T3;
                       L4 = L3 * T4;
                       L5 = L4 * T5;
                       TW = L5 * T6;
                       end_effector_coordinates(counter , :) = [TW(1 , 4) , TW(2 , 4) ,TW(3 , 4) ];
                       end_effector_orientation(counter, :, :) = [TW(1 , 1) , TW(1 , 2) ,TW(1 , 3); TW(2 , 1) , TW(2 , 2) ,TW(2 , 3);TW(3 , 1) , TW(3 , 2) ,TW(3 , 3)];
                       link_0_coordinates(counter , :) = [T1(1 , 4) , T1(2 , 4) ,T1(3 , 4) ];
                       link_1_coordinates(counter , :) = [L2(1 , 4) , L2(2 , 4) ,L2(3 , 4) ];
                       link_2_coordinates(counter , :) = [L3(1 , 4) , L3(2 , 4) ,L3(3 , 4) ];
                       link_3_coordinates(counter , :) = [L4(1 , 4) , L4(2 , 4) ,L4(3 , 4) ];
                       link_4_coordinates(counter , :) = [L5(1 , 4) , L5(2 , 4) ,L5(3 , 4) ];
                       counter = counter + 1;
                    end
                end
            end 
        end
        counter 
    end 
end
hold on;
% scatter3(link_0_coordinates(:, 1),link_0_coordinates(:, 2), link_0_coordinates(:, 3));
% scatter3(link_1_coordinates(:, 1),link_1_coordinates(:, 2), link_1_coordinates(:, 3));
% scatter3(link_2_coordinates(:, 1),link_2_coordinates(:, 2), link_2_coordinates(:, 3));
% scatter3(link_3_coordinates(:, 1),link_3_coordinates(:, 2), link_3_coordinates(:, 3));
% scatter3(link_4_coordinates(:, 1),link_4_coordinates(:, 2), link_4_coordinates(:, 3));
%scatter3(end_effector_coordinates(:, 1),end_effector_coordinates(:, 2), end_effector_coordinates(:, 3));
scatter(end_effector_coordinates(:, 1), end_effector_coordinates(:, 3));
axis equal;
grid on;
legend("A1", "A2",  "A3", "A4", "A5", "EE");
%% Extract Cartesian coordinates of end effector from simulink output
time = out.end_effector_position.Time;
x = out.end_effector_position.Data(:, 1) * 1000;
y = out.end_effector_position.Data(:, 2) * 1000;
z = out.end_effector_position.Data(:, 3) * 1000;
scatter3(x,y,z);

%% Calculating Fixed and Euler angles
i = 1;
r = [end_effector_orientation(i, 1,1) end_effector_orientation(i, 1,2) end_effector_orientation(i, 1,3); end_effector_orientation(i, 2,1) end_effector_orientation(i, 2,2) end_effector_orientation(i, 2,3); end_effector_orientation(i, 3,1) end_effector_orientation(i, 3,2) end_effector_orientation(i, 3,3)];
beta = atan2(-r(3,1), sqrt(r(1,1)^2 + r(2,1)^2));
alpha = atan2(r(2,1) / cos(beta) , r(1,1)/ cos(beta));
gamma = atan2(r(3,2) / cos(beta), r(3,3) / cos(beta));

%% Calculating Equivalent angle-axis and Euler parameters
theta = acos((r(1,1) + r(2,2) + r(3,3) - 1) / 2);
K = [r(3,2) - r(2,3); r(1,3) - r(3,1); r(2,1) - r(1,2)] / (2 * sin(theta));
ep_1 = K(1) * sin(theta / 2);
ep_2 = K(2) * sin(theta / 2);
ep_3 = K(3) * sin(theta / 2);
ep_4 = cos(theta / 2); 

W_abs = ep_1^2 + ep_2^2 + ep_3^2 + ep_4^2;

%% Part 6, Velocity propagation and basic jacobian exctraction
%   V_i+1 = V_i + w_i * P_i+1 + d_i+1.Z_i+1
%   w_i+1 = w_i + theta_dot_i+1*Z_i+1
T1 = subs(T1, [sym('alpha_0') sym('d_1') sym('a_0')] , [alpha_0 d_1 a_0]);
T2 = subs(T2, [sym('alpha_1') sym('d_2') sym('a_1')] , [alpha_1 d_2 a_1]);
T3 = subs(T3, [sym('alpha_2') sym('d_3') sym('a_2')] , [alpha_2 d_3 a_2]);
T4 = subs(T4, [sym('alpha_3') sym('d_4') sym('a_3')] , [alpha_3 d_4 a_3]);
T5 = subs(T5, [sym('alpha_4') sym('d_5') sym('a_4')] , [alpha_4 d_5 a_4]);
T6 = subs(T6, [sym('alpha_5') sym('d_6') sym('a_5')] , [alpha_5 d_6 a_5]);


L2 = (T1 * T2);
L3 = (L2 * T3);
L4 = (L3 * T4);
L5 = (L4 * T5);
TW = (L5 * T6);

R0_1 = T1(1:3, 1:3); P0_1 = T1(1:3, 4);
R1_2 = T2(1:3, 1:3); P1_2 = T2(1:3, 4);
R2_3 = T3(1:3, 1:3); P2_3 = T3(1:3, 4);
R3_4 = T4(1:3, 1:3); P3_4 = T4(1:3, 4);
R4_5 = T5(1:3, 1:3); P4_5 = T5(1:3, 4);
R5_6 = T6(1:3, 1:3); P5_6 = T6(1:3, 4);

Z = [0; 0; 1];
Z1 = T1(1:3, 3);
Z2 = L2(1:3, 3);
Z3 = L3(1:3, 3);
Z4 = L4(1:3, 3);
Z5 = L5(1:3, 3);
Z6 = TW(1:3, 3);

P1 = T1(1:3, 4);
P2 = L2(1:3, 4);
P3 = L3(1:3, 4);
P4 = L4(1:3, 4);
P5 = L5(1:3, 4);
P6 = TW(1:3, 4);

syms theta_dot_1 theta_dot_2 theta_dot_3 theta_dot_4 theta_dot_5 theta_dot_6
%   V_i+1 = V_i + w_i * P_i+1 + d_i+1.Z_i+1
%   w_i+1 = w_i + theta_dot_i+1*Z_i+1
%i = 0
V1 = [0; 0; 0; ];
w1 = (theta_dot_1 * Z1);
%i = 1
V2 = transpose(R1_2) * (V1 + cross(w1, P1_2));
w2 = transpose(R1_2) * w1 + theta_dot_2 * Z;
%i = 2
V3 = transpose(R2_3) * (V2 + cross(w2, P2_3));
w3 = transpose(R2_3) * w2 + theta_dot_3 * Z;
%i = 3
V4 = transpose(R3_4) * (V3 + cross(w3, P3_4));
w4 = transpose(R3_4) * w3 + theta_dot_4 * Z;
%i = 4
V5 = transpose(R4_5) * (V4 + cross(w4, P4_5));
w5 = transpose(R4_5) * w4 + theta_dot_5 * Z;
%i = 5
V6 = transpose(R5_6) * (V5 + cross(w5, P5_6));
w6 = transpose(R5_6) * w5 + theta_dot_6 * Z;

V6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6 * V6;
w6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6 * w6;
%Deriving the coefs of basic jacobian matrix
basic_jacobian_propagation = sym('bjp',6);
%First Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_1);
    if length(cx) > 1
        basic_jacobian_propagation(i, 1) =  cx(2);
    else
        basic_jacobian_propagation(i, 1) =  0;
    end
    cx = coeffs(w6(i), theta_dot_1);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 1) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 1) =  0;
    end
end

%Second Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_2);
    if length(cx) > 1
        basic_jacobian_propagation(i, 2) =  cx(2);
    else
        basic_jacobian_propagation(i, 2) =  0;
    end
    cx = coeffs(w6(i), theta_dot_2);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 2) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 2) =  0;
    end
end

%Third Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_3);
    if length(cx) > 1
        basic_jacobian_propagation(i, 3) =  cx(2);
    else
        basic_jacobian_propagation(i, 3) =  0;
    end
    cx = coeffs(w6(i), theta_dot_3);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 3) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 3) =  0;
    end
end

%Fourth Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_4);
    if length(cx) > 1
        basic_jacobian_propagation(i, 4) =  cx(2);
    else
        basic_jacobian_propagation(i, 4) =  0;
    end
    cx = coeffs(w6(i), theta_dot_4);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 4) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 4) =  0;
    end
end
%Fifth Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_5);
    if length(cx) > 1
        basic_jacobian_propagation(i, 5) =  cx(2);
    else
        basic_jacobian_propagation(i, 5) =  0;
    end
    cx = coeffs(w6(i), theta_dot_5);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 5) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 5) =  0;
    end
end
%Sixth Column
for i = 1:3
    cx = coeffs(V6(i), theta_dot_6);
    if length(cx) > 1
        basic_jacobian_propagation(i, 6) =  cx(2);
    else
        basic_jacobian_propagation(i, 6) =  0;
    end
    cx = coeffs(w6(i), theta_dot_6);
    if length(cx) > 1
        basic_jacobian_propagation(i+3, 6) =  cx(2);
    else
        basic_jacobian_propagation(i+3, 6) =  0;
    end
end


%% Part 7, Explicit form jacobian extraction
% All joints are revolute 
basic_jacobian_explicit = get_jacobian(P6, [Z1, Z2, Z3, Z4, Z5, Z6]);

%% Part 8, Inverse kinematics
T_des = get_forward_kinematics([24, 80, 35, 55, 55, 68]);
P_desired = T_des(1:3, 4);
R_desired = T_des(1:3,1:3);
[theta_angles_1, theta_angles_2, theta_angles_3, theta_angles_4, theta_angles_5, theta_angles_6, is_successful] = get_inverse_kinematics(P_desired, R_desired);
[theta_angles_1, theta_angles_2, theta_angles_3, theta_angles_4, theta_angles_5, theta_angles_6] / pi * 180
if is_successful
    fprintf("Found %d solutions!\n", length(theta_angles_1));
    for i = 1:length(theta_angles_1);
        P_end = get_forward_kinematics([theta_angles_1(i), theta_angles_2(i), theta_angles_3(i), theta_angles_4(i), theta_angles_5(i), theta_angles_6(i)] / pi * 180);
        validation = round((P_end(1:3, 4) - P_desired) * 1000) / 1000;
        validation = round((P_end(1:3, 1:3) - R_desired) * 1000) / 1000;
        validation = round((T_des - P_end) * 1000 )/ 1000;
    end
else
    fprintf("Could not find an answer to inverse problem\n");
end


%% Part 9, Singular Points
x_s = get_singular_points();
end_cordinates = zeros([length(x_s), 3]);
for i=1:length(x_s)
    T = get_forward_kinematics(x_s(i, :));
    end_cordinates(i, :) = T(1:3, 4);
end

plot(end_effector_coordinates(:, 1), end_effector_coordinates(:, 3),'.' ,'Color',  [0.1, 0.2, 0.8]);
axis equal;
grid on;
hold on;
plot(end_cordinates(:, 1), end_cordinates(:, 3),'.' ,'Color',  [0.15, 0.15, 0.15]);
title("Singularities in workspace");
xlabel("X (m)")
ylabel("Z (m)")
legend(["Workspace", "Sigularities"])
%% Part 10, Dynamics

%% Iterative Newton Euler formulations, Notice that all variables are in their own axis
% w_i+1_dot = R_i+1 * w_dot_i + R_i+1 * w_i * theta_dot_i+1 * Z_i+1 + theta_dotdot_i+1 * Z_i+1
% v_dot_i+1 = R_i+1 * (w_dot_i * )
syms w_dot_1 w_dot_2 w_dot_3 w_dot_4 w_dot_5 w_dot_6
syms V_dot_1 V_dot_2 V_dot_3 V_dot_4 V_dot_5 V_dot_6
syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 
syms theta_dot_1 theta_dot_2 theta_dot_3 theta_dot_4 theta_dot_5 theta_dot_6
syms theta_dotdot_1 theta_dotdot_2 theta_dotdot_3 theta_dotdot_4 theta_dotdot_5 theta_dotdot_6

syms g;
Z = [0; 0; 1];

w_0     = [0; 0; 0];
V_0     = [0; 0; 0];
w_dot_0 = [0; 0; 0];
V_dot_0 = [0; 0; g];

%i = 0
w_1      = transpose(R0_1) * w_0 + theta_dot_1 * Z; % Correct
w_dot_1  = transpose(R0_1) * w_dot_0 + cross(transpose(R0_1) * w_0, theta_dot_1 * Z) + theta_dotdot_1 * Z; %correct
V_dot_1  = transpose(R0_1) * (cross(w_dot_0, P0_1) + cross(w_0, cross(w_0, P0_1)) + V_dot_0); %correct
VC_dot_1 = cross(w_dot_1, PC_1) + cross(w_1, cross(w_1,PC_1)) + V_dot_1; %correct
% VC_dot_1 = V_dot_1;

F_1 = m_1 * VC_dot_1; % correct
N_1 = I_1 * w_dot_1 + cross(w_1, I_1 * w_1); %correct
 
%i = 1
w_2      = transpose(R1_2) * w_1 + theta_dot_2 * Z; %correct
w_dot_2  = transpose(R1_2) * w_dot_1 + cross(transpose(R1_2) * w_1, theta_dot_2 * Z) + theta_dotdot_2 * Z;
V_dot_2  = transpose(R1_2) * (cross(w_dot_1, P1_2) + cross(w_1, cross(w_1, P1_2)) + V_dot_1);
VC_dot_2 = cross(w_dot_2, PC_2) + cross(w_2, cross(w_2,PC_2)) + V_dot_2;
% VC_dot_2 = V_dot_2;

F_2 = m_2 * VC_dot_2;
N_2 = I_2 * w_dot_2 + cross(w_2, I_2 * w_2);

%i = 2
w_3      = transpose(R2_3) * w_2 + theta_dot_3 * Z;
w_dot_3  = transpose(R2_3) * w_dot_2 + cross(transpose(R2_3) * w_2, theta_dot_3 * Z) + theta_dotdot_3 * Z;
V_dot_3  = transpose(R2_3) * (cross(w_dot_2, P2_3) + cross(w_2, cross(w_2, P2_3)) + V_dot_2);
VC_dot_3 = cross(w_dot_3, PC_3) + cross(w_3, cross(w_3, PC_3)) + V_dot_3;
% VC_dot_3 = V_dot_3;

F_3 = m_3 * VC_dot_3;
N_3 = I_3 * w_dot_3 + cross(w_3, I_3 * w_3);

%i = 3
w_4      = transpose(R3_4) * w_3 + theta_dot_4 * Z;
w_dot_4  = transpose(R3_4) * w_dot_3 + cross(transpose(R3_4) * w_3, theta_dot_4 * Z) + theta_dotdot_4 * Z;
V_dot_4  = transpose(R3_4) * (cross(w_dot_3, P3_4) + cross(w_3, cross(w_3, P3_4)) + V_dot_3);
VC_dot_4 = cross(w_dot_4, PC_4) + cross(w_4, cross(w_4, PC_4)) + V_dot_4;
% VC_dot_4 = V_dot_4;

F_4 = m_4 * VC_dot_4;
N_4 = I_4 * w_dot_4 + cross(w_4, I_4 * w_4);

%i = 4
w_5      = transpose(R4_5) * w_4 + theta_dot_5 * Z;
w_dot_5  = transpose(R4_5) * w_dot_4 + cross(transpose(R4_5) * w_4, theta_dot_5 * Z) + theta_dotdot_5 * Z;
V_dot_5  = transpose(R4_5) * (cross(w_dot_4, P4_5) + cross(w_4, cross(w_4, P4_5)) + V_dot_4);
VC_dot_5 = cross(w_dot_5, PC_5) + cross(w_5, cross(w_5, PC_5)) + V_dot_5;
% VC_dot_5 = V_dot_5;

F_5 = m_5 * VC_dot_5;
N_5 = I_5 * w_dot_5 + cross(w_5, I_5 * w_5);

%i = 5
w_6      = transpose(R5_6) * w_5 + theta_dot_6 * Z;
w_dot_6  = transpose(R5_6) * w_dot_5 + cross(transpose(R5_6) * w_5, theta_dot_6 * Z) + theta_dotdot_6 * Z;
V_dot_6  = transpose(R5_6) * (cross(w_dot_5, P5_6) + cross(w_5, cross(w_5, P5_6)) + V_dot_5);
VC_dot_6 = cross(w_dot_6, PC_6) + cross(w_6, cross(w_6, PC_6)) + V_dot_6;
% VC_dot_6 = V_dot_6;

F_6 = m_6 * VC_dot_6;
N_6 = I_6 * w_dot_6 + cross(w_6, I_6 * w_6);


%Inward Iterations:
% i = 6
f_6 = F_6;
n_6 = N_6 + cross(PC_6, F_6);
t_6 = transpose(n_6) * Z;
% i = 5
f_5 = R5_6 * f_6 + F_5;
n_5 = N_5 + R5_6 * n_6 + cross(PC_5, F_5) + cross(P5_6, R5_6 * f_6);
t_5 = transpose(n_5) * Z;
% i = 4
f_4 = R4_5 * f_5 + F_4;
n_4 = N_4 + R4_5 * n_5 + cross(PC_4, F_4) + cross(P4_5, R4_5 * f_5);
t_4 = transpose(n_4) * Z;
% i = 3
f_3 = R3_4 * f_4 + F_3;
n_3 = N_3 + R3_4 * n_4 + cross(PC_3, F_3) + cross(P3_4, R3_4 * f_4);
t_3 = transpose(n_3) * Z;
% i = 2
f_2 = R2_3 * f_3 + F_2;
n_2 = N_2 + R2_3 * n_3 + cross(PC_2, F_2) + cross(P2_3, R2_3 * f_3);
t_2 = transpose(n_2) * Z;
% i = 1
f_1 = R1_2 * f_2 + F_1;
n_1 = N_1 + R1_2 * n_2 + cross(PC_1, F_1) + cross(P1_2, R1_2 * f_2);
t_1 = transpose(n_1) * Z;

t = vpa([t_1; t_2; t_3; t_4; t_5; t_6 ]);
M_newton = vpa(t2M(t));
[C_newton, B_newton] = M2V(M_newton);
G_newton = t2G(t) * 10;

%% Lagrangian Formulation

syms theta_dotdot_1 theta_dotdot_2 theta_dotdot_3 theta_dotdot_4 theta_dotdot_5 theta_dotdot_6
syms theta_dot_1 theta_dot_2 theta_dot_3 theta_dot_4 theta_dot_5 theta_dot_6
syms g;
Zero = zeros(3, 1);
Z = [0; 0; 1];

R1_0 = transpose(R0_1);
R2_1 = transpose(R1_2);
R3_2 = transpose(R2_3);
R4_3 = transpose(R3_4);
R5_4 = transpose(R4_5);
R6_5 = transpose(R5_6);
Z21 = R2_1(1:3, 3);
Z32 = R3_2(1:3, 3);
Z43 = R4_3(1:3, 3);
Z54 = R5_4(1:3, 3);
Z65 = R6_5(1:3, 3);

R3_1 = transpose(R1_2 * R2_3);
R4_1 = transpose(R1_2 * R2_3 * R3_4);
R5_1 = transpose(R1_2 * R2_3 * R3_4 * R4_5);
R6_1 = transpose(R1_2 * R2_3 * R3_4 * R4_5 * R5_6);
Z31 = R3_1(1:3, 3);
Z41 = R4_1(1:3, 3);
Z51 = R5_1(1:3, 3);
Z61 = R6_1(1:3, 3);

R4_2 = transpose(R2_3 * R3_4);
R5_2 = transpose(R2_3 * R3_4 * R4_5);
R6_2 = transpose(R2_3 * R3_4 * R4_5 * R5_6);
Z42 = R4_2(1:3, 3);
Z52 = R5_2(1:3, 3);
Z62 = R6_2(1:3, 3);


R5_3 = transpose(R3_4 * R4_5);
R6_3 = transpose(R3_4 * R4_5 * R5_6);
Z53 = R5_3(1:3, 3);
Z63 = R6_3(1:3, 3);

R6_4 = transpose(R4_5 * R5_6);
Z64 = R6_4(1:3, 3);

g = [0; 0; 10];
PC_11 = T1 * [PC_1;1];PC_11 = PC_11(1:3);
PC_22 = L2 * [PC_2;1];PC_22 = PC_22(1:3);
PC_33 = L3 * [PC_3;1];PC_33 = PC_33(1:3);
PC_44 = L4 * [PC_4;1];PC_44 = PC_44(1:3);
PC_55 = L5 * [PC_5;1];PC_55 = PC_55(1:3);
PC_66 = TW * [PC_6;1];PC_66 = PC_66(1:3); %Correct!

j_1 = get_jacobian(PC_11, [Z, Zero, Zero, Zero, Zero, Zero]);
j_2 = get_jacobian(PC_22, [Z21, Z, Zero, Zero, Zero, Zero]);
j_3 = get_jacobian(PC_33, [Z31, Z32, Z, Zero, Zero, Zero]);
j_4 = get_jacobian(PC_44, [Z41, Z42, Z43, Z, Zero, Zero]);
j_5 = get_jacobian(PC_55, [Z51, Z52, Z53, Z54, Z, Zero]);
j_6 = get_jacobian(PC_66, [Z61, Z62, Z63, Z64, Z65, Z]);

M_lagrange =              m_1 * transpose(j_1(1:3, :)) * j_1(1:3, :) + transpose(j_1(4:6, :)) * I_1 * j_1(4:6, :);
M_lagrange = M_lagrange + m_2 * transpose(j_2(1:3, :)) * j_2(1:3, :) + transpose(j_2(4:6, :)) * I_2 * j_2(4:6, :);
M_lagrange = M_lagrange + m_3 * transpose(j_3(1:3, :)) * j_3(1:3, :) + transpose(j_3(4:6, :)) * I_3 * j_3(4:6, :);
M_lagrange = M_lagrange + m_4 * transpose(j_4(1:3, :)) * j_4(1:3, :) + transpose(j_4(4:6, :)) * I_4 * j_4(4:6, :);
M_lagrange = M_lagrange + m_5 * transpose(j_5(1:3, :)) * j_5(1:3, :) + transpose(j_5(4:6, :)) * I_5 * j_5(4:6, :);
M_lagrange = M_lagrange + m_6 * transpose(j_6(1:3, :)) * j_6(1:3, :) + transpose(j_6(4:6, :)) * I_6 * j_6(4:6, :);
M_lagrange = vpa(M_lagrange);
%M_lagrange = simplify(M_lagrange);
[C_lagrange, B_lagrange] = M2V(M_lagrange);
G_larange = -(transpose(j_1(1:3, :)) * m_1 * g + transpose(j_2(1:3, :)) * m_2 * g + transpose(j_3(1:3, :)) * m_3 * g + transpose(j_4(1:3, :)) * m_4 * g + transpose(j_5(1:3, :)) * m_5 * g + transpose(j_6(1:3, :)) * m_6 * g);

M_coeffs = [theta_dotdot_1; theta_dotdot_2; theta_dotdot_3; theta_dotdot_4; theta_dotdot_5; theta_dotdot_6];
C_Coeffs = [theta_dot_1^2; theta_dot_2^2; theta_dot_3^2; theta_dot_4^2 ;theta_dot_5^2; theta_dot_6^2;];
B_Coeffs = [theta_dot_1*theta_dot_2; theta_dot_1*theta_dot_3; theta_dot_1*theta_dot_4; theta_dot_1*theta_dot_5 ;theta_dot_1*theta_dot_6;
            theta_dot_2*theta_dot_3;theta_dot_2*theta_dot_4;theta_dot_2*theta_dot_5;theta_dot_2*theta_dot_6;
            theta_dot_3*theta_dot_4;theta_dot_3*theta_dot_5;theta_dot_3*theta_dot_6;
            theta_dot_4*theta_dot_5;theta_dot_4*theta_dot_6;
            theta_dot_5*theta_dot_6;];
        
t_Lagrange = M_lagrange * M_coeffs + C_lagrange * C_Coeffs + B_lagrange * B_Coeffs + G_larange;
%% Part 11 Create Trajectories, Quadratic and with blends
NUMBER_OF_POINTS = 1000;
T_F = 3;
ACCELERATION = [50, 25, 50, 50, 50, 300];

thetas_o = [0  , 0, 0, -0, 0, 0];
thetas_f = [0, 45, 45, 0, -0, 360];
%thetas_o = [130];
%thetas_f = [45];

[trajectories, trajectories_dot, trajectories_dotdot] = get_trajectories(thetas_o, thetas_f, NUMBER_OF_POINTS  , "parabolic_blends", T_F, ACCELERATION );
figure;
plot( linspace(0, T_F,NUMBER_OF_POINTS) , trajectories);
hold on;
[trajectories, trajectories_dot, trajectories_dotdot] = get_trajectories(thetas_o, thetas_f, NUMBER_OF_POINTS  , "cubic", T_F);
plot( linspace(0, T_F,NUMBER_OF_POINTS) , trajectories);
legend("Parabolic", "cubic");
xlabel("t (s)")
ylabel("angle (degree)")
title("Created Trajectories")

grid on;

% angles = zeros(2 * NUMBER_OF_POINTS, 7);
% angles (:, 1) = linspace(0, 2 * T_F, 2 * NUMBER_OF_POINTS);
% angles (1:floor(end/2), 2:7) = trajectories * pi / 180;
% for i = 1:NUMBER_OF_POINTS
%     angles (floor(end/2) + i, 2:7) = trajectories(end , :) * pi / 180;    
% end

%% Part 12, Time Optimal Trajectory

%% Part 13, Linear ARM Controller: eta >> 1 => Robot Dynamics are ignored
eta = 1;
I_m = [0.1,      0.05,   0.05,  0.025,  0.025, 0.01];
b_m = [0.0500, 0.0250, 0.0250, 0.0125, 0.0125, 0.0050];
I_eff = (eta^2) * I_m + [55, 35, 10, 5, 1, 0.06];
b_eff = (eta^2) * b_m;
for i =1:6
   
end
for i = 1:length(I_m)
   ROBOT_TF(i) = tf(1, [I_eff(i), b_eff(i) ,0]);   % Calculating arm transfer function 
end

% Tuned Parameters From PID tuner sisotool(G)
C_1 = pid(tf([0.1601, 0.1249, 0.0243], [0, 1, 0]));
C_2 = pid(tf([0.0780, 0.0706, 0.0143], [0, 1, 0]));
C_3 = pid(tf([0.0800, 0.0624, 0.0122], [0, 1, 0]));
C_4 = pid(tf([0.0400, 0.0312, 0.0061], [0, 1, 0]));
C_5 = pid(tf([0.0400, 0.0312, 0.0061], [0, 1, 0]));
C_6 = pid(tf([0.0160, 0.0125, 0.0024], [0, 1, 0]));
 
NUMBER_OF_POINTS = 1000;
T_F = 3;
thetas_o = [0  , 0, 0, -0, 0, 0];
thetas_f = [0, 45, 45, 0, -0, 45];
[trajectories, trajectories_dot, trajectories_dotdot] = get_trajectories(thetas_o, thetas_f, NUMBER_OF_POINTS  , "cubic", T_F);
trajectories_time = linspace(0, T_F, NUMBER_OF_POINTS);

%tau = M_newton* M_coeffs + C_newton * C_Coeffs + B_newton * B_Coeffs + G_larange;
time_step = 0.2;

initial_conditions = transpose([thetas_o;  0, 0, 0, 0, 0, 0]);
last_conditions = transpose([thetas_o;  0, 0, 0, 0, 0, 0]);

Kp = diag([250,   100,  20,     10,     5, 0.005]);
Kd = diag([500,   80,  15,     20,     5, 0.001]);
Ki = diag([0.243, 0.01, 0.0043, 0.0100, 0, 0]);
Kp = Kp /  (eta^2);
Kd = Kd /  (eta^2);
Ki = Ki /  (eta^2);

T_end = 15;
input = zeros(6, length(0:time_step:T_end));
output = zeros(6, length(0:time_step:T_end));
last_error = zeros(6, 1);
sum_error = zeros(6, 1);
i = 0;
tic
for t = 0:time_step:15
    t
    theta_desired = transpose(interp1(trajectories_time, trajectories, t,"linear", "extrap"));
    error = theta_desired - initial_conditions(1:6, 1);
    err = transpose(error)
    
    theta_dot_dot_current = (initial_conditions(1:6, 2) - last_conditions(1:6, 2)) / time_step;
    input_tau_m = Kp * error + Ki * sum_error * time_step + Kd * (error - last_error) / time_step;
    input_tau = eta * (input_tau_m - transpose(b_m) .* initial_conditions(1:6, 2) * eta - transpose(I_m) .* theta_dot_dot_current *eta);
    
    
    tau_input = transpose(input_tau)
    [theta, theta_dot] = Robot_Simulator(input_tau, initial_conditions, time_step);
    last_conditions = initial_conditions;
    initial_conditions = [theta, theta_dot];
    last_error = error;
    sum_error = sum_error + error;
    i  = i + 1;
    input(:, i) = theta_desired;
    output(:, i) = theta;
end
toc
t = 0:time_step:15;
plot(t ,output)
hold on
plot(t ,input)
grid on
title("PID Controller")
xlabel("t(s)")
ylabel("angle(degree)")
legend(["theta_1", "theta_2","theta_3","theta_4","theta_5","theta_6","input"])

%% Part 14, TJ and MTJ Controllers
I_m = [0.1,      0.05,   0.05,  0.025,  0.025, 0.01];
b_m = [0.0500, 0.0250, 0.0250, 0.0125, 0.0125, 0.0050];
eta = 1;

NUMBER_OF_POINTS = 1000;
T_F = 3;

thetas_o = [0, 0 , 0 , 0, 0, 0];
thetas_f = [0, 45, 45, 0, 0, 45];
[trajectories, trajectories_dot, trajectories_dotdot] = get_trajectories(thetas_o, thetas_f, NUMBER_OF_POINTS  , "cubic", T_F);
trajectories_time = linspace(0, T_F, NUMBER_OF_POINTS);

%tau = M_newton* M_coeffs + C_newton * C_Coeffs + B_newton * B_Coeffs + G_larange;
time_step = 0.1;
initial_conditions = transpose([thetas_o;  0, 0, 0, 0, 0, 0]);
last_conditions = transpose([thetas_o;  0, 0, 0, 0, 0, 0]);

Kp = diag([10000,  5000 ,  500,   50,  10, 0.5]);
Kv = diag([200,  145,  145,  50,  10, 0.01]);
%Kv = 0;
%Kv = 2 * sqrt(Kp);
T_end = 15;
input = zeros(6, length(0:time_step:T_end));
output = zeros(6, length(0:time_step:T_end));
error = zeros(6, 1);
sum_error = zeros(6, 1);
last_error_dot = zeros(6, 1);
last_imaginary_force = zeros(6, 1);
e_max = 0.005;
e_dot_max = 0.005;
i = 0;
tic
for t = 0:time_step:6
    t
    theta_desired = transpose(interp1(trajectories_time, trajectories, t,"linear", "extrap")) * pi / 180;
    theta_dot_desired = transpose(interp1(trajectories_time, trajectories_dot, t,"linear", "extrap"))* pi / 180;
    
    %X_desired = T2Xd(get_forward_kinematics(theta_desired));
    %X_current = T2Xd(get_forward_kinematics(initial_conditions(1:6, 1)));
    
    current_jacobian = get_numerical_jacobian(basic_jacobian_explicit, initial_conditions(1:6, 1));
    jacobian_desired = get_numerical_jacobian(basic_jacobian_explicit, theta_desired);
    X_dot_desired = jacobian_desired * theta_dot_desired;
    X_dot_current = current_jacobian * initial_conditions(1:6, 2);
    
    %error =     X_desired     -     X_current;
    error_dot = X_dot_desired - X_dot_current;
    error = error + (last_error_dot + error_dot) / 2 * time_step;
    err = transpose(initial_conditions(1:6, 1) - theta_desired) * 180 / pi
    %k = mean(exp(-(abs(error) / e_max + abs(error_dot) / e_dot_max)));
    k = 0;
    imaginary_force = Kp * error + Kv * error_dot + k * last_imaginary_force;
    input_tau_m = transpose(current_jacobian) * imaginary_force;
    %theta_dot_dot_current = (initial_conditions(1:6, 2) - last_conditions(1:6, 2)) / time_step;
    %input_tau = eta * (input_tau_m - transpose(b_m) .* initial_conditions(1:6, 2) * eta - transpose(I_m) .* theta_dot_dot_current *eta);
    %input_tau_m = [input_tau_m(1); input_tau_m(2);  input_tau_m(3); input_tau_m(4); -input_tau_m(5); input_tau_m(6)];
    [theta, theta_dot] = Robot_Simulator(input_tau_m, initial_conditions, time_step);
    last_conditions = initial_conditions;
    initial_conditions = [theta, theta_dot];
    last_imaginary_force = imaginary_force;
    last_error_dot = error_dot;
    i  = i + 1;
    input(:, i) = theta_desired;
    output(:, i) = theta;
end
toc
t = 0:time_step:15;
plot(t ,transpose(output(1:3, :)))
hold on
plot(t ,input)
grid on
title("MTJ Controller")
xlabel("t(s)")
ylabel("angle(degree)")
legend(["theta_1", "theta_2","theta_3","theta_4","theta_5","theta_6","input"])
legend(["theta_1", "theta_2","theta_3","input"])

%%
figure;
hold on;
grid on;
title("Controller Performances");
xlabel("t(s)");
ylabel("angle(degree)");
plot(t, input, 'LineWidth',2);
plot(t, PID, 'LineWidth',2);
plot(t, MTJ, 'LineWidth',2);
plot(t, TJ, 'LineWidth',2);
legend(["Input signal",  "PID CONTROLLER", "MTJ CONTROLLER", "TJ CONTROLLER"])

