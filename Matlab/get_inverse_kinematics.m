
function [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, is_successful] = get_inverse_kinematics(P, R)
    theta_1 = zeros(1, 1);
    theta_2 = zeros(1, 1);
    theta_3 = zeros(1, 1);
    theta_4 = zeros(1, 1);
    theta_5 = zeros(1, 1);
    theta_6 = zeros(1, 1);
    
    % link_1 
    alpha_0 = 0;
    a_0 = 0;
    d_1 = 0;
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

    syms x;
    c_3 = (1 - x^2) / (1 + x^2);
    s_3 = (2 * x)   / (1 + x^2);
    
    f_1 = a_3 * c_3 + d_4*sin(alpha_3) * s_3 + a_2;
    f_2 = a_3 * cos(alpha_2)*s_3 - d_4*sin(alpha_3)*cos(alpha_2)*c_3 - d_4*sin(alpha_2)*cos(alpha_3) - d_3*sin(alpha_2);
    f_3 = a_3 * sin(alpha_2)*s_3 - d_4*sin(alpha_3)*sin(alpha_2)*c_3 + d_4*cos(alpha_2)*cos(alpha_3) + d_3*cos(alpha_2);

    k_1 = f_1;
    k_2 = -f_2;
    k_3 = f_1^2 + f_2^2 + f_3^2 + a_1^2 + d_2^2 + 2*d_2*f_3;
    k_4 = f_3 * cos(alpha_1) + d_2 * cos(alpha_1);

    r0 = P(1)^2 + P(2)^2 + P(3)^2;
    
    eqn1 = ((r0-k_3)^2)*(sin(alpha_1))^2 + ((P(3)-k_4)^2)* 4 * a_1^2 - (4*a_1^2)*((sin(alpha_1))^2)*((k_1^2)+(k_2^2)) == 0;
    
    solved_x_values = vpa(solve(eqn1, x));
    number_of_solutions = length(solved_x_values) * 4;
    if number_of_solutions == 0
        is_successful = 0;
        return;
    end
    
    theta_1 = zeros(number_of_solutions, 1);
    theta_2 = zeros(number_of_solutions, 1);
    theta_3 = zeros(number_of_solutions, 1);
    theta_4 = zeros(number_of_solutions, 1);
    theta_5 = zeros(number_of_solutions, 1);
    theta_6 = zeros(number_of_solutions, 1);
    
    for i = 1:number_of_solutions
        solved_x_value = solved_x_values(mod(i, 4) + 1);
        theta_3(i) = 2 * atan(solved_x_value);
        
        f_1_temp = vpa(subs(f_1, sym('x'), solved_x_value));
        f_2_temp = vpa(subs(f_2, sym('x'), solved_x_value));
        f_3_temp = vpa(subs(f_3, sym('x'), solved_x_value));

        k_1_temp  = vpa(subs(k_1, sym('x'), solved_x_value));
        k_2_temp  = vpa(subs(k_2, sym('x'), solved_x_value));
        k_3_temp  = vpa(subs(k_3, sym('x'), solved_x_value));
        k_4_temp  = vpa(subs(k_4, sym('x'), solved_x_value));


        theta_2(i) = atan2( (r0-k_3_temp )/ (2*a_1) , (k_4_temp -P(3))/(sin(alpha_1))) - atan2(k_1_temp  ,k_2_temp );

        g1 = cos(theta_2(i))*f_1_temp  - sin(theta_2(i))*f_2_temp  + a_1;
        g2 = sin(theta_2(i))*cos(alpha_1)*f_1_temp  + cos(theta_2(i))*cos(alpha_1)*f_2_temp  - sin(alpha_1)*f_3_temp  - d_2*sin(alpha_1);

        theta_1(i) = vpa(atan2(P(2),P(1)) - atan2(real(g2), real(g1)));

        T1 = [cos((theta_1(i))), -sin((theta_1(i))), 0, a_0  ; cos((alpha_0 ))*sin((theta_1(i))), cos((alpha_0 ))*cos((theta_1(i))), -sin((alpha_0 )), -d_1*sin((alpha_0 )) ;sin((alpha_0 ))*sin((theta_1(i))), sin((alpha_0))*cos((theta_1(i))),  cos((alpha_0)),  d_1*cos((alpha_0)) ;0,0,0,1];
        T2 = [cos((theta_2(i))), -sin((theta_2(i))), 0, a_1  ; cos((alpha_1 ))*sin((theta_2(i))), cos((alpha_1 ))*cos((theta_2(i))), -sin((alpha_1 )), -d_2*sin((alpha_1 )) ;sin((alpha_1 ))*sin((theta_2(i))), sin((alpha_1))*cos((theta_2(i))),  cos((alpha_1)),  d_2*cos((alpha_1)) ;0,0,0,1];
        T3 = [cos((theta_3(i))), -sin((theta_3(i))), 0, a_2  ; cos((alpha_2 ))*sin((theta_3(i))), cos((alpha_2 ))*cos((theta_3(i))), -sin((alpha_2 )), -d_3*sin((alpha_2 )) ;sin((alpha_2 ))*sin((theta_3(i))), sin((alpha_2))*cos((theta_3(i))),  cos((alpha_2)),  d_3*cos((alpha_2)) ;0,0,0,1];
        T4 = [cos((0)), -sin((0)), 0, a_3  ; cos((alpha_3 ))*sin((0)), cos((alpha_3 ))*cos((0)), -sin((alpha_3 )), -d_4*sin((alpha_3 )) ;sin((alpha_3 ))*sin((0)), sin((alpha_3))*cos((0)),  cos((alpha_3)),  d_4*cos((alpha_3)) ;0,0,0,1];        
        
        L4 = T1*T2*T3*T4;
        R0_4 = L4(1:3,1:3);

        R46 = transpose(R0_4) * R; %Inverse of R0_4 multiplied by R0
        
%         gamma = atan2(-R46(2,3),-R46(1,3));
%         alpha = atan2(-R46(3,2),R46(3,1));
%         beta =  atan2(-R46(3,2),(R46(3,3)*sin(alpha)));        
%         
        gamma = atan2(-R46(2,3),-R46(1,3));
        alpha = atan2(-R46(3,2),R46(3,1));
        beta =  atan2(sqrt((R46(1,3))^2+(R46(2,3))^2),R46(3,3));  
        
                
        theta_1(i) = vpa(theta_1(i)) + 2 * pi; 
        theta_2(i) = vpa(theta_2(i))+ 2 * pi;
        theta_3(i) = vpa(theta_3(i));
        theta_6(i) = vpa(alpha) + pi * (i > 8);
        theta_5(i) = vpa(beta) * (-1)^(i > 8);
        theta_4(i) = vpa(gamma) + pi * (i > 8);
    end
    
    is_successful = 1;
end

