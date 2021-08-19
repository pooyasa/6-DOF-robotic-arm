function [theta, theta_dot] = Robot_Simulator(input_tau, initial_condition, timestep)
% 
%       input_tau = [500; 50 ;50; 50; 50; -1];     
%       initial_condition = [45 0; 0 0; 0 0;  0 0; 0 0 ; 0 0];
%       timestep = 0.2;
    
    syms t theta_1(t) theta_2(t) theta_3(t) theta_4(t) theta_5(t) theta_6(t)
    thetas = {theta_1 theta_2 theta_3 theta_4 theta_5 theta_6};
  
    tahu = round(transpose(input_tau ))
    persistent tau;
    if length(tau) < 6
        vars = load("tau.mat");
        tau = vars.tau;        
    end

    eqn = tau == input_tau;
    [newEqs, newVars] = reduceDifferentialOrder(eqn, thetas);
    [DM,DF] = massMatrixForm(newEqs, newVars);
    MM = odeFunction(DM, newVars);
    FF = odeFunction(DF, newVars);
    opt = odeset('Mass', MM, 'InitialSlope', [0.005;0]);

    [t, y] = ode45(FF, [0,1], initial_condition, opt);  

    theta_values = y(:, 1:6);
    theta_dot_values = y(:, 7:12);

    theta = transpose(interp1(t, theta_values(:, :), timestep));
    theta_dot = transpose(interp1(t, theta_dot_values(:, :), timestep));
end



%     theta_dot_1 = diff(theta_1, t);
%     theta_dotdot_1 = diff(theta_dot_1, t);
%     theta_dot_2 = diff(theta_2, t);
%     theta_dotdot_2 = diff(theta_dot_2, t);
%     theta_dot_3 = diff(theta_3, t);
%     theta_dotdot_3 = diff(theta_dot_3, t);
%     theta_dot_4 = diff(theta_4, t);
%     theta_dotdot_4 = diff(theta_dot_4, t);
%     theta_dot_5 = diff(theta_5, t);
%     theta_dotdot_5 = diff(theta_dot_5, t);
%     theta_dot_6 = diff(theta_6, t);
%     theta_dotdot_6 = diff(theta_dot_6, t);
% 
%     thetas = {theta_1 theta_2 theta_3 theta_4 theta_5 theta_6};
%     theta_dots = {theta_dot_1 theta_dot_2 theta_dot_3 theta_dot_4 theta_dot_5 theta_dot_6};
%     theta_dotdots = {theta_dotdot_1 theta_dotdot_2 theta_dotdot_3 theta_dotdot_4 theta_dotdot_5 theta_dotdot_6};
%     

%     tau = M_lagrange * M_coeffs + C_lagrange * C_Coeffs + B_lagrange * B_Coeffs + G_larange;
%     
%     tau = subs(tau, sym("theta_1"), theta_1);
%     tau = subs(tau, sym("theta_2"), theta_2);
%     tau = subs(tau, sym("theta_3"), theta_3);
%     tau = subs(tau, sym("theta_4"), theta_4);
%     tau = subs(tau, sym("theta_5"), theta_5);
%     tau = subs(tau, sym("theta_6"), theta_6);
% 
%     tau = subs(tau, sym("theta_dot_1"), theta_dot_1);
%     tau = subs(tau, sym("theta_dot_2"), theta_dot_2);
%     tau = subs(tau, sym("theta_dot_3"), theta_dot_3);
%     tau = subs(tau, sym("theta_dot_4"), theta_dot_4);
%     tau = subs(tau, sym("theta_dot_5"), theta_dot_5);
%     tau = subs(tau, sym("theta_dot_6"), theta_dot_6);
% 
%     tau = subs(tau, sym("theta_dotdot_1"), theta_dotdot_1);
%     tau = subs(tau, sym("theta_dotdot_2"), theta_dotdot_2);
%     tau = subs(tau, sym("theta_dotdot_3"), theta_dotdot_3);
%     tau = subs(tau, sym("theta_dotdot_4"), theta_dotdot_4);
%     tau = subs(tau, sym("theta_dotdot_5"), theta_dotdot_5);
%     tau = subs(tau, sym("theta_dotdot_6"), theta_dotdot_6);
% 


%     stringVars = string(newVars);
%     figure;
%     subplot(2, 1,1);
%     plot(t, theta_values);
%     title("Thetas");
%     legend(stringVars(1:6));
%     subplot(2, 1,2);
%     plot(t, theta_dot_values);
%     title("Omegas");
%     legend(stringVars(7:12));    
