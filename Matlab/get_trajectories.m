function [trajectories, trajectories_dot, trajectories_dotdot] = get_trajectories(thetas_o, thetas_f, number_of_points, type, t_f, acceleration)
    number_of_angles = length(thetas_o);
    trajectories = zeros(number_of_points, number_of_angles);
    trajectories_dot = zeros(number_of_points, number_of_angles);
    trajectories_dotdot = zeros(number_of_points, number_of_angles);
    time_step = t_f / number_of_points;
    
    if type == "cubic"
        for i= 1:number_of_angles
            a_0 = thetas_o(i);
            a_1 = 0;
            a_2 = 3 / (t_f^2) * (thetas_f(i) - thetas_o(i));
            a_3 = -2 / (t_f ^ 3) * (thetas_f(i) - thetas_o(i));
            
            t = linspace(0, t_f , number_of_points);
            trajectories(: , i) = a_0 + a_1 * t + a_2 * t.^2 + a_3 * t.^3;
            trajectories_dot(1:end-1, i) = diff(trajectories(: , i)) / time_step; trajectories_dot(end, i) = trajectories_dot(end - 1, i);
            trajectories_dotdot(1:end-1, i) = diff(trajectories_dot(: , i)) / time_step; trajectories_dotdot(end, i) = trajectories_dotdot(end - 1, i);
        end
        return;
    elseif type == "parabolic_blends"
        for i= 1:number_of_angles
            if thetas_f(i) == thetas_o(i)
                trajectories(: , i) = thetas_f(i); %or  zeros(number_of_points, 1) + thetas_f(i)
                continue
            end
            sign_value = sign(thetas_f(i) - thetas_o(i));
            delta_term = acceleration(i) ^2 * t_f^2 - 4 * sign_value * acceleration(i) * (thetas_f(i) - thetas_o(i));
            if delta_term < 0
                fprintf("Low Acceleration Value for Angle %d, It's not possible", i);
                return;
            end
            t_b =  t_f / 2 - abs(sqrt(delta_term) / (2 * acceleration(i)));
            
            t = linspace(0, t_f , number_of_points);
            
            part_1 = t(t < t_b);
            trajectories(1:length(part_1), i) = sign(thetas_f(i) - thetas_o(i)) * 0.5 * acceleration(i) * part_1.^2 + thetas_o(i);
            
            part_2 = t(t >= t_b & t < t_f - t_b);
            t_m = t_f - 2 * t_b;
            delta = (((thetas_o(i) + thetas_f(i)) / 2) - trajectories(length(part_1) , i) ) * 2;
            v_m = delta / t_m;
            trajectories(length(part_1) + 1:length(part_1) + length(part_2), i) = trajectories(length(part_1) , i) + v_m * (part_2 - t(length(part_1))) ; 
            
            part_3 = t(t >= t_f - t_b);
            trajectories(length(part_1) + length(part_2) + 1:length(part_1) + length(part_2) + length(part_3), i) = transpose(fliplr(thetas_o(i)-transpose(trajectories(1:length(part_1), i))))+ thetas_f(i);
            trajectories_dot(1:end-1, i) = diff(trajectories(: , i)) / time_step; trajectories_dot(end, i) = trajectories_dot(end - 1, i);
            trajectories_dotdot(1:end-1, i) = diff(trajectories_dot(: , i)) / time_step; trajectories_dotdot(end, i) = trajectories_dotdot(end - 1, i);
        end
        return;        
    else 
        disp("Method Not Implemented");
        return;
    end
end
