function x_s = get_singular_points()
    A1_limits = [- 170 , 170];
    A2_limits = [- 80  , 140];
    A3_limits = [- 110 , 155];
    A4_limits = [- 175 , 175];
    A5_limits = [- 135 , 120];
    A6_limits = [- 350 , 350];
    
    A1 = A1_limits(1):1:A1_limits(2);   
    A2 = A2_limits(1):1:A2_limits(2);   %Lower Arm Rotation
    A3 = A3_limits(1):1:A3_limits(2);   %Upper Arm Rotation
    A4 = A4_limits(1):1:A4_limits(2);   %Arm Roll
    A5 = A5_limits(1):2:A5_limits(2);   %Wrist Bend
    A6 = A6_limits(1):1:A6_limits(2);   %Tool Flange
    
    A2 = A2 - 90;
    A3 = A3 - 90;
    
    A1 = [0];
    A4 = [0];
    A6 = [0];
    points = {};
    counter = 1;


    for i = 1:length(A1)
       for j = 1:length(A2)
            for k = 1:length(A3)
                for l = 1:length(A4)
                    for m = 1:length(A5)
                        for n = 1:length(A6)  
                            theta_values = [A1(i), A2(j), A3(k), A4(l), A5(m), A6(n)];
                            theta_1 = theta_values(1);
                            theta_2 = theta_values(2);
                            theta_3 = theta_values(3);
                            theta_4 = theta_values(4);
                            theta_5 = theta_values(5);
                            theta_6 = theta_values(6);
                            det = abs((13*sind(theta_5)*(13*cosd(theta_2) - 13*cosd(theta_3) + sind(theta_2) - sind(theta_3) - 169*cosd(theta_2)*cosd(theta_3) - 13*cosd(theta_2)*sind(theta_3) - 26*cosd(theta_2)*cosd(theta_3)^2 + 168*cosd(theta_3)^2*sind(theta_2) + 26*cosd(theta_3)*sind(theta_2)*sind(theta_3) + 168*cosd(theta_2)*cosd(theta_3)*sind(theta_3))));
                            if (det < 16)
                                points{end + 1} = theta_values;
                            end
                            counter = counter + 1;
                        end
                    end
                end 
            end
            counter
        end 
    end
    
    number_of_singularities = length(points);
    x_s = zeros(number_of_singularities , 6);
    for i = 1:number_of_singularities 
        x_s(i, :) = points{i};
    end
%     toc
%     
% end_effector_coordinates = zeros(length(x_s), 3);
% for i=1:length(points)
%     [T, L4] = get_forward_kinematics(points{i});
%     end_effector_coordinates(i, :) = T(1:3, 4);
% end
% plot(end_effector_coordinates(:, 1), end_effector_coordinates(:, 3),'.');
% axis equal;
% grid on;
% hold on;
% %scatter(end_cordinates(:, 1), end_cordinates(:, 3));    
end
