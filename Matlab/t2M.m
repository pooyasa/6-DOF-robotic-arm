function M= t2M(t)
    syms theta_dotdot_1 theta_dotdot_2 theta_dotdot_3 theta_dotdot_4 theta_dotdot_5 theta_dotdot_6
    syms theta_dotdots;
    
    theta_dotdots = [theta_dotdot_1 theta_dotdot_2 theta_dotdot_3 theta_dotdot_4 theta_dotdot_5 theta_dotdot_6];
    
    M = sym('m',6);
    
    for i=1:6
        for j = 1:6
            test = coeffs(t(i), theta_dotdots(j));
            if length(test) > 1
                M(i, j) = test(2);
            else
                M(i, j) = 0;
            end
        end
    end
end
