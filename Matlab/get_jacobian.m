function J = get_jacobian(P, Z)
    syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 
    J = sym('a',6);
    J(1:3, 1) = [diff(P(1), theta_1); diff(P(2), theta_1); diff(P(3), theta_1);];
    J(4:6, 1) = Z(:,1);

    J(1:3, 2) = [diff(P(1), theta_2); diff(P(2), theta_2); diff(P(3), theta_2);];
    J(4:6, 2) = Z(:,2);

    J(1:3, 3) = [diff(P(1), theta_3); diff(P(2), theta_3); diff(P(3), theta_3);];
    J(4:6, 3) = Z(:,3);

    J(1:3, 4) = [diff(P(1), theta_4); diff(P(2), theta_4); diff(P(3), theta_4);];
    J(4:6, 4) = Z(:,4);

    J(1:3, 5) = [diff(P(1), theta_5); diff(P(2), theta_5); diff(P(3), theta_5);];
    J(4:6, 5) = Z(:,5);

    J(1:3, 6) = [diff(P(1), theta_6); diff(P(2), theta_6); diff(P(3), theta_6);];
    J(4:6, 6) = Z(:,6);
end
