function J = get_numerical_jacobian(J, thetas)
    J = subs(J, sym("theta_1"), thetas(1));
    J = subs(J, sym("theta_2"), thetas(2));
    J = subs(J, sym("theta_3"), thetas(3));
    J = subs(J, sym("theta_4"), thetas(4));
    J = subs(J, sym("theta_5"), thetas(5));
    J = vpa(subs(J, sym("theta_6"), thetas(6)));
end
