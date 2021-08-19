function coef = get_coeffs(V, x)
    coef = [0; 0; 0];
    for i=1:3
        test = coeffs(V(i),x);
        if length(test) > 1
            coef(i) = test(2);
        else
            coef(i) = 0;
        end
    end
end
