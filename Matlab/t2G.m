function G = t2G(t)
    G = sym('GG',[6 ; 1]);
    syms g;
    for i=1:6
        temp = coeffs(t(i), g);
        if length(temp) > 1
            G(i) = temp (2);
        else
            G(i) = 0;
        end
    end
end
