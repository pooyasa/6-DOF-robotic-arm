function [C, B] = M2V(M)
    syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 
        
    C = sym('a',6);
    for i=1:6
        for j=1:6
            C(i,j) = getB(M, i, j, j);
        end
    end
    B = sym('b', [6, 15]);
    for i=1:6
       B(i, 1) = 2 * getB(M, i, 1, 2);
       B(i, 2) = 2 * getB(M, i, 1, 3);
       B(i, 3) = 2 * getB(M, i, 1, 4);
       B(i, 4) = 2 * getB(M, i, 1, 5);
       B(i, 5) = 2 * getB(M, i, 1, 6);
       
       B(i, 6) = 2 * getB(M, i, 2, 3);
       B(i, 7) = 2 * getB(M, i, 2, 4);
       B(i, 8) = 2 * getB(M, i, 2, 5);
       B(i, 9) = 2 * getB(M, i, 2, 6);
       
       B(i, 10) = 2 * getB(M, i, 3, 4);
       B(i, 11) = 2 * getB(M, i, 3, 5);
       B(i, 12) = 2 * getB(M, i, 3, 6);
       
       B(i, 13) = 2 * getB(M, i, 4, 5);
       B(i, 14) = 2 * getB(M, i, 4, 6);
       
       B(i, 15) = 2 * getB(M, i, 5, 6);       
    end
    
end

function b = getB(M, i, j, k)  
    syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6;
    syms thetas;
    thetas = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6];
    b = 0.5 * (diff(M(i,j), thetas(k)) + diff(M(i,k), thetas(j)) - diff(M(j,k), thetas(i)));
end

