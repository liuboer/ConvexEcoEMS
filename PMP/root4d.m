function F = root4d(x,v_0,v_T,s_0,s_T,v_max,T)
F(1) = x(1)*x(3)+x(2);
F(2) = x(1)/2*x(3)^2+x(2)*x(3)+v_0-v_max;
F(3) = x(1)/2*(T-x(4))^2+v_max-v_T;
F(4) = x(1)/6*x(3)^3+x(1)/6*(T-x(4))^3+x(2)/2*x(3)^2+v_0*x(3)+v_max*(T-x(3))+s_0-s_T;
end