function [a,v,s,x,error_flag] = PMP(v_0,v_T,s_0,s_T,v_max,T,eqn_order,a_0,a_T,v_is_constraint)
error_flag = 0;

t = 0:T;

if eqn_order == 3
A_eqn = [0          0        0        1       0       0;
         T.^3/6     T.^2/2   T        1       0       0;
         0          0        0        0       1       0;
         T.^4/24    T.^3/6   T.^2/2   T       1       0;
         0          0        0        0       0       1;
         T.^5/120   T.^4/24  T.^3/6   T.^2/2  T       1];
b_eqn = -[a_0; a_T; v_0; v_T; s_0; s_T];
x = A_eqn \ b_eqn;

j = -(x(1)*t.^2/2+x(2)*t+x(3));
a = -(x(1)*t.^3/6+x(2)*t.^2/2+x(3)*t+x(4));
v = -(x(1)*t.^4/24+x(2)*t.^3/6+x(3)*t.^2/2+x(4)*t+x(5));
s = -(x(1)*t.^5/120+x(2)*t.^4/24+x(3)*t.^3/6+x(4)*t.^2/2+x(5)*t+x(6));

elseif eqn_order == 2

A_eqn = [0        0       1       0;
         T.^2/2   T       1       0;
         0        0       0       1;
         T.^3/6   T.^2/2  T       1];
b_eqn = -[v_0; v_T; s_0; s_T];
x = A_eqn \ b_eqn;

a = -(x(1)*t+x(2));
v = -(x(1)*t.^2/2+x(2)*t+x(3));
s = -(x(1)*t.^3/6+x(2)*t.^2/2+x(3)*t+x(4));

if max(v(2:end-1)) > v_max && v_is_constraint

x = fsolve(@(x) root4d(x,v_0,v_T,s_0,s_T,v_max,T),[0,0,0,0]);


if x(3) < 0 || x(4) < 0 || x(3) > x(4) || x(3) > T || x(4) > T
    error_flag = 1;
    x(3) = min(max(x(3),0),T);
    x(4) = min(max(x(3),x(4)),T);
end
t_stage_1 = 0:x(3);
t_stage_2 = x(3):x(4);
t_stage_3 = x(4):T;

a_stage_1 = x(1)*t_stage_1+x(2);
a_stage_2 = 0.*ones(1,length(t_stage_2));
a_stage_3 = x(1)*(t_stage_3-x(4));

v_stage_1 = v_0+x(1)/2*t_stage_1.^2+x(2)*t_stage_1;
v_stage_2 = v_max.*ones(1,length(t_stage_2));
v_stage_3 = v_max+x(1)/2*(t_stage_3-x(4)).^2;

a = [a_stage_1,a_stage_2,a_stage_3];
v = [v_stage_1,v_stage_2,v_stage_3];
s = cumsum(v);

end

end

end