function d_Z = fn_state(t,Z)

%Z(1)=x
%Z(2)=v(t)
d_Z=[Z(2); 2*(Z(1)-1)*Z(2)];