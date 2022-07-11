function m = f_skew(vec)
if size(vec,1)==3 && size(vec,2)==1
    m = [0,-vec(3,1),vec(2,1);
        vec(3,1),0,-vec(1,1);
        -vec(2,1),vec(1,1),0];
elseif size(vec,1)==1&&size(vec,2)==3
    m = [0,-vec(1,3),vec(1,2);
        vec(1,3),0,-vec(1,1);
        -vec(1,2),vec(1,1),0];
elseif size(vec,1)==1&&size(vec,2)==1
    m=[0,-v;
        v,0];
else
    vec
    error("input vec must be a 3-1 or 1-1 vector");
end
end