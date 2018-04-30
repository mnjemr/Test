% Angular velocity feedback law
function u=getMC(kw,b,w,mmax)
    u=-kw/(norm(b)^2)*cross(b,w);
    for j=1:length(u)
        if abs(u(j))>mmax
            u(j)=sign(u(j))*mmax;
        end
    end
end