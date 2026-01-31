function [out] = table_lookup(datin,datout,in)

%This sub-module is used to do a simple 1-dimensional table look up
out=0.;
if (in<=datin(1))
    out=datout(1);
end
done=0;
k=1;
    
while(~done)
    k=k+1;
    if (in<=datin(k))
        out=datout(k-1)+(datout(k)-datout(k-1))/(datin(k)-datin(k-1))*(in-datin(k-1));
        done=1;
    elseif (k==length(datin))
        out=datout(k);
        done=1;
    end
end

return;