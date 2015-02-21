function feasible=che_Path(n,newPos,map)
% Checks if the Path is feasible
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
if ~feas_Poi(newPos,map), feasible=false;
else
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feas_Poi(ceil(posCheck),map) && feas_Poi(floor(posCheck),map) && ... 
            feas_Poi([ceil(posCheck(1)) floor(posCheck(2))],map) && feas_Poi([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
end
end
