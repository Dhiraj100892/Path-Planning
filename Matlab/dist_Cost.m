
function h=dist_Cost(a,b)
% Find the distance between two points
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 );
