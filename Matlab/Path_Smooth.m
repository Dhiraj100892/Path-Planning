function [ P ] = Path_Smooth( path, map )
% Smooth out the path
Path_S = path;

%% past processing algorithm on the RRT* path points
for i = 1: size(Path_S,1)-1    
    m = size(Path_S,1);
    if( i > m-1) 
        break;
    end
    k = i+1;
    
    for j = i+2:m
        if checkPath(Path_S(i,:),Path_S(j,:),map)
            k = j;
        end
    end
    Path_S = [Path_S(1:i,:);Path_S(k:end,:)];
end
m = size(Path_S,1);
P = Path_S(1,:);

%% For smoothing out the sharp corners
for i = 1:m-2          
    d1 = distanceCost(Path_S(i,:),Path_S(i+1,:));
    d2 = distanceCost(Path_S(i+2,:),Path_S(i+1,:));
    stepsize = 1;
    points = [Path_S(i+1,:); Path_S(i+1,:)];
    s = true; 
    theta1=atan2(Path_S(i,1)-Path_S(i+1,1),Path_S(i,2)-Path_S(i+1,2));  % direction to extend sample to produce new node
    theta2=atan2(Path_S(i+2,1)-Path_S(i+1,1),Path_S(i+2,2)-Path_S(i+1,2));  % direction to extend sample to produce new node
    while s           %choose two points on line such that joining line be these points just hit the obstacle
        newPoint1 = double(int32(Path_S(i+1,:) + stepsize * [sin(theta1)  cos(theta1)]));
        newPoint2 = double(int32(Path_S(i+1,:) + stepsize * [sin(theta2)  cos(theta2)]));   
        if distanceCost(newPoint1,Path_S(i+1,:))>d1/2 || distanceCost(newPoint2,Path_S(i+1,:))>d2/2
            break;
        end
        if checkPath(newPoint1,newPoint2,map)
            points = [newPoint1;newPoint2];
            stepsize = stepsize+1;
        else
            s = false;
        end
    end
    t = 0: 0.1: 1;                      %Quadratic Bezier curve "http://en.wikipedia.org/wiki/B%C3%A9zier_curve"
    x = (1-t).*( (1-t)*points(1,2) + t*Path_S(i+1,2) ) + t.*( (1-t)*Path_S(i+1,2) + t*points(2,2) );
    y = (1-t).*( (1-t)*points(1,1) + t*Path_S(i+1,1) ) + t.*( (1-t)*Path_S(i+1,1) + t*points(2,1) );
    plot(x,y,'LineWidth',2,'color','r')
    P  = [P; [y' x']];
            
        
end
    P = [ P ; Path_S(end,:)];
end

% end
