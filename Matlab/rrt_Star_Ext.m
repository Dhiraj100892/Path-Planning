
function [RRTree1,pathFound,extendFail]=rrt_Star_Ext(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map, min_r)
% extends the tree
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
check = -1*ones(size(RRTree1,1));

while failedAttempts<=maxFailedAttempts
    
    %% goal biased sampling
    if rand < 0.5, 
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    
    %% Find the nearest node to the sampled one
    [A, I]=min( dist_Cost(RRTree1(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree1(I(1),:);
    theta=atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~che_Path(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end
    
    %% Assign the parent
    D = dist_Cost(RRTree1(:,1:2),newPoint);
    cost_C_N = closestNode(1,4) + D(I(1),:);
    [A, I2]=min( dist_Cost(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if A<disTh, % if both trees are connected
        pathFound=[newPoint I(1) I2(1)];extendFail=false;break; 
    end 
    [A, I3]=min( D ,[],1); % check if new node is not already pre-existing in the tree
    if A <disTh, failedAttempts=failedAttempts+1;continue; end 
    for i = 1 : size(RRTree1,1)
        if D(i,1) < min_r
            if RRTree1(i,4) + D(i,1) < cost_C_N
                check(i) = che_Path(RRTree1(i,1:2),newPoint, map);
                if check(i) == 1
                    cost_C_N = RRTree1(i,4) + D(i,1);
                    I(1) = i;
                end            
            end 
        end
    end
    
    %% Change the parents
    for i = 2: size(RRTree1,1)
        if D(i) < min_r
            if cost_C_N + D(i) < RRTree1(i,4)
                if check(i) == -1
                    check(i) = che_Path(RRTree1(i,1:2),newPoint, map);
                end
                if check(i) == 1
                    RRTree1(i,4) = cost_C_N + D(i);
                    RRTree1(i,3) = size(RRTree1,1) + 1;
                end
            end
        end
    end
    
    RRTree1=[RRTree1;newPoint I(1) cost_C_N];extendFail=false;break; % add node
end
