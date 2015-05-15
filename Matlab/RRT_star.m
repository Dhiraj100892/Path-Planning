
clear;
close all;

%% Initialization
map=im2bw(imread('map1.bmp')); % input map read from a bmp file. for new maps write the file name here
source = [200 50]; % source position in Y, X format
goal = [300 50]; % goal position in Y, X format

stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000000;
display=true; % display of RRT
iterations = 100;    %no of iterations
min_r = 50;

%% Increase obstacle size
map1 = map;
for i=1:size(map,1)    
    for j=1:size(map,2)
        if map(i,j)==0
            if i-1>=1, map1(i-1,j)=0; end
            if j-1>=1, map1(i,j-1)=0; end
            if i+1<=size(map1,1), map1(i+1,j)=0; end
            if j+1<=size(map1,2), map1(i,j+1)=0; end
            if i-1>=1 && j-1>=1, map1(i-1,j-1)=0; end
            if i-1>=1 && j+1<=size(map1,2), map1(i-1,j+1)=0; end
            if i+1<=size(map1,1) && j-1>=1, map1(i+1,j-1)=0; end
            if i+1<=size(map1,1) && j+1<=size(map1,2), map1(i+1,j+1)=0; end
        end
    end
end
if ~feas_Poi(source,map1), error('source lies on an obstacle or outside map'); end
if ~feas_Poi(goal,map1), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end

rectangle('Position',[source(1,2), source(1,1), 10, 10], 'Curvature',[1,1],'FaceColor','g');
rectangle('Position',[goal(1,2), goal(1,1), 10, 10], 'Curvature',[1,1],'FaceColor','r');

%% Path Finding
RRTree1=double([source -1 0]); % First RRT rooted at the source, representation node ,parent index and cost
RRTree2=double([goal -1 0]); % Second RRT rooted at the goal, representation node, parent index and cost
counter=0;
Path_C = [];          %sunobtimized path co-ordinates
Path_P = [];          % path properties length, startting row in Path_C & ending row in Path_C
post_processing = true ;  %post processing on path obatined from RRT*
Path_S = [];          % final smoothed version of optimized path
tree1ExpansionFail=false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false; % sets to true if expansion after set number of attempts fails
while  isempty(Path_P) || counter < iterations  % loop to grow RRTs
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrt_Star_Ext(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map1,min_r); % RRT 1 expands from source towards goal
    
    end
    
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        pathLength = 0;
        for i=1:length(path)-1
            pathLength=pathLength+dist_Cost(path(i,1:2),path(i+1,1:2));
        end
        Path_P = [Path_P ; pathLength size(Path_C,1)+1 size(Path_C,1)+size(path,1)];
        Path_C = [Path_C ; path];
%         break;
    end

    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrt_Star_Ext(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map1,min_r); % RRT 1 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        pathLength = 0;
        for i=1:length(path)-1
            pathLength=pathLength+dist_Cost(path(i,1:2),path(i+1,1:2));
        end
        Path_P = [Path_P ; pathLength size(Path_C,1)+1 size(Path_C,1)+size(path,1)];
        Path_C = [Path_C ; path];
%         break;
    end
    if  display
            imshow(map);
            rectangle('Position',[source(1,2)-5, source(1,1)-5, 10, 10], 'Curvature',[1,1],'FaceColor','g');
            rectangle('Position',[goal(1,2)-5, goal(1,1)-5, 10, 10], 'Curvature',[1,1],'FaceColor','r');

            for i = 0:size(RRTree1,1)-2
                line([RRTree1(end-i,2);RRTree1(RRTree1(end-i,3),2)],[RRTree1(end-i,1);RRTree1(RRTree1(end-i,3),1)],'color','b');    
            end
            for i = 0:size(RRTree2,1)-2
                line([RRTree2(end-i,2);RRTree2(RRTree2(end-i,3),2)],[RRTree2(end-i,1);RRTree2(RRTree2(end-i,3),1)],'color','r');    
            end
            if ~isempty(Path_P)
                [min_D ,min_I] = min(Path_P(:,1)); 
                 line(Path_C(Path_P(min_I,2):Path_P(min_I,3),2),Path_C(Path_P(min_I,2):Path_P(min_I,3),1),'LineWidth',2,'color','k');   
            end
            counter=counter+1;M(counter)=getframe;
            
    end
end

if size(Path_P,1)<=0, error('no path found. maximum attempts reached'); end

fprintf('Path Length=%d \n\n', Path_P(min_I,1)); 
imshow(map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(Path_C(Path_P(min_I,2):Path_P(min_I,3),2),Path_C(Path_P(min_I,2):Path_P(min_I,3),1),'LineWidth',1,'color','b');   
hold on;

%% Smooth The path
[Path_S ]= Path_Smooth(Path_C(Path_P(min_I,2):Path_P(min_I,3),:),map1);
plot(Path_S(:,2),Path_S(:,1),'LineWidth',2,'color','r')
legend('RRT*Path','Post Processed Path');
rectangle('Position',[source(1,2)-5, source(1,1)-5, 10, 10], 'Curvature',[1,1],'FaceColor','g');
rectangle('Position',[goal(1,2)-5, goal(1,1)-5, 10, 10], 'Curvature',[1,1],'FaceColor','r');
for i = 1:50
    M(counter+1)= getframe; 
    counter = counter+1;
end
