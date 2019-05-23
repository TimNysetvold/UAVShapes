function [currentobjective,patharray] = getObjective(drone,colavoidvector,oldpatharray,baselength,layerMat)
%   Drone will select an appropriate objective from the matrix of objects.
%   Objectives can be pizza deliveries, finding fires, avoiding other drones,
%   etc. Collision avoidance takes place before here.

% Objectives are listed in the patharray as a series of points that the drone must reach,
% indexed so that the first objective in the list is the one that the drone 
% is currently travelling towards, the second is the next
% objective, etc. We also return the current objective for ease of
% packaging in the main function.

% Objectives have the form [x,y,z, +-1, where +-1 determines 
% if the UAV goes towards or away from the objective].

% Generally, objectives should be created so that you proceed to the
% correct location at your cruising altitude and then descend vertically.

%All of the information we will need is unpacked from the drone array and
%given its own names. This helps if/when the drone array is refactored.

% As a reminder, each drone has is a 21-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

global objectivesreached;
dronePosition=drone(1:3);
stamina=drone(13);
loiter=drone(18);
type=drone(14);
base=[drone(9),drone(10),0,1];
droneCruiseHeight=drone(12);


if(stamina<1000)
    %This loop makes a drone return to base if its stamina is too low.
    %Since it affects the objective of a drone, it belongs here.
    patharray=base;
    %All other previous objectives are eliminated, and we do not 
    %pay attention to routing requirements. We're only concerned 
    %about getting back to base, as fast as possible.
end

disttofinalobj=norm(dronePosition-oldpatharray(end-3:end-1));
disttointermedobj=norm(dronePosition-oldpatharray(1:3));

if(disttofinalobj<.015) %If the distance to the final objective is less 
    %than 15m, we consider it has reached the objective.
  
    % If the drone is at its final objective, it needs a new objective. It
    % gets one using this code. Its behavior depends on the drone type.
    
    % Note that three objectives will be created; one that involves
    % ascending to the cruising altitude vertically; one that involves the
    % horizontal travel, and one that involves descending to the goal.
    
    if (type==1)
    %if drone is a police drone, follows these
        if (oldpatharray(end-3:end)==base)
            %if it's at base, get a new random objective and refuel
            patharray=[rand()*baselength,rand()*baselength,droneCruiseHeight+.5*rand()-.25,1];
            stamina=10000;
        else
            %if it is anywhere else, go to a new objective.
            patharray=[rand()*baselength,rand()*baselength,droneCruiseHeight,1];
        end
        objectivesreached=objectivesreached+1;
        
    elseif (type==2)
    %if drone is an AmazonAir drone, it follows these statements
        if (oldpatharray(end-3:end)==base)
            stamina=16000;
            finalobjective=[rand()*baselength,rand()*baselength,0,1];
            droneCruiseHeight=AltDetLayers([finalobjective(1:2),dronePosition(3)],dronePosition,layerMat);
            midobjective=[finalobjective(1:2),droneCruiseHeight,1]; %dronecruiseheight is now a 
            riseobjective=[base(1:2),droneCruiseHeight,1;]; 
            patharray=[riseobjective,midobjective,finalobjective];
        else
            droneCruiseHeight=AltDetLayers([base(1:2),dronePosition(3)],dronePosition,layerMat);
            midobjective=[dronePosition(1:2),droneCruiseHeight,1];
            riseobjective=[base(1:2),droneCruiseHeight,1];
            patharray=[midobjective,riseobjective,base];
        end
        objectivesreached=objectivesreached+1;
        
        %debug
        cautionarray=patharray==.3;
        if any(cautionarray)
            disp 'careful'
        end

    elseif (type==3)
    %if the drone is a hobbyist, follow these. Creates random motion within a
    % distance of .5 of the base.
        if (pdist([drone(1),drone(2);oldpatharray])<.01)
            patharray=[(rand()-.5)+base(1),(rand()-.5)+base(2),(rand()-.5)+base(3),1];
            objectivesreached=objectivesreached+1;
        end        
    elseif (type==4)
    %if drone is a military drone, follows these.
        %if it's at base, get a new random objective and
        %fuel. This code is suspect; check it before using.
        if (oldpatharray==base)
            patharray=[rand()*baselength*10,rand()*baselength*10,1];
            stamina=100000;
            loiter=0;
        elseif loiter<1800
            %if it is at another objective, it will loiter there for half an hour.
            patharray=[patharray(1)+rand()-.5,patharray(2)+rand()-.5,1];
            loiter=loiter+1;
        elseif loiter>1800
            %if it has been loitering for a half hour, it will select a
            %new objective.
            patharray=[rand()*baselength*10,rand()*baselength*10,droneCruiseHeight,1];
            loiter=0;
        end
        objectivesreached=objectivesreached+1;
    end     
    
    currentobjective=patharray(1:4);
    
elseif(disttointermedobj<.015)
    %After checking to make sure we are not at our current final objective,
    %we check if the drone has any intermediate objectives. (If we are at
    %our current final objective, no intermediate objectives matter and we
    %follow the code above).
    
    % If we are close enough to the xyz coordinates of one of the intermediate
    % objectives, we eliminate that objective and move on to the next.

    % We pop the top objective off and
    % make sure the rest is recorded in the array we will
    % send to the outside for remembering.
    currentobjective=oldpatharray(1:4);
    patharray=oldpatharray(5:end);
elseif(any(colavoidvector))
    %If there is an avoidance vector from some collision avoidance
    %mechanism, we follow it.
    currentobjective=colavoidvector;
    patharray=oldpatharray;
else
    %If none of the above are true (not within small distance of an objective, 
    %and not in conflict) we proceed on the path we were already on.
    currentobjective=oldpatharray(1:4);
    patharray=oldpatharray;
end


end

%Debug code
% if length(patharray)<4
%     disp 'oh gnoez'
% end


