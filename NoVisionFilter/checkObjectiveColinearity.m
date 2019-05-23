function [objective,exitflag] = checkObjectiveColinearity(drone,finalobjective,avoidanceobjective)
    %If we are avoiding an intermediate, and that intermediate is
    %colinear with the drone's current flight path, we turn to the
    %right to avoid it. The angle necessary to make this happen may depend
    %on the method, so we parameterize that and make this a function.
    theta=30;
    
    %%Trying this in only two dimensions
    droneposition=drone(1:3);
    
    desiredtravel=finalobjective(1:3)-droneposition;
    desiredtravel=desiredtravel/norm(desiredtravel);

    conflicttravel=avoidanceobjective(1:3)-droneposition;
    conflicttravel=conflicttravel/norm(conflicttravel);
    
    angle = atan2d(norm(cross(desiredtravel,conflicttravel)), dot(desiredtravel,conflicttravel));
    angle = mod(angle,180);
    
    if(avoidanceobjective(4)==-1)
        theta=-theta;
    end

    if (angle<15)
        %rotate objective slightly to the right using a rotational
        %matrix in the x axis
        
        objective(1:3)=avoidanceobjective(1:3)-desiredtravel+((desiredtravel)*[1,0,0;0,cosd(theta),-sind(theta);0,sind(theta),cosd(theta)]);
        objective(1:3)=avoidanceobjective(1:3)-desiredtravel+((desiredtravel)*[cosd(theta),-sind(theta),0;sind(theta),cosd(theta),0;0,0,1]);
        
%Temp
%         objective(1:3)=avoidanceobjective(1:3);
        exitflag=1;
    else
        objective(1:3)=avoidanceobjective(1:3);
        exitflag=0;
    end
