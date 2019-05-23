function [objective,exitflag] = checkObjectiveColinearity(drone,finalobjective,conflictlocation,avoidanceobjective)
    %If we are avoiding an intermediate, and that intermediate is
    %colinear with the drone's current flight path, we turn to the
    %right to avoid it. The angle necessary to make this happen may depend
    %on the method, so we parameterize that and make this a function.
    theta=3;
    
    droneposition=drone(1:3);
    droneline=finalobjective(1:3)-droneposition;
    droneline=droneline/norm(droneline);

    conflictline=conflictlocation(1:3)-droneposition;
    conflictline=conflictline/norm(conflictline);
    
    if(avoidanceobjective(4)==-1)
        theta=-theta;
    end

    if (all(abs(droneline-conflictline)<.05)||all(abs(droneline+conflictline)<.05))
        %rotate objective slightly to the right using a rotational
        %matrix in the x axis
        
        objective(1:3)=avoidanceobjective(1:3)-droneline+((droneline)*[1,0,0;0,cosd(theta),-sind(theta);0,sind(theta),cosd(theta)]);
        objective(1:3)=avoidanceobjective(1:3)-droneline+((droneline)*[cosd(theta),-sind(theta),0;sind(theta),cosd(theta),0;0,0,1]);
        exitflag=1;
    else
        objective(1:3)=avoidanceobjective(1:3);
        exitflag=0;
    end
