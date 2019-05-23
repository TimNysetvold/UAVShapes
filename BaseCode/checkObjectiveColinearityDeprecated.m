function [objective] = checkObjectiveColinearity(drone,finalobjective,intermediateobjective,theta)
    %If we are avoiding an intermediate, and that intermediate is
    %colinear with the drone's current flight path, we turn to the
    %right to avoid it. The angle necessary to make this happen may depend
    %on the method, so we parameterize that and make this a function.

    droneline=finalobjective(1:3)-drone(1:3);
    droneline=droneline/norm(droneline);

    objectiveline=intermediateobjective(1:3)-drone(1:3);
    objectiveline=objectiveline/norm(objectiveline);
    
    if(intermediateobjective(4)==-1)
        theta=-theta;
    end

    if (all(abs(droneline-objectiveline)<.25)||all(abs(droneline+objectiveline)<.25))
        %rotate objective slightly to the right using a rotational
        %matrix in the x axis
        objective(1:3)=intermediateobjective(1:3)-droneline+((droneline)*[1,0,0;0,cosd(theta),-sind(theta);0,sind(theta),cosd(theta)]);

    else
        objective(1:3)=intermediateobjective(1:3);
    end
