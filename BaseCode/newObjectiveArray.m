function [objectivearray] = newObjectiveArray(dronearray,baselength)
%Creates an array of objectives suitable for the type of drone input.

% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

base=[dronearray(:,9),dronearray(:,10),dronearray(:,11)];
cruiseheight=dronearray(:,12);
onevec=ones(length(cruiseheight),1);


if (~isempty(dronearray))
    %if it is empty, an empty set will suffice
    type=dronearray(1,14);
    switch type
        case 4
            objectivearray=rand(size(dronearray,1),2)*baselength;
        case 3
            objectivearray=base;
        case 2
            objectivearray=[base(:,1:2),cruiseheight,onevec,base];
        case 1
            objectivearray=[rand(size(dronearray,1),2)*baselength,rand(size(dronearray,1),1).*cruiseheight+.5*cruiseheight];
        otherwise
            objectivearray=base;
    end
else
    objectivearray=[];
end
    objectivearray=[objectivearray,onevec];
end


