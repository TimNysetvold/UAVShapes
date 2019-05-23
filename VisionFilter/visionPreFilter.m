function [ conflictindexreturn ] = visionPreFilter(i,dronearray,conflictindex,lookForward)

% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

thisdrone = dronearray(i,:);
thisdroneposition = thisdrone(1,1:3);
futurePosition = thisdroneposition+thisdrone(4:6)*lookForward;
xp = futurePosition(1);
yp = futurePosition(2);
zp = futurePosition(3);

%conflictindex(conflictindex==i) = [];               %eliminate this drone from the list we look for
curconflictdrones=dronearray(conflictindex(:),:);    %find all the drones that conflict with this one
curconflictpositions = curconflictdrones(:,1:3);     %Take their positions
x = curconflictpositions(:,1);
y = curconflictpositions(:,2);
z = curconflictpositions(:,3);

distancesToFutureLocation = sqrt((x-xp).^2 + (y-yp).^2 + (z-zp).^2);

radius = thisdrone(7)*1.5*lookForward+thisdrone(16); %Max speed times lookforward time plus separation standard
% scatter3(xp,yp,zp)
% viscircles([xp,yp],radius)
conflictindexreturn = conflictindex(distancesToFutureLocation < radius);
