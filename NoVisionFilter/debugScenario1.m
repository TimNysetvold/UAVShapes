function [totaldronearray,objectivearray] = debugScenario1(totaldronearray,objectivearray)
% %Debug code forcing certain drones to go to certain spots
% Case 1
objectivearray(1,1:3)=[8,7,.4];
objectivearray(2,1:3)=[8,8.5,.4];
objectivearray(3,1:3)=[8,9,.4];
objectivearray(4,1:3)=[8,8,.4];
totaldronearray(1,1:3)=[4.5,4.5,.4];
totaldronearray(2,1:3)=[4.4,4.4,.4];
totaldronearray(3,1:3)=[4.6,4.6,.4];
totaldronearray(4,1:3)=[4.5,4.6,.4];

end

