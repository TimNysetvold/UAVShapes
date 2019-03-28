function [totaldronearray,objectivearray] = debugScenario2(totaldronearray,objectivearray)
% %Debug code forcing certain drones to go to certain spots
% Case 2
objectivearray(1,1:3)=[4.5,4.5,.4];
objectivearray(2,1:3)=[5.5,4.5,.4];
objectivearray(3,1:3)=[4.5,5.5,.4];
objectivearray(4,1:3)=[5.5,5.5,.4];
totaldronearray(1,1:3)=[5.5,5.5,.4];
totaldronearray(2,1:3)=[4.5,5.5,.4];
totaldronearray(3,1:3)=[5.5,4.5,.4];
totaldronearray(4,1:3)=[4.5,4.5,.4];

end