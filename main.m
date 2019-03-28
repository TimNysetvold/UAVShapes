 function numcrashes = main (input1,input2)
%This is the base code for the drone
%simulator for use by Dr. John Salmon.
%Originally written by Tim Nysetvold.

close all

%RNG data. This gives each simulation a different seed,
%which is recorded so that the sim can be replayed if necessary.
rng('shuffle');
scurr=rng;

% Basic parameters for simulation.
% 1 iteration is equal to 0.1 second of flight time in simulation.
iterations=288000/4; %should be 288000 for 8 hours

%stopper=250;
collisiondistance=.15;
% the FAA has declared the "near miss" distance for drones to be 500 ft
% (150 meters). This would have a value of .15 in this sim.
metalcollisiondistance=.015;
% 150m will probably not result in real collisions between drones.
% We use this metric to determine when a metal-on-metal crash would actually be likely.
baselength=10;
altlength=1;
% The base length determines the size of the city the drones inhabit. It is
% given in kilometers. 
layers=1;


radius=input1;
angle=input2;

%%Separation Standards:
phiStandard=angle*(pi/180);
thetaStandard=angle*(pi/180);
singleAngleStandard = angle*(pi/180);
xy_standard=radius;
z_standard=radius/2;
speedLimit = .004;
lookAheadtime = radius/speedLimit;

% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

% Current types available: 1=police; 2=AmazonAir; 3=Hobbyist; 4=Military
% Remember, priority 1 is the highest priority; priority infinity would be
% the lowest.
 
% droneset 1
numdrones(1)=100; %how many UAVs
cruiseheight(1)=.55;
xbase(1)=baselength/2; %the home base x position of these UAVs
ybase(1)=baselength/2; %the home base y position of these UAVs
dronemaxvel(1)=speedLimit; %this is equal to 30 m/(tick). This value must be input in kilometers per tick
dronemaxaccel(1)=0.0005; %the maximum acceleration available to this UAV. Given in km/(tick^2), which kind of sucks
separation(1)=radius*2; %the separation standard this UAV attempts to maintain
dronestamina(1)=1000000; %the battery or fuel life of these UAVs
priority(1)=2; %How much responsibility this UAV has for avoidance. 1 is least, but only matters in comparison.
type(1)=2;
pause(1)=0; %UAVs do not start out paused

%droneset 2
numdrones(2)=0;
cruiseheight(2)=.55;
xbase(2)=baselength/2;
ybase(2)=baselength/2;
dronemaxvel(2)=speedLimit;
dronemaxaccel(2)=0.0005;
separation(2)=radius*2;
dronestamina(2)=1000000;
priority(2)=3;
type(2)=2;
pause(2)=0;

loiter=0; %Currently unused for all drones


%things that we're recording
activedrones=zeros(iterations,1); %how many drones are actually flying per iteration
metalcrashes=[]; %Vector that will store locations and drone identities of crashes
conflictsrecord=0; %how many conflicts have occurred
faacrashes=0; %how many NMACs under current FAA definition
global objectivesreached;
objectivesreached=0; %how many goals have been accomplished

%Debugging code; used to keep track of a few drones
linecapture.x=[];
linecapture.y=[];
linecapture.z=[];
 
% Creates objective and drone arrays for use in the main loop. Using multiple
% small arrays allow for multiple types of drones to co-exist easily.
% The matrix totaldronearray(num) will have a number
% of drones numdrones, created with parameters
% input at initiation. Saves original drones in 
% stats for data lookup if necessary.

dronestats=zeros(length(numdrones),18);
totaldronearray=zeros(sum(numdrones),18);
objectivearray=[];

for i=1:length(numdrones)
    if i==1
        start=1;
    else
        start=sum(numdrones(1:(i-1)))+1;
    end
    totaldronearray(start:sum(numdrones(1:i)),:)=newDroneArray(numdrones(i),dronemaxvel(i),dronemaxaccel(i),separation(i),priority(i),type(i),dronestamina(i),xbase(i),ybase(i),pause(i),baselength,loiter,cruiseheight(i));
    dronestats(i,:)=newDroneArray(1,dronemaxvel(i),dronemaxaccel(i),separation(i),priority(i),type(i),dronestamina(i),xbase(i),ybase(i),pause(i),baselength,loiter,cruiseheight(i));
end

for i=1:length(numdrones)
    if i==1
        start=1;
    else
        start=sum(numdrones(1:(i-1)))+1;
    end
    objectivearray=[objectivearray;newObjectiveArray(totaldronearray(start:sum(numdrones(1:i)),:),baselength)];
end

%[totaldronearray,objectivearray]=debugScenario1(totaldronearray,objectivearray);

%Converts beginning objectives into the patharray, which may become ragged
%as time progresses

patharray=num2cell(objectivearray,2);

%Combines arrays for ease of in-loop processing.
%patharray=num2cell(objectivearray); 
intermediatearray0s=zeros(numdrones(1)+numdrones(2),4); %This array has any unexpected deconfliction waypoints along the path.
layerMat=layerCreator(altlength, layers);

for i=1:size(totaldronearray,1)
    totaldronearray(i,3)=AltDetLayers([objectivearray(i,1:2),totaldronearray(i,3)],totaldronearray(i,1:3),layerMat);
    totaldronearray(i,12)=totaldronearray(i,3);
    patharray{i}(1,3)=totaldronearray(i,3);
end
markeddrones=1:5:(numdrones);
%markeddrones=[markeddrones,(input1*2):(input1*2)+3];
%markeddrones=1:11;

for currentIteration=1:iterations
    %This is the main control loop for the simulation. Each pass through
    %the loop is one tick.
    
    intermediatearray=intermediatearray0s;
    %We reset the list of intermediate steps.
    
    [distances_xyz,distances_xy,distances_z] = analyzeDistances(totaldronearray);
    
    [totaldronearray,newcrashes]=findMetalCrashes(distances_xyz,metalcollisiondistance,totaldronearray,currentIteration,baselength);
    metalcrashes=[metalcrashes;newcrashes];
    [newcrashes]=findFAACrashes(distances_xyz,faacrashes,collisiondistance,totaldronearray,currentIteration,baselength);
    faacrashes=faacrashes+size(newcrashes,1);
    %This section determines crashes that have occurred.
    
    %Regular, global knowledge
    conflictindex=findConflicts(distances_xyz,totaldronearray);
    %Using prefilter
    
    conflictsrecord=conflictsrecord+size(conflictindex,1);
    %determine conflicts- if a drone is in the bubble of an equal or higher
    %priority drone, return that drone's index and the index of its
    %superior
    
    linecapture.x=[linecapture.x,totaldronearray(markeddrones,1)];
    linecapture.y=[linecapture.y,totaldronearray(markeddrones,2)];
    linecapture.z=[linecapture.z,totaldronearray(markeddrones,3)];
    graphDrones(totaldronearray,patharray,newcrashes,linecapture,baselength,currentIteration)

    for i=1:sum(numdrones)
       %In this section, each drone moves.
                     
       if(~isempty(conflictindex)) %if there is a conflict, find the entry to which it corresponds
           [row,~]=find(conflictindex(:,1)==i);
            curconflicts=conflictindex(row,2);
            %Now, curconflicts has the index of all drones infringing on the
            %current drone. If none of the conflicts in the index involve
            %it, then this array will be empty and we'll ignore it.
            if (~isempty(curconflicts))
            %Use an algorithm like Chain, A*, or LMA here
            
            %%New code here right before potential fields
            intermediatearray(i,1:4)=PotentialFields(i,totaldronearray,curconflicts,patharray,iterations);
            end
       end
       %must create a method for drones who are avoiding to reach
       %objectives. Right now, they pass through potential fields but not
       %GetObjective, which is where they can actually accomplish
       %objectives.
       
       
       %If the drone is not avoiding anything, it looks for the next point
       %along its path.
%       if(~any(intermediatearray(i,:)))
           [intermediatearray(i,:),patharray{i,1}]=getObjective(totaldronearray(i,:),intermediatearray(i,:),patharray{i,1},baselength,layerMat);     
%       end
% 
       
       %move towards its objective
       totaldronearray(i,:)=updatePosition(totaldronearray(i,:),intermediatearray(i,1:4),patharray{i,1});
        
        if (any(isnan(totaldronearray(:))))
           disp 'oh gnoez'
        end
    end

   %decrement stamina of drones
   totaldronearray(:,15)=totaldronearray(:,15)-1;

   %corrupting distances to find minimum non-collision distance
%    distances_xyz(distances_xyz<=collisiondistance)=NaN;
%    mindist(currentIteration,1)=min(min(distances_xyz));
%    mindist(currentIteration,2)=mean(mean(distances_xyz,'omitnan'),'omitnan');
%    mindist(currentIteration,3)=median(median(distances_xyz,'omitnan'),'omitnan');
    
   %Find how many drones are in the air.
   activedrones(currentIteration)=nnz(~totaldronearray(:,12));
       
end

%This section saves all variable to a .m file. The
%title of the file is data, followed by parameters specific to
%each simulation. These work like this:
newname=0;
k=0;
name = 'fail'; 
while newname==0
    k=k+1;
    name=(strcat('zzz',num2str(input1),'Radius',num2str(input2),'Angle',num2str(k),'rep.mat'));
    if ~exist(strcat('zzz',num2str(input1),'Radius',num2str(input2),'Angle',num2str(k),'rep.mat'))
        break;
    end
end
save(name)
%now, for ease of crunching, drop the crashes into here.
numcrashes=[size(metalcrashes,1),faacrashes,conflictsrecord,input1,input2,objectivesreached]
dlmwrite('aaatotalcrashesdata.csv',numcrashes,'-append')

exit




%    plot(linecapturex,linecapturey);
%    plot(linecapturex2,linecapturey2);
%    plot(linecapturex3,linecapturey3);


