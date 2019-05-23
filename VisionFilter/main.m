function numcrashes = main(runNum)
%This is the base code for the drone
%simulator for use by Dr. John Salmon.
%Originally written by Tim Nysetvold.

close all

%RNG data. This gives each simulation a different seed,
%which is recorded so that the sim can be replayed if necessary.
rng(1);
scurr=rng;

% Basic parameters for simulation.
% 1 iteration is equal to 0.1 second of flight time in simulation.
iterations=28800; %should be 288000 for 8 hours
collisiondistance=.15;  % the FAA has declared the "near miss" distance for drones to be 500 ft
% (150 meters). This would have a value of .15 in this sim.
metalcollisiondistance=.015;    % 150m will probably not result in real collisions between drones.
% We use this metric to determine when a metal-on-metal crash would actually be likely.
baselength=10;  % The length of the simluated space, in km.
altlength=1;    % The height of the simulated space, in km.
layers=1;       % Number of layers UAVs are divided into.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Input variables

filename = 'ShapesDOE1.csv';
DOE = dlmread(filename);
input = DOE(runNum,:);

    
%Input variables
numDronesTemp = round(input(1));
lookForward = input(4); %Number of seconds we look forward for collision avoidance
avSpeed = input(2);
stdevSpeed = input(3);
separationStandard = input(5);

% %Debug
% numDronesTemp = 4;
% lookForward = 100; %Number of seconds we look forward for collision avoidance
% avSpeed = .0005;
% stdevSpeed = 0;
% separationStandard = .5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

% Current types available: 1=police; 2=AmazonAir; 3=Hobbyist; 4=Military
% Remember, priority 1 is the highest priority; priority infinity would be
% the lowest.
 
speedLimit = .005;

% droneset 1
numdrones(1)=numDronesTemp; %how many UAVs
cruiseheight(1)=.55;
xbase(1)=baselength/2; %the home base x position of these UAVs
ybase(1)=baselength/2; %the home base y position of these UAVs
dronemaxvel(1)=speedLimit; %this is equal to 30 m/(tick). This value must be input in kilometers per tick
avmaxvel(1) = avSpeed;
stdevmaxvel(1) = stdevSpeed;
dronemaxaccel(1)=0.0005; %the maximum acceleration available to this UAV. Given in km/(tick^2), which kind of sucks
separation(1)=separationStandard; %the separation standard this UAV attempts to maintain
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
avmaxvel(2) = avSpeed;
stdevmaxvel(2) = stdevSpeed;
dronemaxaccel(2)=0.0005;
separation(2)=separationStandard;
dronestamina(2)=1000000;
priority(2)=3;
type(2)=2;
pause(2)=0;

loiter=0; %Currently unused for all drones

dronelist = [1:sum(numdrones)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%things that we're recording
activedrones=zeros(iterations,1);   %how many drones are actually flying per iteration
metalcrashes=[];                    %Vector that will store locations and drone identities of crashes
conflictsrecord=0;                  %how many conflicts have occurred
faacrashes=0;                       %how many NMACs under current FAA definition
global objectivesreached;
objectivesreached=0; %how many goals have been accomplished

%Debugging code; used to keep track of a few drones
linecapture.x=[];
linecapture.y=[];
linecapture.z=[];
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    totaldronearray(start:sum(numdrones(1:i)),:)=newDroneArray(numdrones(i),avmaxvel(i),stdevmaxvel(i),dronemaxaccel(i),separation(i),priority(i),type(i),dronestamina(i),xbase(i),ybase(i),pause(i),baselength,loiter,cruiseheight(i));
    dronestats(i,:)=newDroneArray(1,avmaxvel(i), stdevmaxvel(i),dronemaxaccel(i),separation(i),priority(i),type(i),dronestamina(i),xbase(i),ybase(i),pause(i),baselength,loiter,cruiseheight(i));
end

for i=1:length(numdrones)
    if i==1
        start=1;
    else
        start=sum(numdrones(1:(i-1)))+1;
    end
    objectivearray=[objectivearray;newObjectiveArray(totaldronearray(start:sum(numdrones(1:i)),:),baselength)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Optional debug fragments
% 
% [totaldronearray,objectivearray]=debugScenario1(totaldronearray,objectivearray);
% markeddrones=1:(numdrones);
%markeddrones=[markeddrones,(input1*2):(input1*2)+3];
%markeddrones=1:11;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Converts beginning objectives into the patharray, which may become ragged
%as time progresses

patharray=num2cell(objectivearray,2);

%Combines arrays for ease of in-loop processing.
intermediatearray0s=zeros(numdrones(1)+numdrones(2),4); %This array has any unexpected deconfliction waypoints along the path.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generates layers, and places UAVs and objectives into them.
layerMat=layerCreator(altlength, layers);
for i=1:size(totaldronearray,1)
    totaldronearray(i,3)=AltDetLayers([objectivearray(i,1:2),totaldronearray(i,3)],totaldronearray(i,1:3),layerMat);
    totaldronearray(i,12)=totaldronearray(i,3);
    patharray{i}(1,3)=totaldronearray(i,3);
end

%This is the main control loop for the simulation. Each pass through
%the loop is one tick.
for currentIteration=1:iterations
    
    intermediatearray=intermediatearray0s;  %We reset the list of intermediate steps.
    
    
    %We analyze how far UAVs are from each other with this code.
    [distances_xyz,distances_xy,distances_z] = analyzeDistances(totaldronearray);
    
    %This section determines crashes that have occurred.
    [totaldronearray,newcrashes]=findMetalCrashes(distances_xyz,metalcollisiondistance,totaldronearray,currentIteration,baselength);
    metalcrashes=[metalcrashes;newcrashes];
    [newcrashes]=findFAACrashes(distances_xyz,faacrashes,collisiondistance,totaldronearray,currentIteration,baselength);
    faacrashes=faacrashes+size(newcrashes,1);
    
    thisconflictindexInit=findConflicts(distances_xyz,totaldronearray);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Debug code
%     linecapture.x=[linecapture.x,totaldronearray(markeddrones,1)];
%     linecapture.y=[linecapture.y,totaldronearray(markeddrones,2)];
%     linecapture.z=[linecapture.z,totaldronearray(markeddrones,3)];
%     graphDrones(totaldronearray,patharray,newcrashes,linecapture,baselength,currentIteration)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:sum(numdrones)
       %In this section, each drone moves.
       
       %For local knowledge, we determine what UAVs are visible to this UAV
       if(~isempty(thisconflictindexInit))
           %If there are conflicts, we must consider if the UAVs know about
           %them. If there aren't we can skip this step.
           %For local knowledge, we determine what UAVs are visible to this UAV
           visibleDrones = visionPreFilter(i,totaldronearray,dronelist,lookForward);
           thisconflictindex = conflictReducer(thisconflictindexInit,visibleDrones);
       else
           thisconflictindex = thisconflictindexInit;
       end
       
       if(~isempty(thisconflictindex)) %if there is a conflict, find the entry to which it corresponds
           [row,~]=find(thisconflictindex(:,1)==i);
            curconflicts=thisconflictindex(row,2);
            
            %Now, curconflicts has the index of all drones infringing on the
            %current drone. If none of the conflicts in the index involve
            %it, then this array will be empty and we'll ignore it.
            if (~isempty(curconflicts))
            %If there is a conflict, handle it within this statement.
                intermediatearray(i,1:4) = PotentialFieldsZActive(i,totaldronearray,curconflicts,patharray);
            end
            
       end

       %The drone looks for the next point along its path.
       %[xdummy,ydummy] = getObjective(totaldronearray(i,:),intermediatearray(i,:),patharray{i,1},baselength,layerMat);     
       [intermediatearray(i,:),patharray{i,1}]=getObjective(totaldronearray(i,:),intermediatearray(i,:),patharray{i,1},baselength,layerMat);     
       
       %UAV moves
       totaldronearray(i,:)=updatePosition(totaldronearray(i,:),intermediatearray(i,1:4),patharray{i,1});
        
    end

   %decrement stamina of drones
   totaldronearray(:,15)=totaldronearray(:,15)-1;
    
   %Find how many drones are in the air.
   activedrones(currentIteration)=nnz(~totaldronearray(:,12));
       
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This section saves all variable to a .m file. The
%title of the file is data, followed by parameters specific to
%each simulation. These work like this:
newname=0;
k=0;
name = 'fail'; 
while newname==0
    k=k+1;
    name=(strcat('zzz',num2str(runNum),'num',num2str(k),'Shapes.mat'));
    if ~exist(strcat('zzz',num2str(runNum),'num',num2str(k),'Shapes.mat'))
        break;
    end
end
save(name)
%now, for ease of crunching, drop the crashes into here.
numcrashes=[runNum,input,size(metalcrashes,1),faacrashes,objectivesreached,conflictsrecord]
dlmwrite('aaatotalcrashesdata.csv',numcrashes,'-append')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%exit


