function [totaldronearray, crashesinside] = findMetalCrashes(distances,collisiondistance,totaldronearray,iteration,baselength)
%Lists all metal-on-metal crashes in a certain iteration of the sim. Sends home and causes
%to pause drones that have crashed.

% each drone has is a 21-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

    crashesinside=[];

    %if the distance between two drones is less than the collision
    %distance, the one drone becomes "crasher" and the other
    %drone becomes "crashee". Both are recorded, along with their
    %positions, and then both are sent to their respective bases with a
    %timeout.
    
    %remember that drone index 17 is pause.
    
    [crasher,crashee]=find(distances<collisiondistance);
    
    if ~isempty(crasher)
        
        
        for i=1:size(crasher,1)
        %for all crashes, record the x and ys, send drones home and pause
        %them. However, if one of the drones involved is paused, ignore the
        %crash- it hasn't happened.
        currentwait=max(totaldronearray,[],1);
        currentwait=currentwait(17);
            if (totaldronearray(crasher(i),17)~=0||totaldronearray(crashee(i),17)~=0)
                %do nothing; drones are paused
%             elseif (pdist([totaldronearray(crasher(i),1:2);baselength/2,baselength/2])<baselength/10)
%                 %do nothing; within tower control area
            else
                posx=totaldronearray(crasher(i),1);
                posy=totaldronearray(crasher(i),2);
                posz=totaldronearray(crasher(i),3);
    
                totaldronearray(crasher(i),1:3)=totaldronearray(crasher(i),9:11);
                totaldronearray(crasher(i),17)=currentwait+15;        
    
                totaldronearray(crashee(i),1:3)=totaldronearray(crashee(i),9:11);
                totaldronearray(crashee(i),17)=currentwait+7;

                crashesinside=[crashesinside;crasher(i),crashee(i),posx,posy,posz,iteration];
            end
        end
    end
end

