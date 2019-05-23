function [crashesinside] = findFAACrashes(distances,faacrashes,collisiondistance,totaldronearray,iteration,baselength)
%Lists all crashes in a certain iteration of the sim. Sends home and causes
%to pause drones that have crashed.


    crashesinside=[];
    recorded=0;
    %if the distance between two drones is less than the collision
    %distance, the higher-priority drone becomes "crasher" and the lower
    %priority drone becomes "crashee". Both are recorded, along with their
    %positions, and then both are sent to their respective bases with a
    %timeout.
    
    %remember that drone index 17 is pause.
    
    distances=distances+triu(distances)*50;
    %creates garbage in upper half of distances to prevent double-counting
    
    [crasher,crashee]=find(distances<collisiondistance);
    
    if ~isempty(crasher)
        
        
        for i=1:size(crasher,1)
        %for all crashes, record the x and ys.
        %However, make sure extenuating circumstances don't apply.
        %If it was involved in a crash with the same drone last iteration,
        %we will ignore this crash.
        
            if (totaldronearray(crasher(i),17)~=0||totaldronearray(crashee(i),17)~=0)
                %do nothing; drones are paused
%            elseif (pdist([totaldronearray(crasher(i),1:2);baselength/2,baselength/2])<baselength/10)
                %do nothing; within tower control area.
                %This is sometimes necessary for delivery drones
            else
                posx=totaldronearray(crasher(i),1);
                posy=totaldronearray(crasher(i),2);
                posz=totaldronearray(crasher(i),3);

                crashesinside=[crashesinside;crasher(i),crashee(i),posx,posy,posz,iteration];
            end
        end
    end
end

