function [ conflictindex ] = findConflicts(distances,totaldronearray)
%This funciton returns the indexes of drones which are infringing on other
%drone's airspace. The index of the drone responsible for avoidance is returned in the
%first column; the drone that it is avoiding is returned in the second.

% each drone has is an 21-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority, 16. separation standard, 17. pause, 18. loiter]

conflictindex=[];
%Remember that in the drone vector, 12 is separation standard and 13 is priority.
%   Takes a matrix of distances between drones, analyzes by priorities to
%   determine which are infringing. Uses paremeter "infringezone" to
%   determine what's the cutoff.

% we check ONLY IF i must avoid j in any given check. j will be checked separately. 

%this function makes me cry

    for i=1:size(totaldronearray,1)
        
        collisiondistance=totaldronearray(i,16); %J16 is the separation standard for the drone being checked
        %traverse the array for each drone.
            
            
        index=find(distances(i,:)<collisiondistance);
        %index now contains the drone number for the drones that are in
        %conflict with drone i.
        
        allconflictspriority=totaldronearray(index,15);
        %stubarray holds the priorities of drones in conflict with the
        %current i drone.
        
        highpriorityconflictsindexofindexes=find(allconflictspriority<=totaldronearray(i,15));

        highpriorityconflictsindex=index(highpriorityconflictsindexofindexes);
        
        
        if(~isempty(highpriorityconflictsindex))
            %Does this work? Probably not
            for j=1:length(highpriorityconflictsindex)
            %if the distances between the drones is less than
            %drone j's infringment zone, and i has a higher priority
            %value than j does (higher is worse), i must avoid j. 

                %BUT only if neither drone is paused; else do nothing
                if (totaldronearray(i,17)==0&&totaldronearray(highpriorityconflictsindex(j),17)==0)
                conflictindex=[conflictindex;i,highpriorityconflictsindex(j)];
                end
            end
        end
    end
end
