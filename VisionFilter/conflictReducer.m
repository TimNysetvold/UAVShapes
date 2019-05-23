function newConflictIndex = conflictReducer(oldConflictIndex,visibleDrones)
if length(visibleDrones) > 1
    visibleDroneCombinations = nchoosek(visibleDrones,2);
    reversal = [visibleDroneCombinations(:,2),visibleDroneCombinations(:,1)];
    visibleDronePermutations = [visibleDroneCombinations;reversal];
    try 
    newConflictIndexLogicals = ismember(oldConflictIndex,visibleDronePermutations,'rows');
    catch me
        disp catch
    end
    zerosConflictIndex = oldConflictIndex(newConflictIndexLogicals,:);
    newConflictIndex = zerosConflictIndex(any(zerosConflictIndex,2),:);
else 
    newConflictIndex = [];
end