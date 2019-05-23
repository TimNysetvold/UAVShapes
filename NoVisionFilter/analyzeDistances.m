function [distances, distances_xy, distances_z] = analyzeDistances(dronearray)
%Sees what drones have crashed into each other.
  
%     dronepositions=dronearray(:,1:3);
%     if (~isreal(dronepositions))
%         disp 'oh gnoez'
%     end
%     distances=pdist(dronepositions);
%     distances=squareform(distances);
%     
%     %now eliminate the diagonal of the matrix; it will be all 0s, which
%     %leads the sim to believe aircraft are crashing into themselves
%     n=size(distances,1);
%     distances(1:n+1:end) = NaN;  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Start Seth Code
    
    %%%%XY STUFF
    dronepositions_xy=dronearray(:,1:2);
    if (~isreal(dronepositions_xy))
        disp 'oh gnoez'
    end
    distances_xy=pdist(dronepositions_xy);
    distances_xy=squareform(distances_xy);
    %now eliminate the diagonal of the matrix; it will be all 0s, which
    %leads the sim to believe aircraft are crashing into themselves
    n_xy=size(distances_xy,1);
    distances_xy(1:n_xy+1:end) = NaN;  

    
    %%%%Z STUFF
    dronepositions_z=dronearray(:,3);
    if (~isreal(dronepositions_z))
        disp 'oh gnoez'
    end
    distances_z=pdist(dronepositions_z);
    distances_z=squareform(distances_z);
    %now eliminate the diagonal of the matrix; it will be all 0s, which
    %leads the sim to believe aircraft are crashing into themselves
    n_z=size(distances_z,1);
    distances_z(1:n_z+1:end) = NaN;
    
    %%%%%Get Total Distances
    distances = (distances_xy.^2+distances_z.^2).^(1/2);
    %End Seth Code
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

