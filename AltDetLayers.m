function [droneCruiseHeight] = AltDetLayers(objective,dronePosition,layerMat)
%A simple layering method for determining what layer drones should fly on.
    numLayers=length(layerMat);
    degreesPerLayer=360/numLayers;
    heading=getHeading(objective,dronePosition);
    angle=headingtoAngle(heading);
    %This line assigns a layer to the drone based on its heading.
    %By dividing the heading by the number of degrees per layer,
    %and rounding the result up, we can assign a layer between 1
    %and numLayers.
    droneCruiseHeight=layerMat(ceil(angle/degreesPerLayer));
end