function [altitudeLayers] = layerCreator(altlength, numLayers)

minAltitude = .1;
maxAltitude = altlength;
altitudeLayers = zeros(1,numLayers);
if numLayers == 1
    altitudeLayers = ((maxAltitude - minAltitude) / 2) + minAltitude;
else
    altitudeInterval = ((maxAltitude - minAltitude) / (numLayers));
    
    for i = 1:numLayers
        altitudeLayers(i) = minAltitude + (altitudeInterval * (i-1));
    end
end