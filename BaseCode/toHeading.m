function [heading] = toHeading(angle)
%Turns angles into headings

    if (angle<-.5)
    heading=angle+360;
    elseif (angle>360)
    heading=mod(angle,360);
    else
    heading=angle;
    end

end

