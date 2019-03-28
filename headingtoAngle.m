function [angle] = headingtoAngle(heading)
%This function turns any heading vector into an azimuthal angle.
    east=[1,0,0];
    headingangle=heading/norm(heading);
    dcross=cross(headingangle,east);
    ddot=dot(headingangle,east);
    angle = atan2d(norm(dcross),ddot);
    if (sign(heading(2))<0)
       angle=angle*-1;
    end
    
    if (angle<0)
        angle=angle+360;
    end
end