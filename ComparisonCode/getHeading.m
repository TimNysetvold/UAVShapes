function [heading] = getHeading(objective, droneposition)
    %Determine the new required heading of the drone
    headingvector=objective-droneposition;
    
    if all(~any(headingvector)) %If the heading is zero, we have a problem.
        %This can only happen when a drone is on top of its objective,
        %which should never occur. Nevertheless, 
        %The sim cannot norm a zero vector, so we force it to go east as we
        %debug.
        headingvector=[1,0,0];
    end
    
    heading=headingvector/norm(headingvector);