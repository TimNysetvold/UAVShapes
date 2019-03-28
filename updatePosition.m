function [movement] = updatePosition(drone, objective, ~)
%This function moves the drones towards their objectives. It does so by
%directing the acceleration vector towards the appropriate goal, and then
%numerically integrating the acceleration and velocity functions to get a
%new velocity and position. Gravity is considered in this function; drag
%forces are omitted (instead, we use a speed limit).

% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

        signobj=sign(objective(4));
        objective=objective(1:3);

        gravity=[0,0,-.0000981];
        
        %package the drone data in easy-to-use boxes; this helps when the
        %drone vector is refactored
        
        droneposition=[drone(1),drone(2),drone(3)];
        dronepositionhist=[drone(1),drone(2),drone(3)];
        dronevel=[drone(4),drone(5),drone(6)];
        dronemaxspeed=drone(7);
        dronemaxaccel=drone(8);
        dronepause=drone(17);
        disttoobj=norm(droneposition-objective);
        speed=norm(dronevel);
        priority=drone(15);
        
    if(dronepause>0)
        %First, we check that the drone isn't in "time out". If it is, we
        %decrement its pause value and pass it back unchanged.
        drone(17)=drone(17)-1;
        movement=drone;
    else
        %If the drone is not in time out, it will have to move somehow and
        %we will wind up numerically integrating its acceleration. First,
        %we must determine its direction of desired travel.
        heading=getHeading(objective,droneposition);
        unithead=heading/norm(heading);
        
            if all(~any(dronevel)) %if the UAV is entirely stopped, we
                %would break the sim trying to find its current direction.
                %we assign it a random one.
                unitvel=[rand(),rand(),rand()];
            else
                unitvel=dronevel/norm(dronevel);
            end

            %The drone only changes direction if the error in its direction is
            %sufficiently large. If it is close to where it should go, it
            %continues to fly the heading.
            if(norm(unithead-unitvel)>.05)
                heading=heading-unitvel; %by subtracting the current velocity 
                %from the desired heading, we get the angle in which the acceleration should be applied
            end
            
            if (any(isnan(heading(:)))) %debug code
               disp 'oh gnoez'
            end

            
            %Now, we must consider gravity and solve for the correct
            %acceleration vector. We do this via the laws of sines and
            %cosines (hopefully I'll put a derivation somewhere useful for
            %this).
            
            gravitymag=norm(gravity);
            cosA=dot(gravity,unithead);
            A=acos(cosA);
            C=real(asin((sin(A)*gravitymag))/dronemaxaccel);
            %this may have an error, but I don't think so?
            B=pi-A-C;
            accelvecmag=sqrt(gravitymag^2+dronemaxaccel^2-2*gravitymag*dronemaxaccel*cos(B));
            accelvec=heading*accelvecmag;
            %Now, make sure the drone's acceleration points the correct direction
            dronecorrection=accelvec*signobj;
            
            stopdist=dronemaxspeed*.75;

            %Changing the velocity of the drone
            if disttoobj<stopdist
                %if the drone is close to its objective and must begin to
                %slow down to reach a stop at it, it slows down:
                dronevel=dronevel-dronecorrection;
            else
                %if not, it accelerates towards its objective.
            dronevel = dronevel+dronecorrection;
            end

            speed=norm(dronevel);
            %if the velocity is faster than the allowed speed, normalize the
            %vector and then multiply it by its allowed speed
            if(speed>dronemaxspeed)
                dronevel=(dronevel/speed)*dronemaxspeed;
            end

            %Changing the position of the drone
            droneposition=round(droneposition+dronevel,4);

            %save the new x and y position and velocity of drone:
            for i=1:3
            drone(i) = droneposition(i);
            end
            for i=1:3
            drone(i+3) = dronevel(i);
            end

            %debugging
            speed=norm(dronevel);
            if(speed>dronemaxspeed+.0001)
                disp 'error'
            end

            %plot the proposed motion for debugging
%             quiver3(dronepositionhist(1),dronepositionhist(2),dronepositionhist(3),unithead(1),unithead(2),unithead(3),1000)
%             quiver3(dronepositionhist(1),dronepositionhist(2),dronepositionhist(3),dronevel(1),dronevel(2),dronevel(3),1000)
%             
%             quiver3(dronepositionhist(1),dronepositionhist(2),dronepositionhist(3),dronecorrection(1),dronecorrection(2),dronecorrection(3),1000)
            
            %pass drone to outside world
            movement = drone;
            if (any(isnan(movement(:))))
               disp 'oh gnoez'
            end
        
    end
    
end

