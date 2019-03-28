function finalobjective = PotentialFields(j,dronearray,conflictindex,shipobjective,iteration)

% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

ownship=dronearray(j,:);
shipobjective=shipobjective{j,1};
repulseforce=zeros(3,1);
radius=ownship(16);
attractforce=zeros(3,1);

x=ownship(1);
y=ownship(2);
z=ownship(3);


% if z<-.015
%     disp 'udg'
% end

r=5;
a=3;
g=2;

for i=1:size(conflictindex)
    %Finds distance between ownship and each conflicting drone. It is
    %easier to do this here than to import the data from the
    %conflictsarray; passing a huge array is not worth it
    curconflictdrone=dronearray(conflictindex(i),:);
    xo=curconflictdrone(1);
    yo=curconflictdrone(2);
    zo=curconflictdrone(3);
    
    direction=[(x-xo);(y-yo);(z-zo)];
    direction=direction/norm(direction);
    
    thisdronerepulsex=-(x-xo)*exp(-1/2*(x-xo)^2)+(radius)*exp(-1/2*(radius)^2)*direction(1);
    thisdronerepulsey=-(y-yo)*exp(-1/2*(y-yo)^2)+(radius)*exp(-1/2*(radius)^2)*direction(2);
    thisdronerepulsez=-(z-zo)*exp(-1/2*(z-zo)^2)+(radius)*exp(-1/2*(radius)^2)*direction(3);
    
    
    
    %because of the nature of the PF method, it sees that the drones are at
    %very similar heights and devotes the majority of its effort to
    %changing them. We will need to examine this in a more detailed way
    %later, but for now we will ignore it

    repulseforce(1)=repulseforce(1)+thisdronerepulsex;
    repulseforce(2)=repulseforce(2)+thisdronerepulsey;
    repulseforce(3)=repulseforce(3)+thisdronerepulsez;
    
    
    
end

    repulseforce=repulseforce*r;
    
xa=shipobjective(1);
ya=shipobjective(2);
za=shipobjective(3);
attractforce(1)=-(x-xa);
attractforce(2)=-(y-ya);
attractforce(3)=-(z-za);

groundforce=g*(z+1.03)^-20;
%groundforce=g*(z+.9)^-20;
%groundforce=groundforce*sign(z);

if groundforce>10
%    disp oops
end
%forces groundforce to point up

% if (attractforce(1)==0||attractforce(3)==0) %I forget what this does; must figure it out again
%     attractforce(1)=0;
%     attractforce(2)=0;
%     attractforce(3)=0;
% else
    attractforce=attractforce/norm(attractforce)*a;
% end

% attractforcex=0;
% attractforcey=0;

G1=attractforce(1)+repulseforce(1);
G2=attractforce(2)+repulseforce(2);
G3=attractforce(3)+repulseforce(3)+groundforce;


%quiver(ownship(1),ownship(2), .01*G1, .01*G2);
G=[G1,G2,G3];
G=G/norm(G);
% hold on

G=[G+ownship(1:3),1];
G=checkObjectiveColinearityDeprecated(ownship,shipobjective,G,15);
finalobjective=[G,1];

%debug graphing
% quiver3(ownship(1),ownship(2),ownship(3),G(1),G(2),G(3))
% 

if iteration>500
    objectivevec=G(1:3)-ownship(1:3);
    quiver3(ownship(1),ownship(2),ownship(3),objectivevec(1),objectivevec(2),objectivevec(3))
end
    if (any(isnan(finalobjective(:))))
           disp 'oh gnoez'
        end
        
end



% % % % %old, long (computationally intensive) form
% % % % 
% % % % yo(3)=ya;
% % % % xo(3)=xa;
% % % % xnorm =(xs-xo).^2
% % % % ynorm = (ys-yo).^2
% % % % 
% % % % repulse1 =-r*(1/(1/2*xnorm(1)+1/2*ynorm(1)));
% % % % repulse2 = -r*(1/(1/2*xnorm(2)+1/2*ynorm(2)));
% % % % %attract1 = attract*cos(theta1)*x+attract*sin(theta1)*y;
% % % % attract1 = a*(1/(1/2*xnorm(3)+1/2*ynorm(3)));
% % % % resultant = repulse1+repulse2+attract1;
% % % % % resultant = repulse1+repulse2
% % % % 
% % % % g = gradient(resultant, [xs,ys])
% % % % G1s = subs(g(1), [xs ys], {ownship(1),ownship(2)});
% % % % G2s = subs(g(2), [xs ys], {ownship(1),ownship(2)});
% % % % G1s = double(G1s);
% % % % G2s = double(G2s);




%Archived here is my first semi-successful attempt at this. I'm unsure that
%it actually successfully implements potential fields, but it does do
%SOMETHING that could be construed as avoidance.
% % % ownship=dronearray(j,:);
% % % shipobjective=shipobjective{j,1};
% % % 
% % % %this constant is arbitrary
% % % attract=5;
% % % 
% % % theta1=atan2((shipobjective(2)-ownship(2)),(shipobjective(1)-ownship(1)));
% % % 
% % % 
% % % attractforcex=attract*cos(theta1);
% % % attractforcey=attract*sin(theta1);
% % % 
% % % %multiply the force given by the potential field equation by the angle to
% % % %the attractor or repulsor. Then, add all of these things together.
% % % gradientx=attractforcex;
% % % gradienty=attractforcey;
% % % 
% % % for i=1:size(conflictindex(:,1))
% % %     %Finds distance between ownship and each conflicting drone. It is
% % %     %easier to do this here than to import the data from the
% % %     %conflictsarray; passing a huge array is not worth it
% % %     curconflictdrone=dronearray(conflictindex(i),:);
% % %     dist=pdist([ownship(1:2);curconflictdrone(1:2)]);
% % %     theta2=atan2((curconflictdrone(2)-ownship(2)),(curconflictdrone(1)-ownship(1)));
% % %     
% % %     %Using this conflict distance, we find the force that this intruder will
% % %     %exert on the ownship. This will be used to determine the gradient.
% % % %     repulseforce(1)=-br*yr*dist*exp(-yr/2*dist^2)*cos(theta2);
% % % %     repulseforce(2)=-br*yr*dist*exp(-yr/2*dist^2)*sin(theta2);
% % %     repulseforce(1)=(-1/(dist.^2)*cos(theta2))*.1;
% % %     repulseforce(2)=(-1/(dist.^2)*sin(theta2))*.1;
% % % 
% % %     %We add all of the individual UAV's forces to create a single gradient
% % %     %for the pf approach.
% % %     
% % % 
% % %     
% % %     gradientx=gradientx+repulseforce(1);
% % %     gradienty=gradienty+repulseforce(2);
% % %     
% % % end
% % % 
% % % 
% % % 
% % % 
% % %     %The direction in which the gradient is at a maximum is equal to this
% % %     %expression (angle given with respect to the positive x axis). We can
% % %     %then use that to create an objective by rotating a unit vector by theta;
% % %     %the objective is a little farther than
% % %     %the UAV can reach this step in that direction.
% % %     
% % %     %quiver(ownship(1),ownship(2),gradientx,gradienty)
% % %     
% % %     theta3=atan2(gradienty,gradientx);
% % %     %Because of the rest of the code, it make much more to have an
% % %     %avoidant point than an attractive point. This just means we need to
% % %     %change what side of the drone we're looking at.
% % %     intermedobjective=[cos(theta3),-sin(theta3);sin(theta3),cos(theta3)]*[1;0];
% % %     intermedobjective=intermedobjective/norm(intermedobjective);
% % %     intermedobjective=[intermedobjective'+ownship(1:2),1];
% % %     
% % %     intermedobjective=checkObjectiveColinearity(ownship,shipobjective,intermedobjective,1);
% % % 
% % % finalobjective=[intermedobjective,1]';
% % % end


%   For reference: the code from LMA
%    ownship=dronearray(j,:);
%     allconflictdistances=zeros(size(conflictindex,1),1);
%     
%     for i=1:size(conflictindex(:,1))
%         curconflictdrone=dronearray(conflictindex(i),:);
%         dist=pdist([ownship(1:2);curconflictdrone(1:2)]);
%         allconflictdistances(i,:)=dist;
%     end
%     
%     [~,index]=min(allconflictdistances);
%     trueindex=conflictindex(index(1),:);
%     conflictdrone=dronearray(trueindex(1),:);
%     
%     
%     objective=[conflictdrone(1:2)';-1];