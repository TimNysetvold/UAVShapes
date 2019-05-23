function finalobjective = PotentialFieldsZActive(j,dronearray,conflictindex,shipobjective)

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

r=1;
a=10;
g=1;

for i=1:size(conflictindex,1)
    %Finds distance between ownship and each conflicting drone. It is
    %easier to do this here than to import the data from the
    %conflictsarray; passing a huge array is not worth it    
    
    curconflictdrone=dronearray(conflictindex(i),:);
    xo=curconflictdrone(1);
    yo=curconflictdrone(2);
    zo=curconflictdrone(3);
    
    xdist=x-xo;
    ydist=y-yo;
    zdist=z-zo;
    totaldist=sqrt(xdist^2+ydist^2+zdist^2);
    direction=[(xdist);(ydist);(zdist)];
    direction=direction/norm(direction);
    
    
%     thisdronerepulsex=sign(xdist)*exp(-1/2*(xdist)^2)+(radius)*exp(-1/2*(radius)^2)*direction(1);
%     thisdronerepulsey=sign(ydist)*exp(-1/2*(ydist)^2)+(radius)*exp(-1/2*(radius)^2)*direction(2);
%     thisdronerepulsez=sign(zdist)*exp(-1/2*(zdist)^2)+(radius)*exp(-1/2*(radius)^2)*direction(3);
    %thisdronerepulsez=0;
    
    
    
    thisdronerepulsex=sign(xdist)*exp(-1/2*(xdist/radius)^2);
    thisdronerepulsey=sign(ydist)*exp(-1/2*(ydist/radius)^2);
    thisdronerepulsez=sign(zdist)*exp(-1/2*(zdist/radius)^2);
    
    repulseforce(1)=repulseforce(1)+thisdronerepulsex;
    repulseforce(2)=repulseforce(2)+thisdronerepulsey;
    repulseforce(3)=repulseforce(3)+thisdronerepulsez;
    
    
end

repulseforce=repulseforce/norm(repulseforce);
scaling_factor=radius/abs(totaldist);
repulseforce=repulseforce*r*scaling_factor;
    
xa=shipobjective(1);
ya=shipobjective(2);
za=shipobjective(3);
attractforce(1)=-(x-xa);
attractforce(2)=-(y-ya);
attractforce(3)=-(z-za);
attractforce=attractforce/norm(attractforce)*a;

if(z<.025)
    groundforce=g;
else
    groundforce=0;
end

G1=attractforce(1)+repulseforce(1);
G2=attractforce(2)+repulseforce(2);
G3=attractforce(3)+repulseforce(3)+groundforce;
G=[G1,G2,G3];


G=G/norm(G);



%%debug graphing
%hold on
%quiver3(ownship(1),ownship(2),ownship(3),G(1),G(2),G(3))

G=[G+ownship(1:3),1];
% finalobjective=G

G=checkObjectiveColinearity(ownship,shipobjective,G);
finalobjective=[G,1];

%         if (any(isnan(finalobjective(:))))
%            disp 'oh gnoez'
%         end
        
end

