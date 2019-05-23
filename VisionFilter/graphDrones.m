function [] = graphDrones(totaldronearray,patharray,newcrashes,linecapture,baselength,iteration)


% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

    hold off
    scatter3(totaldronearray(:,1),totaldronearray(:,2),totaldronearray(:,3),'ok')
%   view([0 0]);
    hold on
%    scatter3(totaldronearray(input1*2:end,1),totaldronearray(input1*2:end,2),totaldronearray(input1*2:end,3),'or')
%    scatter3(totaldronearray(4,1),totaldronearray(4,2),totaldronearray(4,3),'or')

   
    if ~isempty(newcrashes)
        scatter3(newcrashes(:,3),newcrashes(:,4),newcrashes(:,5),'xr')
    end
   
   for i=1:size(linecapture.x,1)
    plot3(linecapture.x(i,:),linecapture.y(i,:),linecapture.z(i,:));
   end
    
   for q=1:size(patharray,1)
        temp=patharray{q,1};
        curobjs(q,:)=temp(1,end-3:end-1);
   end
%     scatter3(curobjs(:,1),curobjs(:,2),curobjs(:,3),'pg')
%     viscircles([totaldronearray(:,1),totaldronearray(:,2)],ones(size(totaldronearray(:,1)))*totaldronearray(1,16),'Color','b');
    

    axis([0 baselength 0 baselength 0 1])
    % axis equal
    %%Debug window
    %axis([4.8,5.2,4.8,5.2,0,1])
    axis([4,6,4,6,0,1]);
    
    %axis equal
%     az=(iteration-1000)/10
%     el=45;
%     view([az,el])
    pause(.0001);
    
% if iteration>1000&&iteration<1500
%     
%     filename = 'SpinningSimGif.gif';
%     % Draw plot for y = x.^n
%       % Capture the plot as an image 
%       frame = getframe; 
%       im = frame2im(frame); 
%       [imind,cm] = rgb2ind(im,256); 
%       % Write to the GIF File 
%       if ~exist('SpinningSimGif.gif','file')
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.0001); 
%       else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%       end 
% end
    
end