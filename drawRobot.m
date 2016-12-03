function [] = drawRobot(robotShape,TACR)
%% drawRobot.m
% This function visualizes a 2 tendon actuated continuum robot
%
% Copyright: 2016 Leibniz Universität Hannover, All rights reserved
%
% Email: continuumrobotics@lkr.uni-hannover.de
%
% Version: 1
% Date: 11/16/2016
%
% input: struct TACR, q:[jx3] configuration
%        robotShape.diskPoints: n rows for n disks, 12 columns for coordinates
%        (x,y,z) for central backbone, points tendon 1 (x,y,z), points tendon 2 (x,y,z),
%        points tendon 3 (x,y,z)
%        robotShape.diskRotation: n rows for n disks, columns represent
%        rotation matrices (3x3): columns 1-3 are matrix elements (1,1; 1,2; 1,3),
%        columns 4-6 are matrix elements (2,1; 2,2; 2,3), 
%        columns 7-9 are matrix elements (3,1; 3,2; 3,3),
 
figure();
hold on;  

color = [148/255 148/255 148/255];
diskPoints = robotShape.diskPoints;
diskRotation = robotShape.diskRotation;

ndisks = TACR.ndisks;                   
diskRadius = TACR.diskRadius;            
diskHeight = TACR.diskHeight;            
diskPitchRadius = TACR.diskPitchRadius;

l=1;

%% plot disks %%

radialPoints = 50;    %the number of axial rectangles to draw on the cannula surface

tcirc = 0:2*pi/radialPoints:2*pi; %an array radialPoints long going from 0 to 2pi for calculating circles

for j=1:2
    for i=1:ndisks(j,1)
        R_rot = [diskRotation(l,1:3);diskRotation(l,4:6);diskRotation(l,7:9)];
        basecirc_low = [diskRadius(j,1)*sin(tcirc);diskRadius(j,1)*cos(tcirc);zeros(1,length(tcirc))];
        basecirc_up  = [diskRadius(j,1)*sin(tcirc);diskRadius(j,1)*cos(tcirc);diskHeight+zeros(1,length(tcirc))];
      
        basecirc_rot_low = R_rot*basecirc_low + repmat(diskPoints(l,1:3)',1,radialPoints+1);
        basecirc_rot_up = R_rot*basecirc_up + repmat(diskPoints(l,1:3)',1,radialPoints+1);
        
        fill3(basecirc_rot_low(1,:),basecirc_rot_low(2,:),basecirc_rot_low(3,:),color);
        fill3(basecirc_rot_up(1,:),basecirc_rot_up(2,:),basecirc_rot_up(3,:),color);
        
        x=[basecirc_rot_low(1,1:end); basecirc_rot_up(1,1:end)];
        y=[basecirc_rot_low(2,1:end); basecirc_rot_up(2,1:end)];
        z=[basecirc_rot_low(3,1:end); basecirc_rot_up(3,1:end)];
        
        %draw cylinder shell
        surf(x,y,z,'FaceColor',color,'MeshStyle','row');
        l = l + 1;
    end
end

%% plot backbone + tendons %%

l = l - sum(ndisks(:,1));
for i=1:sum(ndisks(:,1))-1
    x = [diskPoints(l,1) diskPoints(l+1,1)];
    y = [diskPoints(l,2) diskPoints(l+1,2)];
    z = [diskPoints(l,3) diskPoints(l+1,3)];
    plot3(x,y,z,'LineWidth',3,'Color',[28/255 28/255 28/255]);
    x = [diskPoints(l,4) diskPoints(l+1,4)];
    y = [diskPoints(l,5) diskPoints(l+1,5)];
    z = [diskPoints(l,6) diskPoints(l+1,6)];
    plot3(x,y,z,'LineWidth',1.1,'Color',[1 0 0]); % red for tendon 1
    x = [diskPoints(l,7) diskPoints(l+1,7)];
    y = [diskPoints(l,8) diskPoints(l+1,8)];
    z = [diskPoints(l,9) diskPoints(l+1,9)];
    plot3(x,y,z,'LineWidth',1.1,'Color',[0 1 0]); % green  for tendon 2
    x = [diskPoints(l,10) diskPoints(l+1,10)];
    y = [diskPoints(l,11) diskPoints(l+1,11)];
    z = [diskPoints(l,12) diskPoints(l+1,12)];
    plot3(x,y,z,'LineWidth',1.1,'Color',[0 0 1]); % blue  for tendon 3
    
    % plot tendons of the 2nd segment through the 1st segment
    if i <= ndisks(1,1)-1    
        for m=1:3
            vec_1 = -diskPoints(l,1:3) + diskPoints(l,m+3+(m-1)*2:m+5+(m-1)*2);
            vec_2 = -diskPoints(l+1,1:3) + diskPoints(l+1,m+3+(m-1)*2:m+5+(m-1)*2);
            vec_norm_1 = vec_1/norm(vec_1);
            vec_norm_2 = vec_2/norm(vec_2);
            
            p_1_2nd = diskPoints(l,1:3) + vec_norm_1 * diskPitchRadius(2,1);
            p_2_2nd = diskPoints(l+1,1:3) + vec_norm_2 * diskPitchRadius(2,1);
            x = [p_1_2nd(1,1) p_2_2nd(1,1)];
            y = [p_1_2nd(1,2) p_2_2nd(1,2)];
            z = [p_1_2nd(1,3) p_2_2nd(1,3)];
            plot3(x,y,z,'LineWidth',1.1,'Color',[0 0 0]); % inside tendon on segment 1 black
            if i == ndisks(1,1)-1
                diskPoints(l+1,m+3+(m-1)*2:m+5+(m-1)*2) = p_2_2nd;
            end
        end
    end
    l = l + 1;
end
axis equal;
grid on;
xlabel('x-axis [mm]');
ylabel('y-axis [mm]');
zlabel('z-axis [mm]');
view([-20 14]);
end

