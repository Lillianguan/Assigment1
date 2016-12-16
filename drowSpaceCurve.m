
%% drow messured_x   computed_x
figure();
hold on; 

for j=1:19
    x_c=[computed_x(j,1) computed_x(j+1,1)];
    y_c=[computed_y(j,1) computed_y(j+1,1)];
    z_c=[computed_z(j,1) computed_z(j+1,1)];
    x_m=[messured_x(j,1) messured_x(j+1,1)];
    y_m=[messured_y(j,1) messured_y(j+1,1)];
    z_m=[messured_z(j,1) messured_z(j+1,1)];
    plot3(x_c,y_c,z_c,'LineWidth',3,'Color',[1 0 0]); % red for computed spacecurve
    plot3(x_m,y_m,z_m,'LineWidth',3,'Color',[0 0 1]); % blue for messured spacecurve
end
% axis equal;
grid on;
legend('computed spacecurve','messured spacecurve')
title('Spacecurve for Configuration 3');
xlabel('x-axis [mm]');
ylabel('y-axis [mm]');
zlabel('z-axis [mm]');
view([-20 14]);
doc saveas
filename='configuration_3.jpg';
saveas(gcf,filename);