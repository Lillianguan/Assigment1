% This script generates a struct TACR (Tendon Actuated Continuum Robot)
% which contains specific parameters of the robot, it executes the geometric
% model and includes a function which can be used to visualize the robot.
%
% Copyright: 2016 Leibniz Universität Hannover, All rights reserved
%
% Email: continuumrobotics@lkr.uni-hannover.de
%
% Version: 1
% Date: 11/16/2016
%
 
% 2 segment tendon actuated continuum robot
TACR.ndisks = [10;10];               % number of disks per segment
TACR.diskRadius = [8;8];            % disk radius
TACR.diskHeight = 3;                        % heigth of the disks
TACR.diskPitchRadius = [6.5;5];        % pitch circle radius of disks
TACR.segmentLength = [92;102];      % segment length = length of first backbone per segment
 
q = [1.4, -2.2, 0.8; 1.6, 0.8, -2.4];           % actuation parameters (delta l per tendon); 
                                    % Remember: only 2 tendons can be
                                    % retracted at once, the 3rd tendon has
                                    % to extend
 
% compute robot's space curve                                    
robotShape = GeometricModel(TACR,q); 
 
% visualize the robot
drawRobot(robotShape,TACR);

