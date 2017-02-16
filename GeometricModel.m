function[robotShape] = GeometricModel(TACR,q)
%% GeometricModel.m
% This function computes the space curve of a tendon actuated continuum
% robot with 2 segments and utilizes a geometric forward kinematics model
%
% Copyright: 2016 Leibniz Universität Hannover, All rights reserved
%
% Email: continuumrobotics@lkr.uni-hannover.de
%
% Version: 1
% Date: 11/16/2016
%
% input: struct TACR, q:[jx3] configurational parameters
%
% output: robotShape.diskPoints: n rows for n disks, 12 columns for coordinates
%        (x,y,z) for central backbone, points tendon 1 (x,y,z), points tendon 2 (x,y,z),
%         points tendon 3 (x,y,z)
%         robotShape.diskRotation: n rows for n disks, columns represent
%         rotation matrices (3x3): columns 1-3 are matrix elements (1,1; 1,2; 1,3),
%         columns 4-6 are matrix elements (2,1; 2,2; 2,3),
%         columns 7-9 are matrix elements (3,1; 3,2; 3,3),


%% implement the geometric model here
ndisks = TACR.ndisks;
segmentLength=TACR.segmentLength;
diskPitchRadius = TACR.diskPitchRadius;

%define the Angel
beta=pi*2/3;
theta_0=pi/2;

%get segment number and tendons number
seg_num=length(TACR.ndisks);
tendons_num=length(q(1,:));

% Check the input right or not
offset_sum=0;
for a=1:seg_num
    for b=1:tendons_num
        offset_sum=offset_sum+q(a,b);
    end
    if(abs(offset_sum)~=0)
        msg = 'False Configuration,Please enter the right configuration: only 2 tendons can be retracted at once, the 3rd tendon has to extend';
        Disp(msg);
        break;
    end
    offset_sum=0;
end

% calculate the Phi
phi=zeros(seg_num,1);
phi_0=0;
for j=1:seg_num
    phi(j,1)=atan2(q(j,1)*cos(beta)-q(j,2),q(j,1)*sin(beta))+phi_0;
    phi_0=phi(j,1);
end

% calculate the Offset and position of tendons in endeffekt koordinatensystem
deta=zeros(seg_num,tendons_num);
r_e=zeros(seg_num,12);
r_e(:,1:3)=0;
for j=1:seg_num
    columns=4;
    for i=1:tendons_num
        % Offset
        deta(j,i)=diskPitchRadius(j,1)*cos(phi(j,1)+(i-1)*beta);
        % Position
        r_e(j,columns:columns+2)=diskPitchRadius(j,1)*[cos(phi(j,1)+(i-1)*beta) -sin(phi(j,1)+(i-1)*beta) 0];
        columns=columns+3;
    end
    columns=0;
end

r_ee=zeros(sum(ndisks(:,1)),12);
rows=1;
for h=1:sum(ndisks(:,1))
    r_ee(h,:)=r_e(rows,:);
    if h==ndisks(rows,1);
        rows=rows+1;
    end
end


% calculate the bending angle and calculate the curveture and disk-rotation-matrix
theta=zeros(seg_num,1);
k=zeros(seg_num,1);
theta_l=zeros(seg_num,ndisks(1,1));
r_be=zeros(sum(ndisks(:,1)),3);
robotShape.diskRotation=zeros(sum(ndisks(:,1)),9); % robotShape.diskRotation matrix
m=1; % disk counter
rot_y=zeros(3,3); %define the Elementdrehung, um y Achse
rot_z=zeros(3,3);%define the Elementdrehung, um z Achse
rot_=zeros(3,3); %define the rotation matrix, 3x3
for j=1:seg_num
    theta(j,1)=theta_0+q(j,1)/deta(j,1);
    k(j,1)=(pi/2-theta(j,1))/segmentLength(j,1);
    rot_z=[cos(phi(j,1)) -sin(phi(j,1)) 0 ;sin(phi(j,1)) cos(phi(j,1)) 0; 0 0 1];
    rot_zz{j,1}=inv([cos(phi(j,1)) -sin(phi(j,1)) 0 ;sin(phi(j,1)) cos(phi(j,1)) 0; 0 0 1]);
    for n=0:ndisks(j,1)-1;
        theta_l(j,n+1)=pi/2-n*k(j,1)*segmentLength(j,1)/(ndisks(j,1)-1;
        r_be(m,1:3)=1/k(j,1)*[1-sin(theta_l(j,n+1)) 0 cos(theta_l(j,n+1))];
        rot_angel=pi/2-theta_l(j,n+1);
        rot_y=[cos(rot_angel)  0 sin(rot_angel) ;0 1 0 ;-sin(rot_angel) 0 cos(rot_angel)];
        %         rot_y=[cos(theta_l(j,n+1))  0 sin(theta_l(j,n+1)) ;0 1 0 ;-sin(theta_l(j,n+1)) 0 cos(theta_l(j,n+1))];
        rot_=rot_y*rot_z; % rotation from basis to endeffekter
        rot_=inv(rot_);
        robotShape.diskRotation(m,:)=[rot_(1,1) rot_(1,2) rot_(1,3) rot_(2,1)  rot_(2,2) rot_(2,3) rot_(3,1)  rot_(3,2) rot_(3,3) ];
        m=m+1;
        rot_y=zeros(3,3);
        rot_=zeros(3,3);
    end
end

% calculate the robotShape.diskPoint
robotShape.diskPoints=zeros(sum(ndisks(:,1)),12);
% for h=1:sum(ndisks(:,1))
for h=1:10
    R_rot = [robotShape.diskRotation(h,1:3);robotShape.diskRotation(h,4:6);robotShape.diskRotation(h,7:9)];
    robotShape.diskPoints(h,1:3) =  r_be(h,1:3)+(R_rot*r_ee(h,1:3)')';
    robotShape.diskPoints(h,4:6) = r_be(h,1:3)+(R_rot*r_ee(h,4:6)')';
    robotShape.diskPoints(h,7:9) = r_be(h,1:3)+(R_rot*r_ee(h,7:9)')';
    robotShape.diskPoints(h,10:12) = r_be(h,1:3)+(R_rot*r_ee(h,10:12)')';
end
R_rot_seg1 = [robotShape.diskRotation(10,1:3);robotShape.diskRotation(10,4:6);robotShape.diskRotation(10,7:9)];
<<<<<<< HEAD
=======
R_rot_seg1=R_rot_seg1*rot_zz{1,1}';
>>>>>>> 94d27714305c826e3282567500291b04e0ded0f4
for h=11:20
    R_rot =[robotShape.diskRotation(h,1:3);robotShape.diskRotation(h,4:6);robotShape.diskRotation(h,7:9)];
%     robotShape.diskPoints(h,1:3) = robotShape.diskPoints(10,1:3)+(R_rot_seg1* rot_zz{1,1}*(r_be(h,1:3)+(R_rot*r_ee(h,1:3)')')')';
%     robotShape.diskPoints(h,4:6) = robotShape.diskPoints(10,4:6)+(R_rot_seg1*rot_zz{1,1}*(r_be(h,1:3)+(R_rot*r_ee(h,4:6)')')')';
%     robotShape.diskPoints(h,7:9) = robotShape.diskPoints(10,7:9)+(R_rot_seg1*rot_zz{1,1}*(r_be(h,1:3)+(R_rot*r_ee(h,7:9)')')')';
%     robotShape.diskPoints(h,10:12) = robotShape.diskPoints(10,10:12)+(R_rot_seg1*rot_zz{1,1}*(r_be(h,1:3)+(R_rot*r_ee(h,10:12)')')')';
    robotShape.diskPoints(h,1:3) = robotShape.diskPoints(10,1:3)+(R_rot_seg1* (r_be(h,1:3)+(R_rot*r_ee(h,1:3)')')')';
    robotShape.diskPoints(h,4:6) = robotShape.diskPoints(10,4:6)+(R_rot_seg1*(r_be(h,1:3)+(R_rot*r_ee(h,4:6)')')')';
    robotShape.diskPoints(h,7:9) = robotShape.diskPoints(10,7:9)+(R_rot_seg1*(r_be(h,1:3)+(R_rot*r_ee(h,7:9)')')')';
    robotShape.diskPoints(h,10:12) = robotShape.diskPoints(10,10:12)+(R_rot_seg1*(r_be(h,1:3)+(R_rot*r_ee(h,10:12)')')')';



end


