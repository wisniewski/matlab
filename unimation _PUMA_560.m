%% Documentation
% Robotic Arm Unimation PUMA 560
% Project by Mariusz Wisniewski
% Software: MatLab R2011a
% E-mail: 254019@stud.umk.pl
% website: www.mariuszbonifacy.pl
%
% Forward & Inverse kinematics calculated in MatLab, based on book
% "Introduction to Robotics. Machanics and Control" by John Craig (3rd
% edition)
%
% Code based on lab exercises, www.sky-engin.jp (figures and animations)
%
% Other educational materials: lecture by PhD Slawomir Mandra and Internet
%%
function main
%% init program
initiation
%% main body
[x,y,z] = calculate_letter;
draw_letter_blue(x,y,z)
tic;pause(1);toc;cla; %just wait for a while and clear everything
plot3(x,y,z,'.r') %draw letter after clearing
%% ending
view(-150, 10) %change point of view (angle)
forward_kinematics(90,-90,-90,0,0,0) %neutral/home position
end
%%
function initiation
%% erase everything in MatLab
close all
clear all
clc
%% open figure window and entry configuration
figure('units','normalized','outerposition',[0 0 1 1]); %automatically maximize figure
xlabel('x'),ylabel('y'),zlabel('z');
title('Unimation PUMA 560');
axis([-15 15 -15 15 -7 15]);
view(-35, 10) %starting point of view (angle)
daspect([1 1 1]) %block resizing
grid on
hold on
end
%%
function draw_coordinate_system(H, name)
%% description
% X - blue , Y - green, Z - red
% cs - starting coordinate system (vectors)
% column 1 - start of coordinate system
% column 3 - end of axe Y
% column 4 - end
%% calculate CS
%    p0 x y z
cs=[0 1 0 0;
    0 0 1 0;
    0 0 0 1;
    1 1 1 1];
cs=H*cs;
%% draw CS
plot3([cs(1,1) cs(1,2)], [cs(2,1) cs(2,2)], [cs(3,1) cs(3,2)],'b', 'LineWidth', 2)
plot3([cs(1,1) cs(1,3)], [cs(2,1) cs(2,3)], [cs(3,1) cs(3,3)],'g', 'LineWidth', 2)
plot3([cs(1,1) cs(1,4)], [cs(2,1) cs(2,4)], [cs(3,1) cs(3,4)],'r', 'LineWidth', 2)
text(cs(1,1), cs(2,1), cs(3,1), name, 'FontSize', 20)
end
%%
function base00(H)
%% dimensions
radius = 2;
height = -6;
side_count = 26;
%% vertices
n_side = side_count;
for i_ver=1:n_side
    VertexData_0(i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),0];
    VertexData_0(n_side+i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),height];
end
n_ver = 2*n_side;
%% DH matrix - moves
R = H(1:3,1:3);
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];

for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% side patches
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];
%% side patches data
for i_pat=1:n_side
    PatchDate1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchDate1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchDate1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end
%% draw side patches
h1 = patch(PatchDate1_X,PatchDate1_Y,PatchDate1_Z,'y');
set(h1,'FaceLighting','phong','EdgeLighting','phong');
set(h1,'EraseMode','normal');
%% bottom Patches
Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];
%% bottom patches data
for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end
%% draw bottom patches
h2 = patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'y');
set(h2,'FaceLighting','phong','EdgeLighting','phong');
set(h2,'EraseMode','normal');
end
%%
function base01(H)
%% dimensions
radius = 2;
height = 2;
side_count = 20;
%% vertices
n_side = side_count;
for i_ver=1:n_side
    VertexData_0(i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),0];
    VertexData_0(n_side+i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),height];
end
n_ver = 2*n_side;
%% DH matrix - moves
R = H(1:3,1:3);
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];

for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% side patches
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];
%% side patches data
for i_pat=1:n_side
    PatchDate1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchDate1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchDate1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end
%% draw side patches
h1 = patch(PatchDate1_X,PatchDate1_Y,PatchDate1_Z,'y');
set(h1,'FaceLighting','phong','EdgeLighting','phong');
set(h1,'EraseMode','normal');
%% bottom Patches
Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];
%% bottom patches data
for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end
%% draw bottom patches
h2 = patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'y');
set(h2,'FaceLighting','phong','EdgeLighting','phong');
set(h2,'EraseMode','normal');
end
%%
function arm02(H)
%% dimensions
Lx = 6;
Ly = -2;
Lz = 2;
%% data that we need from DH matrix to rotate block
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];
R = H(1:3,1:3);
%% vertices
VertexData_0 = [Lx*ones(8,1),Ly*ones(8,1),Lz*ones(8,1)]...
    .*[0,0,0;
    1,0,0;
    0,1,0;
    0,0,1;
    1,1,0;
    0,1,1;
    1,0,1;
    1,1,1];

n_ver = 8;
for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% patches
Index_Patch = ...
    [1,2,5,3;
    1,3,6,4;
    1,4,7,2;
    4,7,8,6;
    2,5,8,7;
    3,6,8,5];
%% patches data
n_pat = 6;
for i_pat=1:n_pat
    
    PatchData_X(:,i_pat) = VertexData(Index_Patch(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(Index_Patch(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(Index_Patch(i_pat,:),3);
end
%% draw patches
figure(1);
h = patch(PatchData_X,PatchData_Y,PatchData_Z,'y');
set(h,'FaceLighting','phong','EdgeLighting','phong');
set(h,'EraseMode','normal');
end
%%
function arm03(H)
%% dimensions
Lx = 2;
Ly = 5;
Lz = 2;
%% data that we need from DH matrix to rotate block
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];
R = H(1:3,1:3);
%% vertices
VertexData_0 = [Lx*ones(8,1),Ly*ones(8,1),Lz*ones(8,1)]...
    .*[0,0,0;
    1,0,0;
    0,1,0;
    0,0,1;
    1,1,0;
    0,1,1;
    1,0,1;
    1,1,1];

n_ver = 8;
for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% patches
Index_Patch = ...
    [1,2,5,3;
    1,3,6,4;
    1,4,7,2;
    4,7,8,6;
    2,5,8,7;
    3,6,8,5];
%% patches data
n_pat = 6;
for i_pat=1:n_pat
    
    PatchData_X(:,i_pat) = VertexData(Index_Patch(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(Index_Patch(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(Index_Patch(i_pat,:),3);
end
%% draw patches
figure(1);
h = patch(PatchData_X,PatchData_Y,PatchData_Z,'y');
set(h,'FaceLighting','phong','EdgeLighting','phong');
set(h,'EraseMode','normal');
end
%%
function arm04(H)
%% dimensions
radius = 0.4;
height = -1;
side_count = 20;
%% vertices
n_side = side_count;
for i_ver=1:n_side
    VertexData_0(i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),0];
    VertexData_0(n_side+i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),height];
end
n_ver = 2*n_side;
%% DH matrix - moves
R = H(1:3,1:3);
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];

for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% side patches
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];
%% side patches data
for i_pat=1:n_side
    PatchDate1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchDate1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchDate1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end
%% draw side patches
h1 = patch(PatchDate1_X,PatchDate1_Y,PatchDate1_Z,'y');
set(h1,'FaceLighting','phong','EdgeLighting','phong');
set(h1,'EraseMode','normal');
%% bottom Patches
Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];
%% bottom patches data
for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end
%% draw bottom patches
h2 = patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'y');
set(h2,'FaceLighting','phong','EdgeLighting','phong');
set(h2,'EraseMode','normal');
end
%%
function arm05(H)
%% dimensions
radius = 0.1;
height = 0.5;
side_count = 20;
%% vertices
n_side = side_count;
for i_ver=1:n_side
    VertexData_0(i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),0];
    VertexData_0(n_side+i_ver,:) = [radius*cos(2*pi/n_side*i_ver),radius*sin(2*pi/n_side*i_ver),height];
end
n_ver = 2*n_side;
%% DH matrix - moves
R = H(1:3,1:3);
x = H(1,4);
y = H(2,4);
z = H(3,4);
v = [x y z];

for i_ver=1:n_ver
    VertexData(i_ver,:) = v + VertexData_0(i_ver,:)*R';
end
%% side patches
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];
%% side patches data
for i_pat=1:n_side
    PatchDate1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchDate1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchDate1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end
%% draw side patches
h1 = patch(PatchDate1_X,PatchDate1_Y,PatchDate1_Z,'y');
set(h1,'FaceLighting','phong','EdgeLighting','phong');
set(h1,'EraseMode','normal');
%% bottom Patches
Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];
%% bottom patches data
for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end
%% draw bottom patches
h2 = patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'y');
set(h2,'FaceLighting','phong','EdgeLighting','phong');
set(h2,'EraseMode','normal');
end
%%
function effector06(H)
%% vertices of effector
vertices = [0.40 0.15 0 1
    0.40 -0.15 0 1
    -0.40 -0.15 0 1
    -0.40 0.15 0 1
    0.40 -0.15 0.40 1
    0.40 0.15 0.40 1
    -0.40 0.15 0.40 1
    -0.40 -0.15 0.40 1];
vertices = (H*vertices')';
%% faces of effector
faces = [1 2 3 4
    1 2 5 6
    3 4 7 8];
%% draw our effector
patch('Vertices',vertices(:,1:3),'Faces',faces)
end
%%
function forward_kinematics(theta1,theta2,theta3,theta4,theta5,theta6)
%% unit vectors
x = [1 0 0]'; % I do NOT use "y" unit vector
z = [0 0 1]';
%% data
a2 = 4;
e3 = 2;
e4 = 6;
e6 = 0;
%% calculate forward kinematics
h00 = eye(4);
h01 = h00 * rotation(z,theta1);
h02 = h01 * rotation(x,-90)*rotation(z,theta2);
h03 = h02 * transformation(a2,0,0)*rotation(z,theta3)*transformation(0,0,e3);
h04 = h03 * rotation(x,-90)*rotation(z,theta4)*transformation(0,0,e4);
h05 = h04 * rotation(x,90)*rotation(z,theta5);
h06 = h05 * rotation(x,-90)*rotation(z,theta6)*transformation(0,0,e6);
%% craw all coordinate systems
draw_coordinate_system(h01, '1');
draw_coordinate_system(h02, '2');
draw_coordinate_system(h03, '3');
draw_coordinate_system(h04, '4');
draw_coordinate_system(h05, '5');
draw_coordinate_system(h06, '6');
%% draw our figures (base, arm, effector)
base00(h00);
base01(h01);
arm02(h02);
arm03(h03);
arm04(h04);
arm05(h05)
effector06(h06);
end
%%
function [theta1,theta2,theta3,theta4,theta5,theta6] = inverse_kinematics(point_x,point_y,point_z)
%% dimensions and initialize angle variables to zero
angle_case = 0;
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;
a2 = 4;
a3 = 0;
e3 = 2;
d4 = 6;
c4 = cos(theta4);
c5 = cos(theta5);
c6 = cos(theta6);
s4 = sin(theta4);
s5 = sin(theta5);
s6 = sin(theta6);
px = point_x;
py = point_y;
pz = point_z;
%% in mathematical models sqrt theta and theta3 and be + or - so I need to check all four cases ale choose BEST
while angle_case == 0;
    for i = 1:1:4
        if i == 1
            theta1_sign = 1;
            theta3_sign = 1;
        elseif i == 2
            theta1_sign = 1;
            theta3_sign = -1;
        elseif i == 3
            theta1_sign = -1;
            theta3_sign = 1;
        else
            theta1_sign = -1;
            theta3_sign = -1;
        end
        %% stop choosing case and start caculate angles; first theta1
        D = (px^2+py^2+pz^2-a2^2-a3^2-e3^2-d4^2)/(2*a2);
        theta1 = (atan2(py,px)-atan2(e3,theta1_sign*sqrt(px^2+py^2-e3^2)));
        %% calc theta3
        c1 = cos(theta1);
        s1 = sin(theta1);
        theta3 = (atan2(a3,d4)-atan2(D,theta3_sign*sqrt(a3^2+d4^2-D^2)));
        %% calc theta2
        c3 = cos(theta3);
        s3 = sin(theta3);
        t23 = atan2((-a3-a2*c3)*pz-(c1*px+s1*py)*(d4-a2*s3),(a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py));
        theta2 = (t23 - theta3);
        %% calc theta4
        s23 = ((-a3-a2*c3)*pz+(c1*px+s1*py)*(a2*s3-d4))/(pz^2+(c1*px+s1*py)^2);
        c23 = ((a2*s3-d4)*pz+(a3+a2*c3)*(c1*px+s1*py))/(pz^2+(c1*px+s1*py)^2);
        r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
        r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
        r33 = s23*c4*s5 - c23*c5;
        theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
        %% calc theta5
        r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
        r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
        r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
        s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
        c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
        theta5 = atan2(s5,c5);
        %% calc theta6
        s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
        c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
        theta6 = atan2(s6,c6);
        %% convert to degrees
        theta1 = theta1*180/pi;
        theta2 = theta2*180/pi;
        theta3 = theta3*180/pi;
        theta4 = theta4*180/pi;
        theta5 = theta5*180/pi;
        theta6 = theta6*180/pi;
        %% inverse theta2
        if theta2>=160 && theta2<=180
            theta2 = -theta2;
        end
        %% ending when we get optimal angle (one of four cases)
        if theta1<=160 && theta1>=-160 && (theta2<=20 && theta2>=-200) && theta3<=45 && theta3>=-225 && theta4<=266 && theta4>=-266 && theta5<=100 && theta5>=-100 && theta6<=266 && theta6>=-266
            angle_case = 1;
            theta3 = theta3+180;
            break
        end
        if i == 4 && angle_case == 0
            angle_case = 1;
            break
        end
    end
end
end
%%
function H = rotation(axis, theta)
%% calculate our rotation, sind - angle in degrees
c=@cosd;
s=@sind;
%% rotate matrix R
R = axis*axis'*(1-c(theta)) + [ c(theta)          -axis(3)*s(theta)   axis(2)*s(theta)
    axis(3)*s(theta)     c(theta)         -axis(1)*s(theta)
    -axis(2)*s(theta)    axis(1)*s(theta)    c(theta) ];
%% from a11 to a33 in H is R
H = eye(4);
H(1:3,1:3) = R;
end
%%
function H = transformation(x, y, z)
%% transformation matrix
H = [ 1 0 0 x
    0 1 0 y
    0 0 1 z
    0 0 0 1 ];
end
%%
function draw_letter_blue(x,y,z)
i = 0;
for t = 1:1:length(z)
    %% calculating actual points
    i=i+1;
    cla
    point_z = z(t);
    point_x = x(t);
    point_y = y(t);
    %% move our arm
    [theta1,theta2,theta3,theta4,theta5,theta6] = inverse_kinematics(point_x,point_y,point_z);
    forward_kinematics(theta1,theta2,theta3-180,0,0,0)
    %% calculating past points
    A(1,i)=point_x;
    B(1,i)=point_y;
    C(1,i)=point_z;
    %% draw points and manipulator
    hold on
    plot3(A,B,C,'.');
    drawnow
end
end
%%
function [x,y,z]=calculate_letter
%% vertical lines | |
z = 1:0.1:9;
for i=1:1:length(z)
    x(i) = 4;
    y(i) = 4;
end
%% data for diagonal lines \ /
for i=1:1:length(z)
    xa(i) = -4;
    ya(i) = 4;
end

z = [z z];
x = [x xa];
y = [y ya];

xb = -4:0.1:4;
x = [x xb];

for i=1:1:length(xb)
    yb(i) = 4;
end
y = [y yb];
%% finally data for whole letter |\/|
zb = linspace(9,5,40) % I want ony 40 elements from 9 to 5
zc = linspace(5,9,41) % and now I want 41 elements
z = [z zb zc]; %now I have 243 elements in [z], [y] & [x]
end