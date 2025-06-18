% *Header for 3128*

% Imma A. Student
% ASEN 3128
% FileName.m
% Created: 1/24/2023

function wind_angles = AirRelativeVelocityVectorToWindAngles(velocity_body) %
% Inputs:   velocity_body = column vector of aircraft air-relative velocity in body coordinates
%                         = [u,v,w] 
%
%
% Outputs:  wind_angles = [speed beta alpha] 
%           speed = aircraft airspeed
%           beta = side slip angle
%           alpha = angle of attack
%
% Methodology: Use definitions to calculate wind angles and speed from velocity vector

wind_angles = velocity_body;

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
% *Header for 3111*

%% ASEN 3111 - Computational Assignment XX - Main
% Provide a brief summary of the problem statement and code so that you or
% someone else can later understand what you attempted to do, doesn't have
% to be this long
%
%   Author: {Your Name}
%   Collaborators: J. Doe, J. Smith {acknowledge who you worked with}
%   Date: {date last revised}
%
% Start Code
clc; clear all; close all;

% Loading In Data
% Note: this one overwrites T everytime. I got around it this time with an if/elseif 
% loop
files = dir('Data/');
for temp = 4:7
    long_name = strcat(files(temp).folder,'/',files(temp).name);
    T = readtable(long_name);
end
%% 
% Copying Earlier Graphs to a New Figure
% This copies the exact values seen on the earlier graph to a new one. 
saveas(f1, 'temp.fig')
f2 = openfig('temp.fig');
delete('temp.fig')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
% Printing Graphs
% Write one of these for each or so. Will be trying to figure out how to get 
% it to go to a folder but not today
print(f1,'-dpng','Trial1.png');

%% 
% For multiple plots all at once at the end of a function
nme = ["nameofplot.png",...];
for k = 1:numofplots
    print(k,'-dpng',nme(k));
end

%%
% Making Titles and File Names Using Values From Earlier Code
% This absolutely does not work with latex formatting for some fucking reason 
% which is a pain in the ass.

titlestr = "Problem #: Value 1=%.f, Value 2=%.f";
titlestr = compose(titlestr,Value1,Value2);
pngstr = "Problem#_Value1%.f_Value2%.f";
pngstr = compose(pngstr,Value1,Value2) + ".png";



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Function Header
% Since you want consistency even though half the time you are so busy you don't use it.
% 
% Function Name: 
% Function Description
%
% Author: Alia Feltes-DeYapp
% Collaborators: 
% Date Last Modified: 
% 
% Inputs:   a       = 
%           b       = 
%
% Outputs:  c       = 
%           d       = 
% 
% Methodology:  



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
% Color Gradient of Lines Based On an Outside Variable
% Is this almost excessively specific? Yes. However, when you need it, it's 
% nicer to have this then have to spend upwards of 3 hours looking for this exact 
% concept.

m = 11; % number of potential options
colorgradVol = turbo(m); % sets the color for each option, size [m x 4]

% creates an array that can be used to compare an external variable to the
% options. This is used to determine the index that will be called for the
% specified color.
gradDef = (20:1:30); % size [m x 1]

% calculations and such
n = randi([20,30]); % your external variable
colors = colorgradVol(find(n == gradDef),:); % colors = [R G B A]

% plotting
plot(x, y, '.','markersize',6,'MarkerFaceColor',colors)
plot3(x, y, z,'Color',colors)

% 
% Rotation Matrix Function
% This is just useful overall. I use it for homework a lot. (Also I should've 
% written code for this last semester but didn't for some reason)

function[result,verifyMat] = RotationMatrixFunc(angle1,angle2,angle3,rot1,rot2,rot3)
% Inputs:   angle1 = angle about first axis of rotation     {deg}
%           angle2 = angle about second axis of rotation    {deg}
%           angle3 = angle about third axis of rotation     {deg}
%           rot1 = define the which axis is rotated about first
%           rot2 = define the which axis is rotated about second
%           rot3 = define the which axis is rotated about third
%
% Outputs:  result = 3x3 rotation matrix of numerical values
%           verifyMat = debugging matrix of symbolic functions
% 
% Methodology:  create a rotation matrix by multiplying together 
%               3 single-axis rotation matrices

angle1 = deg2rad(angle1);
angle2 = deg2rad(angle2);
angle3 = deg2rad(angle3);

syms t
M(:,:,1) = [1,0,0; 0,cos(t),sin(t); 0,-sin(t),cos(t)];
M(:,:,2) = [cos(t),0,-sin(t); 0,1,0; sin(t),0,cos(t)];
M(:,:,3) = [cos(t),sin(t),0; -sin(t),cos(t),0; 0,0,1];

step = subs(M(:,:,rot2),angle2) * subs(M(:,:,rot1),angle1);
verifyMat = subs(M(:,:,rot3),angle3) * step;
result = double(verifyMat);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
% Plot Aircraft EOM Function
% I've gotten tired of writing out the plots over and over again for 3128 labs 
% so I made a function that will just plot them for me. Might modify this so that 
% it'll also print them out afterwards

function PlotAircraftEOM(titles,figureNo,time,X)
% PlotAircraftEOM.m
% Author: Alia Feltes-DeYapp
% Collaborators: N/A
% Date Last Modified: Feb 14, 2023
% INPUTS:   titles  = vector of length 5 with string values
%                   = [title1 title2 title3 title4 title5]
%           figureNo    = vector of length 5 with double values for
%                       numbered figures
%                       = [1 2 3 4 5]
%           time    = time vector determined via ODE45 or experimental data
%           X   = state variable matrix
%               = [XE  YE  ZE  Psi  Theta  Phi  uE  vE  wE  p  q  r]
% 
% OUTPUTS:  figure(1)   = 3D plot of a/c flight path
%           figure(2)   = [3] 2D subplots of a/c inertial position vector
%                       components over time
%           figure(3)   = [3] 2D subplots of a/c Euler angles over time
%           figure(4)   = [3] 2D subplots of a/c inertial velocity vector
%                       components over time
%           figure(5)   = [3] 2D subplots of a/c angular velocity vector
%                       components over time
% 
% NOTES:    These plots are made oriented with the negative z in facing
%           upwards. This is for clarity in reading and analyzing the
%           graphs and has been noted on each plot that this applies to.


x = X(:,1); y = X(:,2); z = X(:,3);
psi = X(:,4); theta = X(:,5); phi = X(:,6);
uE = X(:,7); vE = X(:,8); wE = X(:,9);
p = X(:,10); q = X(:,11); r = X(:,12);

% figure(1)
figure(figureNo(1)); hold on; grid on;
plot3(x,y,z); plot3(x(1),y(1),z(1),'*k'); plot3(x(end),y(end),z(end),'*r')
set(gca,'zdir','reverse'); view(105,-2);
title(titles(1)); xlabel('x-position [m]'); ylabel('y-position [m]'); zlabel('z-position (WRT to orientation of NEZ frame) [m]')
hold off;

% figure(2)
figure(figureNo(2)); hold on; grid on;
sgtitle(titles(2))
    subplot(3,1,1);
    plot(time,x)
    title('x-position'); xlabel('time [s]'); ylabel('x-position [m]')

    subplot(3,1,2); 
    plot(time,y)
    title('y-position'); xlabel('time [s]'); ylabel('y-position [m]')

    
    subplot(3,1,3);
    plot(time,z); set(gca,'ydir','reverse')
    title('z-position (WRT to orientation of NEZ frame)'); xlabel('time [s]'); ylabel('z-position [m]')
hold off;

% figure(3)
figure(figureNo(3)); hold on; grid on;
sgtitle(titles(3))
    subplot(3,1,1);
    plot(time,psi)
    title('\psi'); xlabel('time [s]'); ylabel('\psi [rad]')

    subplot(3,1,2); 
    plot(time,theta)
    title('\theta'); xlabel('time [s]'); ylabel('\theta [rad]')
    
    subplot(3,1,3);
    plot(time,phi); set(gca,'ydir','reverse')
    title('\phi (WRT to orientation of NEZ frame)'); xlabel('time [s]'); ylabel('\phi [rad]')

% figure(4)
figure(figureNo(4)); hold on; grid on;
sgtitle(titles(4))
    subplot(3,1,1);
    plot(time,uE)
    title('u_E'); xlabel('time [s]'); ylabel('u_E [m/s]')

    subplot(3,1,2); 
    plot(time,vE)
    title('v_E'); xlabel('time [s]'); ylabel('v_E [m/s]')
    
    subplot(3,1,3);
    plot(time,wE); set(gca,'ydir','reverse')
    title('w_E (WRT to orientation of NEZ frame)'); xlabel('time [s]'); ylabel('w_E [m/s]')

% figure(5)
figure(figureNo(5)); hold on; grid on;
sgtitle(titles(5))
    subplot(3,1,1);
    plot(time,p)
    title('p'); xlabel('time [s]'); ylabel('p [rad/s]')

    subplot(3,1,2); 
    plot(time,q)
    title('q'); xlabel('time [s]'); ylabel('q [rad/s]')
    
    subplot(3,1,3);
    plot(time,r); set(gca,'ydir','reverse')
    title('r (WRT to orientation of NEZ frame)'); xlabel('time [s]'); ylabel('r [m/s]')