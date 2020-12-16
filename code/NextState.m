function newConfig = NextState(currentConfig, jointWheelSpeed, dt, maxSpeed)
%       Takes currentConfig: A 12-vector representing the current configuration 
% of the robot (3 variables for the chassis configuration, 5 variables for
% the arm configuration, and 4 variables for the wheel angles). 
%       jointWheelSpeed: A 9-vector of controls indicating the arm joint 
%speeds thetadot (5 variables) and the wheel speeds u (4 variables). 
%       dt: A timestep Δt. 
%       maxSpeed: A positive real value indicating the maximum angular 
%speed of the arm joints and the wheels.
%       
% Returns A 12-vector representing the configuration of the robot time Δt later. 

% Example Input:
% 
% currentConfig = [0;0;0;0;0;0;0;0;0;0;0;0];
% newMatrix = [currentConfig];
% jointWheelSpeed = [0;0;0; 0; 0; 10; 10; 10; 10];
% dt = 0.01;
% maxspeed = 5;
% for i = 1:100
%     newConfig = NextState(currentConfig, jointWheelSpeed, dt, maxspeed);
%     newMatrix = [newMatrix, newConfig];
%     currentConfig = newConfig;
% end    
% 
% writematrix(newMatrix', 'NextState.csv')
% Output:
%   newConfig
%   The resulting csv file and newMatrix variable showing that the robot chassis should drive 
%   forward (in the +xbhat direction) by 0.475 meters. 
%
% The function NextState is based on a simple first-order Euler step, i.e.,
%
% new arm joint angles = (old arm joint angles) + (joint speeds) * Δt
% new wheel angles = (old wheel angles) + (wheel speeds) * Δt
% new chassis configuration is obtained from odometry, as described in Chapter 13.4 

changingConfig = currentConfig;
r = 0.0475;
l = 0.47/2;
w = 0.3/2;

%for i = 1:100
oldjointangle = changingConfig(4:8);
oldwheelangle = changingConfig(9:12);
oldchassisConfig = changingConfig(1:3);
   
jointspeed = jointWheelSpeed(1:5);
for j = 1:5
  if jointspeed(j) > maxSpeed
      jointspeed(j) = maxSpeed;
  elseif jointspeed(j) < -maxSpeed
      jointspeed(j) = -maxSpeed;
  end    
end    
   
   
wheelspeed = jointWheelSpeed(6:9);
   
for j = 1:4
  if wheelspeed(j) > maxSpeed
      wheelspeed(j) = maxSpeed;
  elseif wheelspeed(j) < -maxSpeed
      wheelspeed(j) = -maxSpeed;
  end    
end   
   
newjointangle = oldjointangle + jointspeed * dt;
newwheelangle = oldwheelangle + wheelspeed * dt;
   
F = r/4 * [-1/(l+w), 1/(l+w) , 1/(l+w) -1/(l+w); 1, 1, 1, 1;-1, 1, -1, 1];
deltatheta = wheelspeed * dt;
   
newchassisConfig = oldchassisConfig + F * deltatheta;
   
newConfig = [newchassisConfig; newjointangle; newwheelangle];
%newMatrix = [newMatrix, newConfig];
%changingConfig = newConfig;
    
%end

%writematrix(newMatrix', 'NextState.csv')
