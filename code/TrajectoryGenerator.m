function cumulatedTraj = TrajectoryGenerator(TseInitial, TscInitial, Tscfinal, Tcegrasp, TceStandoff, k)
% Takes Tseinitial: The initial configuration of the end-effector in the reference trajectory
%       TscInitial: The cube's initial configuration
%       Tscfinal: The cube's desired final configuration
%       Tcegrasp: The end-effector's configuration relative to the cube when it is grasping the cube
%       TceStandoff:  The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
%       k: The number of trajectory reference configurations per 0.01 seconds
% Returns a csv file showing representing the trajectory of the
% end-effector frame.  The csv file is called output.csv and will be stored
% in the same directory
% Example Input:
% 
% clear; clc;
% TseInitial = [0, 0, 1, 0.5518;0, 1, 0, 0;-1, 0, 0, 0.4012;0, 0, 0, 1];
% TscInitial = [1, 0, 0, 1;0, 1, 0, 0;0, 0, 1, 0.025;0, 0, 0, 1];
% Tscfinal = [0, 1, 0, 0;-1, 0, 0, -1;0, 0, 1, 0.025;0, 0, 0, 1];
% Tcegrasp = [0, 0, 1, 0;0, 1, 0, 0;-1, 0, 0, 0;0, 0, 0, 1];
% TceStandoff = Tcegrasp;
% k = 1;
% TrajectoryGenerator(TseInitial, TscInitial, Tscfinal, Tcegrasp, TceStandoff, k);
% Output:
%   'TrajectoryGenerator.csv' file 
%    cumulatedTraj
%
% This function uses ScrewTrajectory function to calculate the 8 reference trajectorys
% for the gripper of the robot.  The gripper has the following 8 trajectorys.
% 1. A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
% 2. A trajectory to move the gripper down to the grasp position.
% 3. A trajectory to close the gripper.
% 4. A trajectory to move the gripper back up to the "standoff" configuration.
% 5. A trajectory to move the gripper to a "standoff" configuration above the final configuration.
% 6. A trajectory to move the gripper to the final configuration of the object.
% 7. A trajectory to open the gripper.
% 8. A trajectory to move the gripper back to the "standoff" configuration. 
cumulatedTraj = [];

TscStandoff1 = TscInitial;
TscStandoff1(3, 4) =  TscInitial(3, 4) + 0.200;

TscStandoff2 = Tscfinal;
TscStandoff2(3, 4) =  Tscfinal(3, 4) + 0.200;

TseStandoff1 = TscStandoff1 * TceStandoff;
TseStandoff2 = TscStandoff2 * TceStandoff;

Tsegrasp1 = TscInitial * Tcegrasp;
Tsegrasp2 = Tscfinal * Tcegrasp;

traj1 = ScrewTrajectory(TseInitial, TseStandoff1, 4, 400 * k, 3);
traj2 = ScrewTrajectory(TseStandoff1, Tsegrasp1, 1, 100 * k, 3);
traj4 = ScrewTrajectory(Tsegrasp1, TseStandoff1, 1, 100 * k, 3);
traj5 = ScrewTrajectory(TseStandoff1, TseStandoff2, 4, 400 * k, 3);
traj6 = ScrewTrajectory(TseStandoff2, Tsegrasp2, 1, 100 * k, 3);
traj8 = ScrewTrajectory(Tsegrasp2, TseStandoff2, 1, 100 * k, 3);

for i = 1:length(traj1)
    
    T = cell2mat(traj1(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 0];
    cumulatedTraj = [cumulatedTraj; r];
    
end

for i = 1:length(traj2)
    
    T = cell2mat(traj2(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 0];
    cumulatedTraj = [cumulatedTraj; r];
    
end

for i = 1:64
    T = Tsegrasp1;
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 1];
    cumulatedTraj = [cumulatedTraj; r];
end    

for i = 1:length(traj4)
    
    T = cell2mat(traj4(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 1];
    cumulatedTraj = [cumulatedTraj; r];
    
end

for i = 1:length(traj5)
    
    T = cell2mat(traj5(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 1];
    cumulatedTraj = [cumulatedTraj; r];
    
end

for i = 1:length(traj6)
    
    T = cell2mat(traj6(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 1];
    cumulatedTraj = [cumulatedTraj; r];
    
end

for i = 1:64
    T = Tsegrasp2;
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 0];
    cumulatedTraj = [cumulatedTraj; r];
end    

for i = 1:length(traj8)
    
    T = cell2mat(traj8(i));
    r = [T(1,1), T(1,2), T(1,3), T(2,1), T(2,2), T(2,3), T(3,1), T(3,2), T(3,3), T(1,4), T(2,4), T(3,4), 0];
    cumulatedTraj = [cumulatedTraj; r];
    
end

writematrix(cumulatedTraj, 'TrajectoryGenerator.csv')


end

