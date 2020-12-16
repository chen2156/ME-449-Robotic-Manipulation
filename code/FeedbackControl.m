function [result, Xerr] = FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, currentConfig)
% Takes Tse: The current actual end-effector configuration X (also written Tse). 
%       Tsed: The current end-effector reference configuration Xd (i.e., Tse,d). 
%       Tsednext: The end-effector reference configuration at the next 
%timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time Δt later. 
%       Kp and Ki: The PI gain matrices Kp and Kirespectively. 
%       dt:  The timestep Δt between reference trajectory configurations.
%       CurrentConfig: the current 13-vector configuration of the robot 
%
% Returns The commanded end-effector twist V expressed in the end-effector frame {e}. 
% Example Input:
% 
% clear; clc;
% CurrentConfig = [0;0;0;0;0;0.2;-1.6;0;0;0;0;0;0];
% Tse = [0.170, 0, 0.985, 0.387;0, 1, 0, 0;-0.985, 0, 0.170, 0.570; 0, 0, 0, 1];
% Tsed = [0, 0, 1, 0.5;0, 1, 0, 0;-1, 0, 0, 0.5;0, 0, 0, 1];
% Tsednext = [0, 0, 1, 0.6;0, 1, 0, 0;-1, 0, 0, 0.3;0, 0, 0, 1];
% Kp = eye(6);
% Ki = zeros(6);
% dt = 0.01;
% 
% global Xerrdtsum
% Xerrdtsum = zeros(6, 1);
% [result, Xerr] = FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, CurrentConfig)
%
% Output:
%   result, Xerr
%    
%
% To calculate the control law FeedbackControl, we need the current actual 
% end-effector configuration X(q,θ), a function of the chassis 
% configuration q and the arm configuration θ. The values (q,θ) come 
% directly from the simulation results (Milestone 1). In other words, assume perfect sensors.
%
% The error twist Xerr that takes X to Xd in unit time is extracted from the
% 4x4 se(3) matrix [Xerr] = log(X^(-1) * Xd). FeedbackControl also needs 
% to maintain an estimate of the integral of the error, e.g., by adding 
% XerrΔt to a running total at each timestep. The feedforward reference 
% twist Vd that takes Xd to Xd,next in time Δt is extracted from 
% [Vd] = (1/Δt) log(Xd^(-1)*Xd_next). 
%
%The output of FeedbackControl is the commanded end-effector twist V 
%expressed in the end-effector frame {e}. To turn this into commanded wheel
%and arm joint speeds (u,thetadot), we use the pseudoinverse of the mobile 
%manipulator Jacobian Je(θ), 
global Xerrdtsum

Vdhat = 1/dt * MatrixLog6(TransInv(Tsed) * Tsednext);
Vd = se3ToVec(Vdhat);
AdVd = Adjoint(TransInv(Tse) * Tsed) * Vd;
Xerrhat = MatrixLog6(TransInv(Tse) * Tsed);
Xerr = se3ToVec(Xerrhat);
Xerrdtsum = Xerrdtsum + Xerr * dt;
V = AdVd + Kp * Xerr + Ki * Xerrdtsum;

B1 = [0;0;1;0;0.033;0];
B2 = [0;-1;0;-0.5076;0;0];
B3 = [0;-1;0;-0.3526;0;0];
B4 = [0;-1;0;-0.2176;0;0];
B5 = [0;0;1;0;0;0];

Blist = [B1, B2, B3, B4, B5];
M = [1, 0, 0, 0.033; 0, 1, 0, 0; 0, 0, 1, 0.6546;0, 0, 0, 1];
%thetalist0 = [0;0;0.15;-1.8;0]; %value can be adjusted for initial guess
%eomg = 0.01;
%ev = 0.001;

%[thetalist, success] = IKinBody(Blist, M, Tse, thetalist0, eomg, ev);

thetalist = currentConfig(4:8);

Jarm = JacobianBody(Blist, currentConfig(4:8));

r = 0.0475;
l = 0.47/2;
w = 0.3/2;
F = r/4 * [-1/(l+w), 1/(l+w) , 1/(l+w) -1/(l+w); 1, 1, 1, 1;-1, 1, -1, 1];
F6 = [zeros(2, 4); F; zeros(1, 4)];

Tb0 = [1,0,0, 0.1662; 0, 1, 0, 0;0, 0, 1, 0.0026;0, 0, 0, 1];
T0e = FKinBody(M, Blist, thetalist);

Jbase = Adjoint(TransInv(T0e) * TransInv(Tb0)) * F6;
Je = [Jbase, Jarm];
Jeplus = pinv(Je, 1e-2);

result = Jeplus * V;



