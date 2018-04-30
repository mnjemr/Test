function [azimuth,pitch,bank] = Quat2Eul(q0,q1,q2,q3)
    % Takes quaternions and calculates the equivalent Euler angles
    % Inputs
    % q0,q1,q2,q3 = quaternions with q0 being the "scalar" value
    %
    % Outputs
    % bank,pitch,azimuth = Euler angles (rad) in 1,2,and 3 axis

    m11 = 2.*(q1.*q2 + q0.*q3);
    m12 = q0.^2 + q1.^2 - q2.^2 - q3.^2;
    m21 = -2.*(q1.*q3 - q0.*q2);
    m31 = 2.*(q2.*q3 + q0.*q1);
    m32 = q0.^2 - q1.^2 - q2.^2 + q3.^2;

    bank = atan2(m31,m32);
    pitch = asin(m21);
    azimuth = atan2(m11,m12);
return

% function [newes] = Quat2Eul(q)
%     %
%     %Function to convert quaternions to Euler angles. The convention used is that of Bunge (1965, 1982)
%     %
%     %Input
%     %	q: 4 x n matrix of quaternions corresponding to the rotation described by Euler angles
%     %
%     %Output:
%     %	newes: n x 3 matrix of Euler angles (with columns phi1 PHI phi2) equivalent to the rotation 
%     %	described by the input quaternion
%     % 
%     %Uses trignometric relations:
%     %q1 = cos((p1+p2)/2)cos(P/2);
%     %q2 = cos((p1-p2)/2)sin(P/2);
%     %q3 = sin((p1-p2)/2)sin(P/2);
%     %q4 = sin((p1+p2)/2)cos(P/2);
%     %
%     %  Copyright 2014 Mark Pearce
%     %
%     %  This file is part of EBSDinterp.
%     %
%     %  EBSDinterp is free software: you can redistribute it and/or modify
%     %  it under the terms of the CSIRO Open Source Software Licence
%     %  distributed with this software as "CSIRO_BSD_MIT_License_v2_0.txt".
%     %
%     %  EBSDinterp is distributed in the hope that it will be useful,
%     %  but WITHOUT ANY WARRANTY; without even the implied warranty of
%     %  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     %  GNU General Public License for more details.
%     %
% 
%     %Split apart quaternions
%     q1 = q(:,1);
%     q2 = q(:,2);
%     q3 = q(:,3);
%     q4 = q(:,4);
% 
%     %Calculate phi1 + phi2 and phi1 - phi2
%     p1pp2 = atan2(q4,q1);
%     p1mp2 = atan2(q3,q2);
% 
%     %Calculate the Euler angles 
%     phi2 = p1pp2 - p1mp2;
%     PHI = 2*atan2(sqrt(q2.^2+q3.^2),sqrt(q1.^2+q4.^2));
%     phi1 = p1pp2 + p1mp2;
% 
%     %If the second Euler angle is 0 then all the rotation can be done by the last (or first) Euler angle 
%     PHIzero = abs(PHI)<=1E-13; %Change these ones
% 
%     %Deal with q1<0 by changing sign of quaternion
%     changeref = ones(size(q1(PHIzero)));
%     changeref(q1(PHIzero)<0) = -1;
% 
%     %Consolidate rotation into phi2
%     phi2(PHIzero) = 2*asin(max(-1,min(1,changeref).*q4(PHIzero)));
%     phi1(PHIzero) = 0;
% 
%     %Output in degrees
%     newes = mod([phi1./pi*180 PHI./pi*180 phi2./pi*180],360);
% end