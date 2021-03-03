%MDL_SRC2M Create model of SRC2 MANIPULATOR%
% MDL_SRC2M is a script that creates the workspace variable src2 which
% describes the kinematic characteristics of the manipulator in SRC2 
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         arm along +ve x-axis configuration
%
% Reference::
% - https://www..com/
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also mdl_ur5, mdl_ur10, mdl_puma560, SerialLink.

% MODEL: NASA, SRC2m, 4DOF, standard_DH



% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function r = mdl_src2m()
    
    deg = pi/180;
    
    % robot length values (metres)
    a = [0, 0.8, 0.8, 0.165]';

    d = [0.12, 0, 0, 0]';

    alpha = [-pi/2, 0, 0, 0]';

    theta = [0, 0, 0, 0]';
    
    DH = [theta d a alpha];

    mass = [1, 1, 1, 1];

    center_of_mass = [
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0];
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    robot = SerialLink(DH, ...
        'name', 'SRC2M', 'manufacturer', 'NASA');
    
    % add the mass data, no inertia available
    links = robot.links;
    for i=1:4
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end

    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'src2m', robot);
        assignin('caller', 'qz', [0 0 0 0 0]); % zero angles
        assignin('caller', 'qr', [0 0 0 0 0]*deg); % vertical pose as per Fig 2
    end
end