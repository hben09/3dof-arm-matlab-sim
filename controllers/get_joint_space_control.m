function tau = get_joint_space_control(q_curr, dq_curr, q_des, dq_des, params, varargin)
% GET_JOINT_SPACE_CONTROL
%
% Computes joint torques using joint space control with two available modes:
%   1. Simple PD control with gravity compensation
%   2. Inverse dynamics control (computed torque method)
%
% INPUTS:
%   q_curr      : Current joint angles [3x1] (rad)
%   dq_curr     : Current joint velocities [3x1] (rad/s)
%   q_des       : Desired joint angles [3x1] (rad)
%   dq_des      : Desired joint velocities [3x1] (rad/s)
%   params      : Structure containing control gains and physical parameters
%   varargin    : G (simple mode) or ddq_des, 'inverse_dynamics'
%
% OUTPUTS:
%   tau         : Commanded joint torques [3x1] (Nm)
%
% USAGE:
%   tau = get_joint_space_control(q, dq, q_des, dq_des, params, G)
%   tau = get_joint_space_control(q, dq, q_des, dq_des, params, ddq_des, 'inverse_dynamics')

    %% 0. Parse Input Arguments
    if nargin == 6
        mode = 'simple';
        G = varargin{1};
        ddq_des = [];
    elseif nargin == 7
        mode = 'inverse_dynamics';
        ddq_des = varargin{1};
        if ~strcmp(varargin{2}, 'inverse_dynamics')
            error('For 7 arguments, 7th argument must be ''inverse_dynamics''.');
        end
        G = [];
    else
        error('Invalid number of arguments. Use 6 for simple mode or 7 for inverse dynamics mode.');
    end

    %% 1. Extract Control Gains
    Kp = params.kp * eye(3);
    Kd = params.kd * eye(3);

    %% 2. Calculate Tracking Error
    e  = q_des - q_curr;
    de = dq_des - dq_curr;

    %% 3. Compute Control Torques
    if strcmp(mode, 'simple')
        tau = Kp*e + Kd*de + G;

    elseif strcmp(mode, 'inverse_dynamics')
        % 3a. Get dynamics matrices
        B = get_B_matrix(q_curr(1), q_curr(2), q_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, ...
            params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
            params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
            params.I3(1,1), params.I3(2,2), params.I3(3,3));

        C = get_C_matrix(q_curr(1), q_curr(2), q_curr(3), ...
            dq_curr(1), dq_curr(2), dq_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, ...
            params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
            params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
            params.I3(1,1), params.I3(2,2), params.I3(3,3));

        G = get_G_vector(q_curr(1), q_curr(2), q_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, params.g);

        % 3b. Calculate desired acceleration with feedback
        y = ddq_des + Kd*de + Kp*e;

        % 3c. Friction compensation
        damping_coeff = 0.5;
        F_friction = damping_coeff * dq_curr;

        % 3d. Assemble final torque
        tau = B * y + C * dq_curr + G + F_friction;

    else
        error('Invalid mode. Use ''simple'' or ''inverse_dynamics''.');
    end

end
