function tau = get_operational_space_control(q_curr, dq_curr, p_target, v_target, params, varargin)
% GET_OPERATIONAL_SPACE_CONTROL Compute joint torques using operational space control
%
% Supports two control modes:
%   1. Simple PD control (Siciliano Eq 8.110) - acts like a virtual spring
%   2. Inverse dynamics control - full feedback linearization
%
% Inputs:
%   q_curr      - Current joint angles [3x1] (rad)
%   dq_curr     - Current joint velocities [3x1] (rad/s)
%   p_target    - Target end-effector position [3x1] (m)
%   v_target    - Target end-effector velocity [3x1] (m/s)
%   params      - Structure containing:
%                   .a2, .a3 - Link lengths (m)
%                   .kp - Proportional gain
%                   .kd - Derivative gain
%                   For inverse dynamics mode, also requires:
%                     .m1, .m2, .m3 - Link masses (kg)
%                     .I1, .I2, .I3 - Inertia tensors [3x3] (kg*m^2)
%                     .g - Gravitational acceleration (m/s^2)
%   varargin    - Optional arguments:
%                   If 1 arg: G (gravity torques) for simple mode
%                   If 2 args: a_target, mode where mode = 'inverse_dynamics'
%
% Outputs:
%   tau         - Commanded joint torques [3x1] (Nm)
%
% Examples:
%   tau = get_operational_space_control(q, dq, p_des, v_des, params, G)
%   tau = get_operational_space_control(q, dq, p_des, v_des, params, a_des, 'inverse_dynamics')

    %% Parse input arguments
    if nargin == 6
        % Simple mode: 6th argument is G
        mode = 'simple';
        G = varargin{1};
        a_target = [];
    elseif nargin == 7
        % Inverse dynamics mode: 6th arg is a_target, 7th is mode
        a_target = varargin{1};
        mode = varargin{2};
        G = [];
    else
        error('Invalid number of arguments. Use 6 for simple mode or 7 for inverse dynamics mode.');
    end

    %% 1. Get Jacobian
    J_full = get_Jacobian(q_curr(1), q_curr(2), q_curr(3), params.a2, params.a3);
    J_pos = J_full(1:3, :);  % Only control position (X,Y,Z), take top 3 rows

    %% 2. Calculate Current Cartesian State
    p_curr = get_forward_kinematics(q_curr, params.a2, params.a3);
    v_curr = J_pos * dq_curr;

    %% 3. Cartesian Control Law
    Kp = params.kp * eye(3);
    Kd = params.kd * eye(3);

    e = p_target - p_curr;
    de = v_target - v_curr;

    %% 4. Compute Joint Torques
    if strcmp(mode, 'simple')
        % Simple operational space control (PD + gravity compensation)
        F_des = Kp*e + Kd*de;
        tau = J_pos' * F_des + G;

    elseif strcmp(mode, 'inverse_dynamics')
        % Inverse dynamics control (computed torque in Cartesian space)

        % Get dynamics matrices
        B = get_B_matrix(q_curr(1), q_curr(2), q_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, ...
            params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
            params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
            params.I3(1,1), params.I3(2,2), params.I3(3,3));

        C = get_C_matrix(q_curr(1), q_curr(2), q_curr(3), dq_curr(1), dq_curr(2), dq_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, ...
            params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
            params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
            params.I3(1,1), params.I3(2,2), params.I3(3,3));

        G = get_G_vector(q_curr(1), q_curr(2), q_curr(3), ...
            params.m1, params.m2, params.m3, params.a2, params.a3, params.g);

        % Get Jacobian derivative
        J_dot = get_J_dot(q_curr(1), q_curr(2), q_curr(3), dq_curr(1), dq_curr(2), dq_curr(3), ...
                          params.a2, params.a3);

        % Desired Cartesian acceleration
        y = a_target + Kd*de + Kp*e;

        % Compute torques with full dynamics compensation
        inv_J_term = J_pos \ (y - J_dot * dq_curr);

        damping_coeff = 0.5;  % Must match f_arm_dynamics.m
        F_friction = damping_coeff * dq_curr;

        tau = B * inv_J_term + C * dq_curr + G + F_friction;

    else
        error('Invalid mode. Use ''simple'' or ''inverse_dynamics''.');
    end

end
