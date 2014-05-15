syms roll_torque pitch_torque yaw_torque real
syms normalized_thrust real
syms mass real
syms L K real
syms gamma_1 gamma_2 gamma_3 gamma_4 real
syms f1 f2 f3 f4 real

%% PX4 Convention

% x-configuration
S = solve(sqrt(2)/2*L*(f1 - f2 - f3 + f4) == roll_torque, ...
          sqrt(2)/2*L*(f1 + f2 - f3 - f4) == pitch_torque, ...
          K*(-f1 + f2 - f3 + f4) == yaw_torque, ...
          (f1 + f2 + f3 + f4) == mass*normalized_thrust, ...
          f1, f2, f3, f4);
[simplify(S.f1/gamma_1);
 simplify(S.f2/gamma_2);
 simplify(S.f3/gamma_3);
 simplify(S.f4/gamma_4);]

% +-configuration
S = solve(L*(-f2 + f4) == roll_torque, ...
          L*(f1 - f3) == pitch_torque, ...
          K*(-f1 + f2 - f3 + f4) == yaw_torque, ...
          (f1 + f2 + f3 + f4) == mass*normalized_thrust, ...
          f1, f2, f3, f4);
[simplify(S.f1/gamma_1);
 simplify(S.f2/gamma_2);
 simplify(S.f3/gamma_3);
 simplify(S.f4/gamma_4);]

%% RPG Convention

% x-configuration
S = solve(sqrt(2)/2*L*(f1 - f2 - f3 + f4) == roll_torque, ...
          sqrt(2)/2*L*(-f1 - f2 + f3 + f4) == pitch_torque, ...
          K*(f1 - f2 + f3 - f4) == yaw_torque, ...
          (f1 + f2 + f3 + f4) == mass*normalized_thrust, ...
          f1, f2, f3, f4);
[simplify(S.f1/gamma_1);
 simplify(S.f2/gamma_2);
 simplify(S.f3/gamma_3);
 simplify(S.f4/gamma_4);]

% +-configuration
S = solve(L*(-f2 + f4) == roll_torque, ...
          L*(-f1 + f3) == pitch_torque, ...
          K*(f1 - f2 + f3 - f4) == yaw_torque, ...
          (f1 + f2 + f3 + f4) == mass*normalized_thrust, ...
          f1, f2, f3, f4);
[simplify(S.f1/gamma_1);
 simplify(S.f2/gamma_2);
 simplify(S.f3/gamma_3);
 simplify(S.f4/gamma_4);]

