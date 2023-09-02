% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 

  informationMatrix = eye(3);

  % Maximum number of iterations and convergence threshold
  max_iterations = 100;
  convergence_threshold = 1e-6;

  for iter = 1:max_iterations

    disp(iter);

    % TODO: initialize H and b of the linear system
    H = zeros(9, 9);
    bT = zeros(1, 9);
 
    % TODO: loop through the measurements and update H and b
    % You may call the functions error_function and jacobian, see below
    % We assume that the information matrix is the identity.
    for i = 1:size(Z, 1)
      J = jacobian(i, Z);
      H = H + J'*informationMatrix*J;
      bT = bT + error_function(i, X, Z)'*informationMatrix*J;
    end

    b = bT';

    delta = reshape(-pinv(H)*b, 3, 3);

    disp("X: ");
    disp(X);

    disp("delta: ");
    disp(delta);

    X = X .+ delta;

    % Check for convergence
    disp("norm delta: ");
    disp(norm(delta));
    
    if norm(delta) < convergence_threshold
      disp("converged");
      break;
    end

  end



  % TODO: solve and update the solution
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement

  scanMatched = [Z(i, 1); Z(i, 2); Z(i, 3)];
  odom = [Z(i, 4); Z(i, 5); Z(i, 6)];

  predicted = X*odom;

  e = scanMatched - predicted;

end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian

  z_i = Z(i, 4:6);

  J = [[z_i], [0, 0, 0], [0, 0, 0]; [0, 0, 0], [z_i], [0, 0, 0]; [0, 0, 0], [0, 0, 0], [z_i]];

end
