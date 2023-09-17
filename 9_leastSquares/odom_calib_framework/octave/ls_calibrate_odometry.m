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

  % TODO: initialize H and b of the linear system
  H = zeros(9, 9);
  bT = zeros(1, 9);

  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  for i = 1:size(Z, 1)
    J = jacobian(i, Z);
    H = H + J'*informationMatrix*J;
    err = error_function(i, X, Z);

    bT = bT + err'*informationMatrix*J;
  end

  b = bT';

  delta = reshape(-pinv(H)*b, 3, 3);

  X = X .+ delta;

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

  scanMatched = Z(i, 1:3)';
  odom = Z(i, 4:6)';

  predicted = X*odom;

  rawError = scanMatched - predicted;

  e = rawError; %.*rawError;

end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  z_i = Z(i, 4:6);

  J = zeros(3, 9);

  J(1, 1:3) = -z_i;
  J(2, 4:6) = -z_i;
  J(3, 7:9) = -z_i;

end
