function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

defaultImportanceWeight = 1 / numParticles;

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q_t = [0.1, 0; 0, 0.1];


% process each particle
for i = 1:numParticles
  robotPose = particles(i).pose;
  
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    landmarkID = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(landmarkID).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
      landmarkX = robotPose(1) + z(j).range * cos(z(j).bearing + robotPose(3));
      landmarkY = robotPose(2) + z(j).range * sin(z(j).bearing + robotPose(3));

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
      particles(i).landmarks(landmarkID).mu = [landmarkX; landmarkY];
      particles(i).landmarks(landmarkID).sigma = pinv(H) * Q_t * pinv(H)';

      particles(i).weight = defaultImportanceWeight; % Maybe get rid of this?

      % Indicate that this landmark has been observed
      particles(i).landmarks(landmarkID).observed = true;

    else

      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      Q = H * particles(i).landmarks(landmarkID).sigma * H' + Q_t;

      % TODO: calculate the Kalman gain
      K = particles(i).landmarks(landmarkID).sigma * H' * pinv(Q);

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
      z_diff = [z(j).range - expectedZ(1); normalize_angle(z(j).bearing - expectedZ(2))];

      % TODO: update the mean and covariance of the EKF for this landmark
      particles(i).landmarks(landmarkID).mu = particles(i).landmarks(landmarkID).mu + K * z_diff;
      particles(i).landmarks(landmarkID).sigma = (eye(2) - K * H) * particles(i).landmarks(landmarkID).sigma;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      A = (det(2*pi.*Q))^(-0.5);
      B = exp(-0.5 * z_diff' * pinv(Q) * z_diff);
      likelihood = A * B;
      particles(i).weight = particles(i).weight * likelihood;

    end

  end % measurement loop
end % particle loop

end
