function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);

N = (max(size(mu))-3)/2;

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];

for i = 1:m

	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;

	% If the landmark is observed for the first time:
	if(!observedLandmarks(landmarkId))

		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		mu(2*landmarkId+2) = mu(1) + z(i).range*cos(z(i).bearing + mu(3));
		mu(2*landmarkId+3) = mu(2) + z(i).range*sin(z(i).bearing + mu(3));

		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% TODO: Add the landmark measurement to the Z vector
	Z(2*i-1) = z(i).range;
	Z(2*i) = z(i).bearing;

	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:

	Lx = mu(2*landmarkId+2);
	Ly = mu(2*landmarkId+3);

	del = zeros(2,1);
	del(1,1) = Lx - mu(1);
	del(2,1) = Ly - mu(2);
	q = del.' * del;

	expectedZ(2*i-1) = sqrt(q);
	expectedZ(2*i) = atan2(del(2, 1), del(1, 1)) - mu(3);

	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
	lowH = [-sqrt(q)*del(1), -sqrt(q)*del(2), 0, sqrt(q)*del(1), sqrt(q)*del(2);
			del(2), -del(1), -q, -del(2), del(1)];
	lowH = lowH/q;

	F = zeros(5, 2*N+3);
	F(1,1) = 1;
	F(2,2) = 1;
	F(3,3) = 1;
	F(4,3+2*i-1) = 1;
	F(5,3+2*i) = 1;

	Hi = lowH*F;

	% Augment H with the new Hi
	H = [H;Hi];	
endfor

% TODO: Construct the sensor noise matrix Q
sensorNoise = 0.01;
Q = eye(2*m, 2*m)*sensorNoise;

% TODO: Compute the Kalman gain
inner = H*sigma*H.' + Q;
K = sigma * H.' * pinv(inner);

% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
innovation = Z - expectedZ;
innovation = normalize_all_bearings(innovation);

% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu = mu + K*innovation;
mu(3) = normalize_angle(mu(3));

KH = K*H;
identityM = eye(max(size(KH)));
sigma = (identityM - KH)*sigma;

end
