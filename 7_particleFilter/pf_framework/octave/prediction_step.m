function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1NoiseStdDev = noise(1);
transNoiseStdDev = noise(2);
r2NoiseStdDev = noise(3);

numParticles = length(particles);

for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  r1Noise = normrnd(0, r1NoiseStdDev);
  transNoise = normrnd(0, transNoiseStdDev);
  r2Noise = normrnd(0, r2NoiseStdDev);

  r1 = u.r1 + r1Noise;
  t = u.t + transNoise;
  r2 = u.r2 + r2Noise;

  particles(i).pose(1) = particles(i).pose(1) + t * cos(particles(i).pose(3) + r1);
  particles(i).pose(2) = particles(i).pose(2) + t * sin(particles(i).pose(3) + r1);
  particles(i).pose(3) = particles(i).pose(3) + r1 + r2;
  

end

end
