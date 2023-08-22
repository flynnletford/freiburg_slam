% Turn off pagination:
more off;

clear all;
close all;

% Make tools available
addpath('tools');

% Read sensor readings, i.e. odometry
data = read_data('../data/odometry.dat');

noise = [0.005, 0.01, 0.005]';

% how many particles
numParticles = 100;

% initialize the particles array
particles = struct;
for i = 1:numParticles
  particles(i).weight = 1. / numParticles;
  particles(i).pose = zeros(3,1);
  particles(i).history = cell();
end

% Perform filter update for each odometry-observation read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:50
    printf('timestep = %d\n', t);

    % Perform the prediction step of the particle filter
    particles = prediction_step(particles, data.timestep(t).odometry, noise);

    % re-weight the particles according to their distance to [0 0]
    sigma = diag([0.2 0.2]);
    for i = 1:numParticles
      pose = particles(i).pose;
      newPose = pose(1:2); % Our resampling doesn't use theta so we remove it from the pose we pass it.
      particles(i).weight = exp(-1/2 * newPose' * inv(sigma) * newPose);
    end

    resampledParticles = resample(particles);

    % plot the particles before (red) and after resampling (blue)
    bpos = [particles.pose];
    apos = [resampledParticles.pose];
    plot(bpos(1,:), bpos(2,:), 'r+', 'markersize', 5, apos(1,:), apos(2,:), 'b*', 'markersize', 5);

    particles = resampledParticles;

    % Generate visualization plots of the current state of the filter
    plot_state(particles, t);
end
