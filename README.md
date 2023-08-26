# freiburg_slam

## Occupany Grid Mapping:
![alt-text](https://github.com/flynnletford/freiburg_slam/blob/d97a54f68e2743d880635f4cb0d4fa5bf0d55c79/7_gridMapping/gridmap.gif)

## Particle Filter:
![alt-text](https://github.com/flynnletford/freiburg_slam/blob/1a904c44f8cdc826999cb86540229194e891e295/7_particleFilter/particle_filter.gif)

## FastSLAM:
FastSLAM is a combination of EKF SLAM and a particle filter. Every particle makes use of the assumption that comes with "Mapping with Known Poses". 
Here each particle's pose estimate is assumed to be right, and so the landmark positions can then be estimated based on this assumption.
Every particle keeps an EKF for every landmark it sees. 

The predicted pose is the weighted average of all of the particles at each time step.

### Prediction Step:
The odometry motion model is used to update the predicted position of the particle.

### Correction Step:
Each measurement that comes in is compared against the expected measurement for the landmark it is determined to be measuring.
The weight of each particle is then calculated. Particles whose measurements most closely match the predicted measurements will have a higher weighting.

### Resampling:
Low variance resampling is used where particles with the highest weights are more likely to remain in the particle pool for the next timestep.

![alt-text](https://github.com/flynnletford/freiburg_slam/blob/4c9aef9ae88b47e9ae1e3531acbf27f195b9fe12/8_fastSLAM/fastSLAM.gif)
