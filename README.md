# Overview
This is the project documentation for the Localization Project provided by the Udacity's Self-Driving Car Nanodegree.

This project aims to implement a 2 dimensional Particle Filter in C++ to localize a vehicle by using the observation and control data provided by the vehicle.

<p align="center">
  <img width="460" height="300" src="https://miro.medium.com/max/1735/1*VMTKYLh2c2cW7yE6mxFU7g.png">
</p>

The particle filter consists of four steps:
- Initialization Step - initialize the particles based on the GPS measurements and gaussian noise.
- Prediction Step - predicts the state for the next time step using the process model.
- Update Step (Update Weights) - updates the weights for each particle based on the likelihood of the observed measurements.
- Resample Step - Resamples from the updated set of particles to form the new set of particles.



