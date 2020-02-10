# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

<p align="center">
  <img width="460" height="300" src="https://miro.medium.com/max/1735/1*VMTKYLh2c2cW7yE6mxFU7g.png">
</p>

The particle filter consists of four steps:
- Initialization Step - initialize the particles based on the GPS measurements and gaussian noise.
- Prediction Step - predicts the state for the next time step using the process model.
- Update Step (Update Weights) - updates the weights for each particle based on the likelihood of the observed measurements.
- Resample Step - Resamples from the updated set of particles to form the new set of particles.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.



