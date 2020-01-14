/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */

  std::default_random_engine gen;

  num_particles = 25;  // TODO: Set the number of particles
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  particles.reserve(num_particles);
  particles.resize(num_particles);
  weights.reserve(num_particles);
  weights.resize(num_particles);

  cout << "Particle size from init: " << particles.size() << endl;

  for (int i = 0; i < num_particles; ++i) {
    // initialize particles parameters
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
    weights[i] = 1.0;
  }

  // particle filter is initialized
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    // initialize random generator
    default_random_engine gen;

    // Make distributions for adding noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    cout << "In prediction" << endl;
    for (auto i = 0; i < particles.size(); i++) {
      // check if yaw_rate is 0
      if (fabs(yaw_rate) > 0.0001) {
        particles[i].x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
        particles[i].y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        particles[i].theta = particles[i].theta + yaw_rate * delta_t;
      }
      else {
        particles[i].x += velocity * delta_t * cos(particles[i].theta);
        particles[i].y += velocity * delta_t * sin(particles[i].theta);
      }

      // Add noise to the particles
      particles[i].x += dist_x(gen);
      particles[i].y += dist_y(gen);
      particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
  cout<<"Size of predicted: " << predicted.size() << endl;
  cout<<"Size of observations: " << observations.size() << endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
   // declare x_map and y_map which are the observations in map coordinate system
   double x_map, y_map;
   for (int i = 0; i < particles.size(); i++) {
     // the new weight is initialized with 1.0
     double new_part_weight = 1.0;
     for (int j = 0; j < observations.size(); j++) {
       // transform the coordinate from car coordinate to map coordinate
       x_map = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
       y_map = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);

       // distances vector which will have distances between landmarks and observations
       vector<double>distances;
       for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
            // compute the distances
            double land_dist = dist(x_map, y_map, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
            distances.push_back(land_dist);
       }

       // the index of the nearest landmark
       int minDistance = minIndex(distances);

       // dealocate the memory of the distances vector
       distances.clear();

       // use the multivariate probability in the particle new weight
       new_part_weight *= multiv_prob(std_landmark[0], std_landmark[1], x_map, y_map,
                                    map_landmarks.landmark_list[minDistance].x_f,
                                    map_landmarks.landmark_list[minDistance].y_f);
     }
     // modify the old weight with the new one
     particles[i].weight = new_part_weight;
     weights[i] = new_part_weight;
   }
}

void ParticleFilter::resample() {

  // initialize random generators
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  srand((unsigned)time(0));

  // a vector which contains resampled particles
  std::vector<Particle>resampled;

  // pick a random index
  int index = (rand() % (num_particles-1));
  double beta = 0.0;
  for (int i = 0; i < num_particles; i++) {
    beta += distribution(generator) * 2 * maxElement(weights);
    while (weights[index] < beta) {
      beta = beta - weights[index];
      index = (index + 1) % num_particles;
    }
    resampled.push_back(particles[index]);
  }
  // replace the old particles with new resampled ones
  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
