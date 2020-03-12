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

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  //Set the number of particles
  num_particles = 20;

  // Create random engine
  std::default_random_engine gen;

  // Create normal distributions for x, y and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Particles initialization
  for(int i = 0; i < num_particles; i++) {

    double sample_x = dist_x(gen);
    double sample_y = dist_y(gen);
    double sample_theta = dist_theta(gen);

    double weight = 1.0;

    Particle particle = {
      i, // id
      sample_x, // x
      sample_y, // y
      sample_theta, // theta
      weight, // weight
      {}, // associations
      {}, // sense_x
      {}, // sense_y
    };

    this->particles.push_back(particle);
    this->weights.push_back(weight);

  }

  this->is_initialized = true;

  // std::cout << "INITIALIZATION DONE!" << std::endl; // DEBUG
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {

  // Create random engine
  std::default_random_engine gen;

  // Update state of the particle given previous velocity and yaw rate measurements
  for (Particle& particle : this->particles) {

    if (fabs(yaw_rate) < 0.01) {
      particle.x += velocity * cos(particle.theta) * delta_t;
      particle.y += velocity * sin(particle.theta) * delta_t;
    } else {
      particle.x += (velocity/yaw_rate) * (sin(particle.theta + (yaw_rate*delta_t)) - sin(particle.theta));
      particle.y += (velocity/yaw_rate) * (cos(particle.theta) - cos(particle.theta + (yaw_rate * delta_t)));
      particle.theta += yaw_rate*delta_t;
    }

    // Create normal distributions for x, y and theta
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);

    // Update the particle state with the noise
    particle.x += dist_x(gen);
    particle.y += dist_y(gen);
    particle.theta += dist_theta(gen);
    }
  // std::cout << "PREDICTION DONE!" << std::endl; // DEBUG
}

void ParticleFilter::dataAssociation(vector<LandmarkObs>& transformed_observations,
                                     vector<LandmarkObs> inRange_landmarks) {

  for (LandmarkObs& transformed_observation : transformed_observations) {

    transformed_observation.id = -1;
    double minimum_distance = 1000;

    for (const LandmarkObs& landmark : inRange_landmarks) {
      double distance = dist(transformed_observation.x, transformed_observation.y, landmark.x, landmark.y);
      if (distance < minimum_distance) {
        minimum_distance = distance;
        transformed_observation.id = landmark.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {

  double weight_normalizer = 0.0;

  for (Particle& particle : this->particles) {
    std::vector<LandmarkObs> transformed_observations; //transformed observations from vehicle to world coordinate frame
    std::vector<LandmarkObs> inRange_landmarks;
    for (const LandmarkObs& observation : observations) {

      LandmarkObs transformed_observation;
      coordinateTransformation(particle.x, particle.y, particle.theta, observation, transformed_observation);
      transformed_observations.push_back(transformed_observation);
    }

    for (const Map::single_landmark_s& map_landmark : map_landmarks.landmark_list) {

      if (dist(particle.x, particle.y, map_landmark.x_f, map_landmark.y_f) < sensor_range) {
        // inRange_landmarks.emplace_back(map_landmark.id_i, map_landmark.x_f, map_landmark.y_f);
        inRange_landmarks.push_back(LandmarkObs{map_landmark.id_i, map_landmark.x_f, map_landmark.y_f});
      }
    }

    // Perform the data association between the observed landmarks (transformed_observations) and the visible landmarks (inRange_landmarks)
    this->dataAssociation(transformed_observations, inRange_landmarks);

    // Perform the actual weight update
    particle.weight = 1.0; // reset the particle weight

    // Prepare data structures for storing associations for the single particle
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // Compute the multi-variate Gaussian
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_x_2 = pow(sigma_x, 2);
    double sigma_y_2 = pow(sigma_y, 2);
    double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));

    for (LandmarkObs& transformed_observation : transformed_observations) {
      // std::cout << "[updateWeights] transformed_observation.id = " << transformed_observation.id << std::endl; // DEBUG
      double trans_obs_x = transformed_observation.x;
      double trans_obs_y = transformed_observation.y;
      double multi_prob = 1.0;

      for (LandmarkObs& landmark : inRange_landmarks) {

        if (transformed_observation.id == landmark.id) {
          double error_x = trans_obs_x - landmark.x;
          double error_y = trans_obs_y - landmark.y;
          multi_prob = normalizer * exp(-0.5 * (pow((error_x), 2)/sigma_x_2 + pow((error_y), 2)/sigma_y_2));
          particle.weight *= multi_prob;
        }
      }

      // Store associations and observations
      associations.push_back(transformed_observation.id);
      sense_x.push_back(trans_obs_x);
      sense_y.push_back(trans_obs_y);

      this->SetAssociations(particle, associations, sense_x, sense_y);
    }

    weight_normalizer += particle.weight;
  }

  // Normalize all perticles' weights and store them
  for (int i = 0; i < this->num_particles; i++) {
    this->particles[i].id = i;
    this->particles[i].weight /= weight_normalizer;
    this->weights[i] = this->particles[i].weight;
  }

  // std::cout << "WEIGHTS UPDATE DONE!" << std::endl; // DEBUG
}

void ParticleFilter::resample() {

  std::vector<Particle> resampled_particles;

  // Create particles distribution from particles weights
  std::default_random_engine gen;
  std::discrete_distribution<int> d(this->weights.begin(), this->weights.end());

  // Resample
  for(int i=0; i < this->num_particles; i++) {
    resampled_particles.push_back(particles[d(gen)]);
  }

  // New set of particles
  this->particles = resampled_particles;
  // std::cout << "RESAMPLE DONE!" << std::endl; // DEBUG
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