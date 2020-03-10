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
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */

  //Set the number of particles
  num_particles = 3;

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

  std::cout << "INITIALIZATION DONE!" << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  /*
  // Add Gaussian Noise to the velocity and yaw rate measurements. (std???)
  // Create random engine
  std::default_random_engine gen;

  // Create normal distributions for velocity and yaw_rate (std???)
  std::normal_distribution<double> dist_velocity(velocity, std_pos[0]); // TODO
  std::normal_distribution<double> dist_yaw_rate(yaw_rate, std_pos[1]); // TODO
  double meas_velocity = dist_velocity(gen);
  double meas_yaw_rate = dist_yaw_rate(gen);
  // Create random engine
  std::default_random_engine gen;
  */

  // Update state of the particle given previous velocity and yaw rate measurements
  for (Particle& particle : this->particles) {

    double updated_theta;
    double updated_x;
    double updated_y;

    if (fabs(yaw_rate) < 0.0001) {
      updated_theta = particle.theta + yaw_rate*delta_t;
      updated_x = particle.x + velocity * cos(particle.theta) * delta_t;
      updated_y = particle.y + velocity * sin(particle.theta) * delta_t;
    } else {
      updated_theta = particle.theta + yaw_rate*delta_t;
      updated_x = particle.x + velocity/yaw_rate * (sin(updated_theta) - sin(particle.theta));
      updated_y = particle.y + velocity/yaw_rate * (cos(particle.theta) - cos(updated_theta));
    }

    particle.x = updated_x;
    particle.y = updated_y;
    particle.theta = updated_theta;

    // Create random engine
    std::default_random_engine gen;
    // Create normal distributions for x, y and theta
    std::normal_distribution<double> dist_x(particle.x, std_pos[0]);
    std::normal_distribution<double> dist_y(particle.y, std_pos[1]);
    std::normal_distribution<double> dist_theta(particle.theta, std_pos[2]);

    // Update the particle state with the noisy measurement
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    }
  std::cout << "PREDICTION DONE!" << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> transformed_observations,
                                     vector<LandmarkObs>& inRange_landmarks) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

  //  TODO: check, transformed_observation.id is always 0
  for (LandmarkObs& transformed_observation : transformed_observations) {

    transformed_observation.id = -1;
    double minimum_distance = std::numeric_limits<double>::max();

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
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double weight_normalizer = 0.0;

  for (Particle& particle : this->particles) {
    std::vector<LandmarkObs> transformed_observations; //transformed observations from vehicle to world coordinate frame
    std::vector<LandmarkObs> inRange_landmarks;
    std::cout << "[updateWeights] Coordinate transformation..." << std::endl; //DEBUG
    for (const LandmarkObs& observation : observations) {

      LandmarkObs transformed_observation;
      coordinateTransformation(particle.x, particle.y, particle.theta, observation, transformed_observation);

      if (dist(particle.x, particle.y, transformed_observation.x, transformed_observation.y) < sensor_range) {
        transformed_observations.push_back(transformed_observation);
      }
    }
    std::cout << "[updateWeights] inRange_landmarks creation..." << std::endl; //DEBUG
    for (const Map::single_landmark_s& map_landmark : map_landmarks.landmark_list) {

      if (dist(particle.x, particle.y, map_landmark.x_f, map_landmark.y_f) < sensor_range) {
        // inRange_landmarks.emplace_back(map_landmark.id_i, map_landmark.x_f, map_landmark.y_f);
        inRange_landmarks.push_back(LandmarkObs{map_landmark.id_i, map_landmark.x_f, map_landmark.y_f});
      }
    }

    std::cout << "[updateWeights] dataAssociation..." << std::endl; //DEBUG
    // Perform the data association between the observed landmarks (transformed_observations) and the visible landmarks (inRange_landmarks)
    this->dataAssociation(transformed_observations, inRange_landmarks);

    // Perform the actual weight update
    std::cout << "[updateWeights] multi-variate Gaussia evaluation..." << std::endl; //DEBUG

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

      double trans_obs_x = transformed_observation.x;
      double trans_obs_y = transformed_observation.y;
      double multi_prob = 1.0;

      for (LandmarkObs& landmark : inRange_landmarks) {

        if (transformed_observation.id == landmark.id) {
          multi_prob = normalizer * exp(-1.0 * ((pow((trans_obs_x - landmark.x), 2)/(2.0 * sigma_x_2)) + (pow((trans_obs_y - landmark.y), 2)/(2.0 * sigma_y_2))));
          particle.weight *= multi_prob;
        }
      }

      // Store associations and observations
      associations.emplace_back(transformed_observation.id);
      std::cout << "transformed_observation.id = " << transformed_observation.id << std::endl;
      sense_x.emplace_back(trans_obs_x);
      sense_y.emplace_back(trans_obs_y);

      this->SetAssociations(particle, associations, sense_x, sense_y);
    }
    weight_normalizer += particle.weight;
  }
  std::cout << "[updateWeights] weigths normalization..." << std::endl; //DEBUG
  // Normalize all perticles' weights and store them
  for (int i = 0; i < this->num_particles; i++) {
    this->particles[i].weight /= weight_normalizer;
    this->weights[i] = this->particles[i].weight;
  }
  std::cout << "WEIGHTS UPDATE DONE!" << std::endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::vector<Particle> resampled_particles;

  // Create particles distribution from particles weights
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(this->weights.begin(), this->weights.end());

  // Resample
  for(unsigned int i=0; i < this->num_particles; i++){
    const int index = d(gen);
    resampled_particles.emplace_back(particles[index]);
  }

  this->particles.clear();
  // this->weights.clear();
  // New set of particles
  this->particles = resampled_particles;
  std::cout << "RESAMPLE DONE!" << std::endl;
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