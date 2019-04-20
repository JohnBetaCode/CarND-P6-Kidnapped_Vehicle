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

//OK
/**
 * init Initializes particle filter by initializing particles to Gaussian
 *   distribution around first position and all the weights to 1.
 * @param x Initial x position [m] (simulated estimate from GPS)
 * @param y Initial y position [m]
 * @param theta Initial orientation [rad]
 * @param std[] Array of dimension 3 [standard deviation of x [m], 
 *   standard deviation of y [m], standard deviation of yaw [rad]]
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: (Done) Set the number of particles. Initialize all particles to 
   * first position (based on estimates of x, y, theta and their uncertainties
   * from GPS) and all weights to 1. 
  */

  if (is_initialized) {
    return;
  }

  // TODO: (Done) Set the number of particles
  // Few particles, not enough to cover all of the high likelihood positions
  // Many particles slow down the filter
  num_particles = 100;  

  // Extract and set the standard deviations for x, y, and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Creating normal distribution for x, y, and theta
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  /*
   * TODO: (None) Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   * (and others in this file).
  */
 
  // Generate particles with normal distribution with mean on GPS values.
  for (int i = 0; i < num_particles; i++) {

    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;

    // Print your samples to the terminal.
    // std::cout << "Particle: " << i 
    //           << " x:" << particle.x  
    //           << " y:" << particle.y 
    //           << " theta:" << particle.theta 
    //           << std::endl;

    particles.push_back(particle);
    weights.push_back(1);
	}

  // The filter is now initialized.
  is_initialized = true;

}

//OK
/**
 * prediction Predicts the state for the next time step
 *   using the process model.
 * @param delta_t Time between time step t and t+1 in measurements [s]
 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
 *   standard deviation of y [m], standard deviation of yaw [rad]]
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: (Done) Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */


  // Creating normal distributions for particles parameters
  std::normal_distribution<double> dist_x(0, std_pos[0]); // Standard deviation for x coordinate
  std::normal_distribution<double> dist_y(0, std_pos[1]); // Standard deviation for y coordinate
  std::normal_distribution<double> dist_theta(0, std_pos[2]); // Standard deviation for theta

  // Calculate new state for each particle
  for (int i = 0; i < num_particles; i++) {

  	double theta = particles[i].theta;

    // If yaw is not changing
    // Limit to consider yaw rate is changing is not noise
    if (fabs(yaw_rate) < 0.0001 ) { 
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );
      // yaw continue to be the same
    } else {
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    }

    // Adding noise to each particle parameter
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }

}

//OK
/**
 * dataAssociation Finds which observations correspond to which landmarks 
 *   (likely by using a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: (Done) Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // Iterate over each observation
  for (unsigned int i = 0; i < observations.size(); i++) { 

    // Initialize a big big min distance
    double minDistance=std::numeric_limits<double>::max();

    // Assign observation as unsigned id or unsigned prediction
    observations[i].id = -1;

    // Iterate over each predicted
    for (unsigned int j = 0; j < predicted.size(); j++) { 

      double distance = sqrt(
        pow(observations[i].x - predicted[j].x, 2.0) +
        pow(observations[i].y - predicted[j].y, 2.0));

      // If the "distance" is less than minDistance then update the prediction id.
      if (distance < minDistance ) {
        minDistance = distance;
        predicted[j].id=j;
      }
    }
  }
}

//OK
/**
 * updateWeights Updates the weights for each particle based on the likelihood
 *   of the observed measurements. 
 * @param sensor_range Range [m] of sensor
 * @param std_landmark[] Array of dimension 2
 *   [Landmark measurement uncertainty [x [m], y [m]]]
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks*/
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a multi-variate Gaussian 
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


  // Iterate over all particles
  for (int idx = 0; idx < num_particles; idx++){

    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // Landmarks variables initialization
    vector<LandmarkObs> transformed_observations;
    
    // Iterate over all observations and transform to map axis reference
    for (unsigned int j = 0; j < observations.size(); j++) {
      LandmarkObs TransLandMark; // Transformed landmark variable
      TransLandMark.x = particles[idx].x + observations[j].x*std::cos(particles[idx].theta) - observations[j].y*std::sin(particles[idx].theta);
      TransLandMark.y = particles[idx].y + observations[j].x*std::sin(particles[idx].theta) + observations[j].y*std::cos(particles[idx].theta);
      transformed_observations.push_back(TransLandMark); // Add to vector
    }

    // Set particles weight in 1
    particles[idx].weight = 1.;

    // Initialize min distance to associate observation with a big big number
    double min_dist = std::numeric_limits<double>::max();

    // Iterate over all transformed observations and associate closest observation to landmark
    for (unsigned int i = 0; i < transformed_observations.size(); i++ ){
      int minIndex = 0;
      for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        // Calculate "distance" 
        double dist = std::pow(transformed_observations[i].x - map_landmarks.landmark_list[j].x_f, 2.0 ) + 
                      std::pow(transformed_observations[i].y - map_landmarks.landmark_list[j].y_f, 2.0);
        if (dist < min_dist) {
          min_dist = dist;
          minIndex = j;
        }
      }

      // Variables for operations
      double x = transformed_observations[i].x; // x observation in map coordinates
      double y = transformed_observations[i].y; // y observation in map coordinates
      double mu_x = map_landmarks.landmark_list[minIndex].x_f; // x coordinate of the nearest landmarks
      double mu_y = map_landmarks.landmark_list[minIndex].y_f; // y coordinate of the nearest landmarks
      double std_x = std_landmark[0]; // standard deviation in the x ranges
      double std_y = std_landmark[1]; // standard deviation in the y ranges

      // Calculating the Particle's Final Weight
      // Multivariate-Gaussian probability density
      double multiplier = 1/(2*M_PI*std_x*std_y)*std::exp(-0.5*(std::pow((x-mu_x)/std_x, 2) + std::pow((y-mu_y)/std_y, 2)));
      
      // To get the final weight just multiply all the calculated measurement probabilities together
      if (multiplier > 0){
        particles[idx].weight *= multiplier;
      }

      associations.push_back(minIndex+1);
      sense_x.push_back(x);
      sense_y.push_back(y);
        
    }
    
    // Update weights
    weights[idx] = particles[idx].weight;

    //Set a particles list of associations, along with the associations'
    SetAssociations(particles[idx], associations, sense_x, sense_y);
  }

}

//OK
/**
 * re sample Re-samples from the updated set of particles to form
 *   the new set of particles.
 */
void ParticleFilter::resample() {
  /**
   * TODO: (Done) Re-sample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // Get weights and max weight.
  vector<double> weights;

  // Initialize max weight with the maximum number of a double variable
  double maxWeight = std::numeric_limits<double>::min();

  for(int i = 0; i < num_particles; i++) {

    // Add weight to weights vector 
    weights.push_back(particles[i].weight);

    // Re-assign maximum weight
    if (particles[i].weight > maxWeight) {
      maxWeight = particles[i].weight;
    }
  }

  // Creating distributions for values and indexes
  std::uniform_real_distribution<double> distDouble(0.0, maxWeight);
  std::uniform_int_distribution<int> distInt(0, num_particles - 1);

  // Generating index.
  int index = distInt(gen);

  // Initialize beta
  double beta = 0.0;

  // Initialize re-sampled particles vector
  vector<Particle> resampledParticles;
  
  // Apply the wheel code
  for(int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  // Re-assign particles as the re-sampled particles vector
  particles = resampledParticles;

}

//OK
/**
   * Set a particles list of associations, along with the associations'
   *   calculated world x,y coordinates
   * This can be a very useful debugging tool to make sure transformations 
   *   are correct and associations correctly connected
   */
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

//OK
/**
 * Used for obtaining debugging information related to particles.
 */
string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

//OK
/**
 * Used for obtaining debugging information related to particles.
 */
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