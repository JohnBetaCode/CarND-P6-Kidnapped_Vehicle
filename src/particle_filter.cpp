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

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;

    particles.push_back(p);
    weights.push_back(1);
  }

  is_initialized = true;


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

  double new_x;
  double new_y;
  double new_theta;

  for (int i = 0; i < num_particles; i++){
    Particle p = particles[i];
    if (yaw_rate == 0) {
      new_x = p.x + velocity*delta_t*std::cos(p.theta);
      new_y = p.y + velocity*delta_t*std::sin(p.theta);
      new_theta = p.theta;
    } else {
      new_x = p.x + velocity/yaw_rate*(std::sin(p.theta + yaw_rate*delta_t) - std::sin(p.theta));
      new_y = p.y + velocity/yaw_rate*(std::cos(p.theta) - std::cos(p.theta + yaw_rate*delta_t));
      new_theta = p.theta + yaw_rate*delta_t;
    }

  std::normal_distribution<double> dist_x(new_x, std_pos[0]);
  std::normal_distribution<double> dist_y(new_y, std_pos[1]);
  std::normal_distribution<double> dist_theta(new_theta, std_pos[2]);

  particles[i].x = dist_x(gen);
  particles[i].y = dist_y(gen);
  particles[i].theta = dist_theta(gen);

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
    for(unsigned int i = 0; i < observations.size(); i++) {
      int nObs = observations.size();
      int nPred = predicted.size();
      for( int i = 0; i < nObs; i++) { // For each observation
        double minDist = std::numeric_limits<double>::max();
        int mapId = -1;
        for(int j = 0; j < nPred; j++ ) { // For each predition.
          double xDist = observations[i].x - predicted[j].x;
          double yDist = observations[i].y - predicted[j].y;
          double distance = xDist * xDist + yDist * yDist;
          if(distance < minDist) {
            minDist = distance;
            mapId = predicted[j].id;
          }
          observations[i].id = mapId;
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

  for (int p = 0; p < num_particles; p++) {
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    vector<LandmarkObs> transformed_observations;
    LandmarkObs t_o;
    LandmarkObs obs;
    Particle particle = particles[p];

    for (unsigned int j = 0; j < observations.size(); j++) {
      obs = observations[j];
      t_o.x = particle.x + obs.x*std::cos(particle.theta) - obs.y*std::sin(particle.theta);
      t_o.y = particle.y + obs.x*std::sin(particle.theta) + obs.y*std::cos(particle.theta);
      transformed_observations.push_back(t_o);
    }

    particles[p].weight = 1;
    double closest = std::pow(sensor_range, 2.0);
    for (unsigned int i = 0; i < transformed_observations.size(); i++ ){
      LandmarkObs obs = transformed_observations[i];
      int minIndex = 0;
      for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        double landmark_x = map_landmarks.landmark_list[j].x_f;
        double landmark_y = map_landmarks.landmark_list[j].y_f;

        double calculated = std::pow(obs.x - landmark_x, 2.0 ) + std::pow(obs.y - landmark_y, 2.0);
        if (calculated < closest) {
          closest = calculated;
          minIndex = j;
        }
      }
      double x = obs.x;
      double y = obs.y;
      double mu_x = map_landmarks.landmark_list[minIndex].x_f;
      double mu_y = map_landmarks.landmark_list[minIndex].y_f;
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];

      double multiplier = 1/(2*M_PI*std_x*std_y)*std::exp(-0.5*(std::pow((x-mu_x)/std_x, 2) + std::pow((y-mu_y)/std_y, 2)));
      if (multiplier > 0){
        particles[p].weight *= multiplier;
      }

      associations.push_back(minIndex+1);
      sense_x.push_back(x);
      sense_y.push_back(y);
        
        
    }
    weights[p] = particles[p].weight;
    SetAssociations(particles[p], associations, sense_x, sense_y);
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<Particle> new_particles;
  std::uniform_int_distribution<> int_dist(0, num_particles-1);
  int index = int_dist(gen);

  double max_weight = *std::max_element(weights.begin(), weights.end());
  std::uniform_real_distribution<double> real_dist(0.0, 2*max_weight);
  double beta = 0;

  for (int i = 0; i < num_particles; i++) {
    beta += real_dist(gen);
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
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