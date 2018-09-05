/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle_filter.h"

using namespace std;
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 50;
	//default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	for (int i = 0; i < num_particles; ++i) {
		 Particle p = {};
		 p.x = dist_x(gen);
		 p.y = dist_y(gen);
		 p.theta = dist_theta(gen);	 
		 p.weight = 1;
		 particles.push_back(p);
		 weights.push_back(1);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//default_random_engine gen;
	// This line creates a normal (Gaussian) distribution for x
	 
	for (int i = 0; i < num_particles; ++i) {
	
		 double new_x;
		 double new_y;
		 double new_theta;

		 if(yaw_rate == 0){
	      //moving straight
		      new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
		      new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
		      new_theta = particles[i].theta;
		    } else { 
		      new_x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			  new_y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			  new_theta = particles[i].theta +  yaw_rate*delta_t;

			}
		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);	

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);	
	}
}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

//}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	

	for (int j = 0; j < num_particles ; ++j) {

		double x_part = particles[j].x;
		double y_part = particles[j].y;
		double theta =  particles[j].theta;

		// Predicated landmarks for each particle
		vector<LandmarkObs> predictions;  
		for (int n = 0; n < map_landmarks.landmark_list.size(); ++n)
		{
			Map::single_landmark_s landmark = map_landmarks.landmark_list[n];
			double d = dist(landmark.x_f, landmark.y_f, x_part, y_part);      
		     if(d <= sensor_range){	
					LandmarkObs new_obs = {};
					new_obs.id = landmark.id_i;
					new_obs.x  = landmark.x_f;
					new_obs.y  = landmark.y_f;
					predictions.push_back(new_obs);
			}
		}
		
		// Transformation of noisy observation into map cordinates
		vector<LandmarkObs> particle_observations;    

		for (int i = 0; i < observations.size(); ++i) {
			LandmarkObs landmark = {};
			double x_obs= observations[i].x; 
			double y_obs= observations[i].y; 

			//transform to map x coordinate
			landmark.x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);

			//transform to map y coordinate
			landmark.y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			landmark.id = observations[i].id;
		
			particle_observations.push_back(landmark);
		}

		//Association of each transformed observation to the closest landmark
		for (int i = 0; i < particle_observations.size(); ++i) {
			int id = -1;
			double shortdistance = numeric_limits<double>::max();  
			for (int n = 0; n < predictions.size(); ++n)
			{
				double distance = dist(particle_observations[i].x,particle_observations[i].y,predictions[n].x,predictions[n].y);

				if(shortdistance > distance)
				{
					shortdistance = distance;
					id = predictions[n].id;
				}
			}
			particle_observations[i].id = id;	
		}
		
		// Calculate particles weight using multi variate Guassian proba. distribution fn.
		particles[j].weight = 1.0;

		for (int i = 0; i < particle_observations.size(); ++i) {
			int association = particle_observations[i].id;
			double meas_x = particle_observations[i].x; 
			double meas_y = particle_observations[i].y; 
			double mu_x, mu_y ;
			if(association!=0)
			{
				for (unsigned int k = 0; k < predictions.size(); k++) {
        		if (predictions[k].id == association) {
          				mu_x = predictions[k].x;
          				mu_y = predictions[k].y;
        			}
        		}
				long double multiplier = 1/(2*M_PI*std_landmark[0]*std_landmark[1])*exp(-(pow((meas_x-mu_x),2)/(2*std_landmark[0]*std_landmark[0]) + pow((meas_y-mu_y),2)/(2*std_landmark[1]*std_landmark[1])));
				if(multiplier > 0)
				{
					particles[j].weight *= multiplier;
				}
			}
		}	
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//Get weights and max weight.

  //temp vectors for the new set of resampled particles
  vector<Particle> new_particles;

  // get all the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index 
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
