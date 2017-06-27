
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
#include "particle_filter.h"
using namespace std;




const int  NumParticles   = 300;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  default_random_engine gen;
  
  num_particles=NumParticles;
  normal_distribution<double>dist_x(x, std[0]);
  normal_distribution<double>dist_y(y,std[1]);
  normal_distribution<double>dist_theta(theta,std[2]);
  for(unsigned int i=0;i<num_particles;++i){
    Particle particle;
    particle.id=i;
    particle.x=dist_x(gen);
    particle.y=dist_y(gen);
    particle.theta=dist_theta(gen);
    particle.weight=1.0;
    particles.push_back(particle);
    
  }
  
  is_initialized=true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  default_random_engine gen;
  normal_distribution<double>dist_x(0,std_pos[0]);
  normal_distribution<double>dist_y(0,std_pos[1]);
  normal_distribution<double>dist_theta(0,std_pos[2]);
  //reset particles for each prediction
  particles.clear();
  for(unsigned int i=0;i<NumParticles;++i){
    Particle p =particles[i];
    if(fabs(yaw_rate)>1e-5){
      p.x=p.x+(sin(p.theta+yaw_rate*delta_t)-sin(p.theta))*(velocity/yaw_rate);
      
      p.y=p.y+(cos(p.theta)-cos(p.theta+yaw_rate*delta_t))*(velocity/yaw_rate);
    }else{
      p.x=p.x+cos(p.theta)*velocity*delta_t;
      p.y=p.y+sin(p.theta)*velocity*delta_t;
    }
    
    p.theta=p.theta+(yaw_rate*delta_t);
    p.x+=dist_x(gen);
    p.y+=dist_y(gen);
    p.theta+=dist_theta(gen);
    particles.push_back(p);
    
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  
  // set this value large enough to help find the minimum next
  double default_max = numeric_limits<double>::max();
  
  
  //for each observation find the closest landmark from the list of predicted landmarks and store its index in the array of predicted landmakrs
  for(auto &obs : observations)
  {
    double min_dist = default_max;
    
    for(int i = 0; i < predicted.size(); i++)
    {
      LandmarkObs &pred = predicted[i];
      
      double dist = sqrt(pow(pred.x - obs.x,2)+pow(pred.y - obs.y,2.0));
      if(dist < min_dist)
      {
        //assign the closest landmark's index to each observation
        obs.id = i;
        min_dist = dist;
      }
    }
  }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
  
  weights.clear();
  
  for(auto &par:particles){
    
    
    vector<LandmarkObs> glob_observations;
    for (auto &obs : observations)
    {
      LandmarkObs global_obs;
      
      // set each landmark's id as -1 to ensure each one would has same id.
      global_obs.id = -1;
      
      // rotate and shift
      global_obs.x = obs.x * cos(par.theta) - obs.y * sin(par.theta) + par.x;
      global_obs.y = obs.x * sin(par.theta) + obs.y * cos(par.theta) + par.y;
      
      glob_observations.push_back(global_obs);
    }
  
    
    
    //loop over each landmark to find the closest one and assign the observed measurement to that predict landmarks
    vector<LandmarkObs>predict_landmarks;

      for(int m=0;m<map_landmarks.landmark_list.size();++m){
        double dist=sqrt(pow(par.x - map_landmarks.landmark_list[m].x_f,2.0)+pow(par.y- map_landmarks.landmark_list[m].y_f,2.0));
        if(dist<=sensor_range){
          LandmarkObs landmark;
          landmark.id=map_landmarks.landmark_list[m].id_i;
          landmark.x=map_landmarks.landmark_list[m].x_f;
          landmark.y=map_landmarks.landmark_list[m].y_f;
          predict_landmarks.push_back(landmark);
        }
      }
    
    
    double w;
    
    //deal with the case that when some impossible predictions or no predictions exist.
    if((predict_landmarks.size() == 0 && glob_observations.size() > 0) ||
       (glob_observations.size() == 0 && predict_landmarks.size() > 0))
    {
      w = 1e-20;
    }
    else
    {
      w = 1;
       //call dataAssociation method to get closest glob_observation's id and then assign the values of predict landmark in that id to a particular particle
      dataAssociation(predict_landmarks, glob_observations);
      
      // find probability of all observations using gaussian distribution
      for (auto &o: glob_observations)
      {
        // in case assosiated landmark was found
        if (o.id >= 0)
        {
          LandmarkObs &ldk = predict_landmarks[o.id];
          long double exp_form=-0.5*(((pow(ldk.x-o.x,2.0)/(std_landmark[0]*std_landmark[0])))+((pow(ldk.y-o.y,2.0)/(std_landmark[1]*std_landmark[1]))));
          
          long double update_w=exp(exp_form)/sqrt(2*M_PI*std_landmark[0]*std_landmark[1]);
          w*=update_w;
        }
      }
    }
    
    
    par.weight=w;
    weights.push_back(w);
  
    
    }//end loop for updated each particle's weight
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  
  double weightSum=0.0;
  //double weightSquareSum=0.0;
  for (unsigned int i =0; i<num_particles;++i){
    
    weightSum=weightSum+weights[i];
    //weightSquareSum+=weights[i]*weights[i];
  }
  
  //initialize the step and current position on the wheel
  double step=weightSum/num_particles;
  discrete_distribution<double>distribution(weights.begin(),weights.end());
  default_random_engine gen;
  double position=distribution(gen);
  
  double csum[1000];
  csum[0]=weights[0];
  for(int j=0;j<weights.size();++j){
    //add current sum of weights to the current particle's weight
    csum[j]=csum[j-1]+weights[j];
  }
  
  
  int idx=0;
  vector<Particle>outpars;
  //walk along the wheel to select the particles
  for(int k=0;k<num_particles;++k){
    //Particle out;
    position=position+step;
    while(position>weightSum){
      position=position-weightSum;//rotate a new wheel
      idx=0;
    }
    while(position>csum[idx]){
      idx=(idx+1)%num_particles;
    }
//    out.x=particles[idx].x;
//    out.y=particles[idx].y;
//    out.theta=particles[idx].theta;
    outpars.push_back(particles[idx]);
  }
  particles=outpars;
  
}

//void ParticleFilter::write(std::string filename) {
//	// You don't need to modify this file.
//	std::ofstream dataFile;
//	dataFile.open(filename, std::ios::app);
//	for (int i = 0; i < num_particles; ++i) {
//		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
//	}
//	dataFile.close();
//}
