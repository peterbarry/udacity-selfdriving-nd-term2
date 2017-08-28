/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *      Added content for Udacity project : Peter Barry.
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include <random>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
		num_particles = NUM_PARTICLES;

		std::default_random_engine generator_x;
        std::normal_distribution<double> distribution_x(x,std[0]);

		std::default_random_engine generator_y;
		std::normal_distribution<double> distribution_y(y,std[1]);

		std::default_random_engine generator_theta;
		std::normal_distribution<double> distribution_theta(theta,std[2]);

		std::vector<Particle>::iterator particles_iterator;
		std::vector<double>::iterator weights_iterator;



		weights.reserve(num_particles);
		cout << "Particle size:" << particles.size() << endl;

		particles.reserve(num_particles);
		cout << "Particle size:" << particles.size() << endl;

		particles_iterator = particles.begin();
		weights_iterator =  weights.begin();

		cout << "Initialising Particle Filters with " << num_particles << " particles" << endl;
		for (int i=0; i < num_particles; ++i) {
			try{
				  struct Particle local_particle;

			    local_particle.id = i;
					local_particle.x =  distribution_x(generator_x);
					local_particle.y = distribution_y(generator_y);
					local_particle.theta = distribution_theta(generator_theta);
					local_particle.weight=1.0;

					particles_iterator = particles.insert(particles_iterator,local_particle);

					weights_iterator = weights.insert(weights_iterator,(double(1)));

					//todo add weights.at(i) = 0;
					//todo: Do I need to add random gausian noise to this too or is the nornal selection enough.

			}
			catch(std::out_of_range o){
			    std::cout<< "Error:" << o.what()<<std::endl;
			}
		}

		cout << "Init completed" << endl;

		cout << "Particle size:" << particles.size() << endl;

		is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	double distance;
	double theta_delta;
	std::default_random_engine generator_x;
	std::normal_distribution<double> distribution_x(0,std_pos[0]);

	std::default_random_engine generator_y;
	std::normal_distribution<double> distribution_y(0,std_pos[1]);

	std::default_random_engine generator_theta;
	std::normal_distribution<double> distribution_theta(0,std_pos[2]);

	cout << "Prediction  Updates: Velocity:" << velocity << "Delta Time:" << delta_t << "Yaw rate:" << yaw_rate << endl;

	distance = velocity * delta_t; // assume time and distance in same metics.
	theta_delta = yaw_rate * delta_t;

	cout << "Distance Covered:" << distance << "Angle Change:" << theta_delta << endl;



	for (int i=0; i < num_particles; ++i) {
		try{

			//  This assumes bicycle model for vehicle.
			//todo : probably wrong if you read: https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/2c318113-724b-4f9f-860c-cb334e6e4ad7/lessons/5c50790c-5370-4c80-aff6-334659d5c0d9/concepts/ca98c146-ee0d-4e53-9900-81cec2b771f7

				particles.at(i).theta += theta_delta + distribution_theta(generator_theta);
				particles.at(i).theta = fmod(particles.at(i).theta , (2 * M_PI) );

				particles.at(i).x = particles.at(i).x + (cos(particles.at(i).theta) * distance);
				particles.at(i).x = particles.at(i).x + distribution_x(generator_x);

				particles.at(i).y = particles.at(i).y + (sin(particles.at(i).theta) * distance);
				particles.at(i).y = particles.at(i).y + distribution_y(generator_y);


		}
		catch(std::out_of_range o){
				std::cout<<o.what()<<std::endl;
		}
	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	// itererate thrtough observations, and assing an Id from the predicted list.
	vector<LandmarkObs>::iterator ob_it;  // declare an iterator for observations
	vector<LandmarkObs>::iterator pred_it;  // declare an iterator for predictions


	for(ob_it=observations.begin() ; ob_it < observations.end(); ob_it++ ) {
		int min_dist= INT_MAX;

		ob_it->id = INT_MAX; // no landmark found.

		for(pred_it=predicted.begin() ; pred_it < predicted.end(); pred_it++ ) {
			int distance = dist(ob_it->x,ob_it->y,pred_it->x,pred_it->y);
			if ( distance < min_dist) {
				// update Id.
                min_dist = distance;
                ob_it->id = pred_it->id;
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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html



		std::vector<Particle>::iterator particles_it;
		std::vector<Map::single_landmark_s>::iterator landmark_it;
		std::vector<LandmarkObs>::iterator landmarks_inrange_it ; // List of landmarks in the map

		std::vector<LandmarkObs> global_observations;
		std::vector<LandmarkObs>::iterator global_observations_it;
		std::vector<LandmarkObs>::iterator observations_it;


		//cout << "Update Weights: Sensor Range: " << sensor_range << endl;

		// GEnerate landmarks wirthin range of vehicle.
		for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
			// Iterate through all the particles
			//cout << "*** Checking Particle ID:" << particles_it->id << endl << "Searching map" << endl;

			landmarks_inrange_it = particles_it->inrange_landmarks.begin();

			for (landmark_it=map_landmarks.landmark_list.begin();
					 landmark_it < map_landmarks.landmark_list.end() ;
					 landmark_it++ ){
					// Check if in range.
					//cout << ".";
					int distance = dist(particles_it->x,particles_it->y,(double)(landmark_it->x_f),(double)(landmark_it->y_f));

					if (distance <= sensor_range) {
						LandmarkObs landmark;

						//todo copy struct generically
						landmark.id = landmark_it->id_i;
						landmark.x = (double)landmark_it->x_f;
						landmark.y = (double)landmark_it->y_f;

						// The landmark is in range of this particle - ignoring uncertainty of measurements
						//cout << endl<< "*** Landmark ID in range:" << landmark_it->id_i << " Range:" << distance << endl;

						landmarks_inrange_it = particles_it->inrange_landmarks.insert(landmarks_inrange_it,landmark);
					}
					// now landmark_inrange has a list of landmarks in range for this particle. (global)
			}



			// Assume observatons are relative to this particle, update observsastion to global space based on that
			global_observations_it = global_observations.begin();
			for ( observations_it = observations.begin() ; observations_it < observations.end(); observations_it++) {
				LandmarkObs global_obs;

				//cout << endl << "Procesing Local observations: " << endl << observations_it->id << endl << "Local X:" << observations_it->x << endl << "Local Y:" << observations_it->y << endl;

				double theta=particles_it->theta;
				double delta_x=particles_it->x;
				double delta_y=particles_it->y;
				double x=0;
				double y=0;

				x = (observations_it->x * cos(theta)) - (observations_it->y * sin(theta)) + delta_x;
				y = (observations_it->x * sin(theta)) + (observations_it->y * cos(theta)) + delta_y;


				global_obs.id = observations_it->id;
				global_obs.x = x;
				global_obs.y = y;

				global_observations_it = global_observations.insert(global_observations_it,global_obs);

			}

			// Here we have a list of global_observations update to global space.
			// and we have a list of inrange_landmarks in global address space.
			dataAssociation(particles_it->inrange_landmarks,global_observations);
            //cout << endl << "Particle Associations complete - nearest neighbour " << endl ;
            


            // Iterate over associations - global observations now updated  with landmark id.
            for ( global_observations_it = global_observations.begin() ; global_observations_it < global_observations.end(); global_observations_it++) {
                double obs_x=0.0;
                double obs_y=0.0;
                double landmark_x=0.0;
                double landmark_y=0.0;
                double Px;
                
                obs_x=global_observations_it->x;
                obs_y=global_observations_it->y;
                
                if ( global_observations_it->id > 0) {
                    landmark_x = (double) map_landmarks.landmark_list[(global_observations_it->id) -1].x_f;
                    landmark_y = (double) map_landmarks.landmark_list[(global_observations_it->id) -1].y_f;
                }
                
                Px = multiGausian( obs_x, landmark_x, obs_y, landmark_y, std_landmark[0], std_landmark[1]);
                particles_it->weight *= Px;
                
            }
            // Particle have an update un normalised weight 
            //cout << endl << "Particle  complete - weight " << endl ;

		}

        //cout << endl << "Particles  complete - un normalised weights " << endl ;

    
        // Normalize weights for all particles.
    
        double wTotal= 0.0;
        for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
            // Iterate through all the particles
            wTotal += particles_it->weight;
        }
    
        for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
            // Iterate through all the particles
            particles_it->weight = particles_it->weight / wTotal;
  
        }
        //cout << endl << "Particles  complete - normalised weights " << endl ;

        // Debug check to see of sum to 1.
        double wTotalNorm= 0.0;
        for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
            // Iterate through all the particles
            wTotalNorm += particles_it->weight;
        }

        if ( wTotalNorm != 1)
            cout << "ERROR: Total weight: should be 1: but is " << wTotalNorm;
    



}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    
    // Create a vector of distributions
    // Setup the random bits
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<double> weights_distro;
    std::vector<double>::iterator weights_iterator;

    std::vector<Particle>::iterator particles_it;

    weights_iterator = weights_distro.begin();
    for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
        weights_iterator = weights_distro.insert(weights_iterator,(particles_it->weight));
       // weights_distros[i++] = particles_it->weight;
    }
    
    std::discrete_distribution<> d (weights_distro.begin(),weights_distro.end());
    
    int i,resample_index;
    
    std::vector<Particle> new_particles;
    std::vector<Particle>::iterator new_particles_iterator;
    
    Particle particle;
    
    memset(&particle,0, sizeof(Particle));
    new_particles_iterator = new_particles.begin();

    //cout << "Resampling :" << endl;
    for (i = 0 ; i <NUM_PARTICLES;++i) {
        // Generate new particles
        resample_index = d(gen);
        
        particle.id = i;
        particle.x = particles[resample_index].x;
        particle.y =particles[resample_index].y;
        particle.theta =particles[resample_index].theta;
        particle.weight = 1.0;
        new_particles_iterator = new_particles.insert(new_particles_iterator,(particle));

        //cout << resample_index << endl;
    }
    
    // new particles list.
    particles  = new_particles;
   //cout << "Resampling End:" << endl;


    

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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
