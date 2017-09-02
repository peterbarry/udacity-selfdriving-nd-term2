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
        std::normal_distribution<double> distribution_x(0,std[0]);

		std::default_random_engine generator_y;
		std::normal_distribution<double> distribution_y(0,std[1]);

		std::default_random_engine generator_theta;
		std::normal_distribution<double> distribution_theta(0,std[2]);

		std::vector<double>::iterator weights_iterator;


		//weights.reserve(num_particles);
		//cout << "Particle size:" << particles.size() << endl;

		//cout << "Particle size:" << particles.size() << endl;


		cout << "Initialising Particle Filters with " << num_particles << " particles" << endl;
		for (int i=0; i < num_particles; ++i) {
                Particle local_particle;

                local_particle.id = i;
                local_particle.x = x +  distribution_x(generator_x);
                local_particle.y = y + distribution_y(generator_y);
                local_particle.theta = theta + distribution_theta(generator_theta);
                local_particle.weight=1.0;
                particles.push_back(local_particle);
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
    std::default_random_engine gen;

	std::normal_distribution<double> distribution_x(0,std_pos[0]);
	std::normal_distribution<double> distribution_y(0,std_pos[1]);
	std::normal_distribution<double> distribution_theta(0,std_pos[2]);

	//cout << "Prediction  Updates: Velocity:" << velocity << "Delta Time:" << delta_t << "Yaw rate:" << yaw_rate << endl;

	distance = velocity * delta_t; // assume time and distance in same metics.
	theta_delta = yaw_rate * delta_t;

	//cout << "Distance Covered:" << distance << "Angle Change:" << theta_delta << endl;



	for (int i=0; i < num_particles; ++i) {


			//  This assumes bicycle model for vehicle.
			// https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/2c318113-724b-4f9f-860c-cb334e6e4ad7/lessons/5c50790c-5370-4c80-aff6-334659d5c0d9/concepts/ca98c146-ee0d-4e53-9900-81cec2b771f7

                if (fabs(yaw_rate) < 0.000001) {
                    particles[i].x += velocity * delta_t * cos(particles[i].theta);
                    particles[i].y += velocity * delta_t * sin(particles[i].theta);
                    //particles[i].theta += yaw_rate * delta_t; // small update.

                }
                else {
                    particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
                    particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
                    particles[i].theta += yaw_rate * delta_t;
                    //particles.at(i).theta = fmod(particles.at(i).theta , (2 * M_PI) );

                }
                // add noise
                particles[i].x += distribution_x(gen);
                particles[i].y += distribution_y(gen);
                particles[i].theta += distribution_theta(gen);

                //particles[i].theta = fmod(particles[i].theta , (2 * M_PI) );


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

    //cout << "Associate landmark with observasion" << endl;
	for(ob_it=observations.begin() ; ob_it < observations.end(); ob_it++ ) {
		int min_dist= INT_MAX;

		ob_it->id = INT_MAX; // no landmark found.

        //cout << "Observasion (x,y)" <<ob_it->x << "," << ob_it->y << endl;

		for(pred_it=predicted.begin() ; pred_it < predicted.end(); pred_it++ ) {
			int distance = dist(ob_it->x,ob_it->y,pred_it->x,pred_it->y);
			if ( distance < min_dist) {
				// update Id.
                min_dist = distance;
                ob_it->id = pred_it->id;
                //cout << "Updating landmark: id:" << ob_it->id << "Distance:" << distance << endl;

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

    // for each particle...
    for (int i = 0; i < num_particles; i++) {

        // get the particle x, y coordinates
        double point_x = particles[i].x;
        double point_y = particles[i].y;
        double point_theta = particles[i].theta;

        // create a vector to hold the map landmark locations predicted to be within sensor range of the particle
        vector<LandmarkObs> inrange_landmarks;
        vector<LandmarkObs> global_obs;

        // for each map landmark...


        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

            // Check if in range.
            //cout << ".";
            double distance = dist(point_x,point_y,(double)(map_landmarks.landmark_list[j].x_f),(double)(map_landmarks.landmark_list[j].y_f));

            if (distance <= sensor_range) {
                LandmarkObs landmark;

                //todo copy struct generically
                landmark.id = map_landmarks.landmark_list[j].id_i;
                landmark.x = (double)map_landmarks.landmark_list[j].x_f;
                landmark.y = (double)map_landmarks.landmark_list[j].y_f;

                // The landmark is in range of this particle - ignoring uncertainty of measurements
                //cout << endl<< "*** Landmark ID in range:" << landmark_it->id_i << " Range:" << distance << endl;

                //inrange_landmarks.push_back(landmark);
                inrange_landmarks.push_back(LandmarkObs{ map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
            }
            // now landmark_inrange has a list of landmarks in range for this particle. (global)
        }



        // Assume observatons are relative to this particle, update observsastion to global space based on that
        for (unsigned int j = 0; j < observations.size(); j++) {
            LandmarkObs obs;

            //cout << endl << "Procesing Local observations: " << endl << observations_it->id << endl << "Local X:" << observations_it->x << endl << "Local Y:" << observations_it->y << endl;


            double px = (observations[j].x * cos(point_theta)) - (observations[j].y * sin(point_theta)) + point_x;
            double py = (observations[j].x * sin(point_theta)) + (observations[j].y * cos(point_theta)) + point_y;

            obs.id = observations[j].id;
            obs.x = px;
            obs.y = py;

            global_obs.push_back(obs);

        }


        // perform dataAssociation for the predictions and transformed observations on current particle
        dataAssociation(inrange_landmarks, global_obs);

        // reinit weight
        particles[i].weight = 1.0;

        for (unsigned int j = 0; j < global_obs.size(); j++) {

            double land_x=0.0, land_y=0.0;

            int associated_pred = global_obs[j].id;
            // get the x,y coordinates of the prediction associated with the current observation
            for (unsigned int k = 0; k < inrange_landmarks.size(); k++) {
                if (inrange_landmarks[k].id == associated_pred) {
                    land_x = inrange_landmarks[k].x;
                    land_y = inrange_landmarks[k].y;
                }
            }
            // todo - replace search with lookup, but crashing now
            //if ( global_obs[i].id > 0) {
            //    land_x = (double) map_landmarks.landmark_list[(global_obs[i].id) -1].x_f;
            //    land_y = (double) map_landmarks.landmark_list[(global_obs[i].id) -1].y_f;
            //}
            //else {
            //    cout << "ERROR Invalid observ id" << endl;
            //    land_x = land_y = 0.0;
            //}


            // calculate weight for this observation with multivariate Gaussian

            double prob_w ;

            prob_w = multiGausian( global_obs[j].x, land_x, global_obs[j].y, land_y, std_landmark[0], std_landmark[1]);

            // product of this obersvation weight with total observations weight
            particles[i].weight *= prob_w;

            //if (prob_w == 0) {
            //    cout << "Info Probability zero" << endl;
            //}
        }
    }
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
    //std::vector<double>::iterator weights_iterator;

    std::vector<Particle>::iterator particles_it;


    double wTotal= 0.0;
    for ( int i = 0 ; i < num_particles; ++i) {
        weights_distro.push_back(particles[i].weight);
        wTotal+=particles[i].weight;
    }

    // Normalize weights for all particles.
    for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
        // Iterate through all the particles
        particles_it->weight = particles_it->weight / wTotal;

    }

    // Debug check to see of sum to 1.
    double wTotalNorm= 0.0;
    for(particles_it=particles.begin() ; particles_it < particles.end(); particles_it++ ) {
        // Iterate through all the particles
        wTotalNorm += particles_it->weight;
    }
    if (wTotalNorm <= 0.99 ||  wTotalNorm >= 1.01) {
        cout << "Normalised weights do not sum to 1." << endl;
    }



    std::discrete_distribution<> d (weights_distro.begin(),weights_distro.end());

    int i,resample_index;

    std::vector<Particle> new_particles;

    Particle particle;

    memset(&particle,0, sizeof(Particle));

    //cout << "Resampling :" << endl;
    for (i = 0 ; i <NUM_PARTICLES;++i) {
        // Generate new particles
        resample_index = d(gen);

        particle.id = i;
        particle.x = particles[resample_index].x;
        particle.y =particles[resample_index].y;
        particle.theta =particles[resample_index].theta;
        particle.weight = particles[resample_index].weight;
        new_particles.push_back(particle);

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
