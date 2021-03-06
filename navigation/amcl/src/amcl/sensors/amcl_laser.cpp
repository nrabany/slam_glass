/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
// Modified by Nicolas Rabany 2018.12.05
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include "amcl/sensors/amcl_laser.h"

using namespace amcl;

#include <iostream>
#include <fstream>
using namespace std;
#include "ros/ros.h"

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t *map) : AMCLSensor(),
                                                     max_samples(0), max_obs(0),
                                                     temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser()
{
  if (temp_obs)
  {
    for (int k = 0; k < max_samples; k++)
    {
      delete[] temp_obs[k];
    }
    delete[] temp_obs;
  }
}

void AMCLLaser::SetModelBeam(double z_hit,
                             double z_short,
                             double z_max,
                             double z_rand,
                             double sigma_hit,
                             double lambda_short,
                             double chi_outlier,
                             localization_method_t localization_method_type,
                             int thresh_val,
                             double Kp,
                             double sigma_hit_behind)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
  this->localization_method = localization_method_type;
  this->thresh_val = thresh_val;
  this->Kp = Kp;
  this->sigma_hit_behind = sigma_hit_behind;
}

void AMCLLaser::SetModelLikelihoodField(double z_hit,
                                        double z_rand,
                                        double sigma_hit,
                                        double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void AMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
                                            double z_rand,
                                            double sigma_hit,
                                            double max_occ_dist,
                                            bool do_beamskip,
                                            double beam_skip_distance,
                                            double beam_skip_threshold,
                                            double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if (this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModelProb, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t *set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  cells_index_t cells_index;
  double p_glass;
  double inc_angle = -1;
  double std_glass;
  double p_can_see;
  double map_range_behind;
  double z_behind;

  self = (AMCLLaser *)data->sensor;

  vector<double> p_vector; 

  p_vector.reserve(self->max_beams);

  double psee, phit, obsAlpha;

  total_weight = 0.0;

  double first_range, second_range, laser_range, prob, i_angle = -1;

  //////////////////////////////////7
  // uint64_t nseconds2 = ros::Time::now().toNSec();
  // ofstream myfile2;
  // myfile2.open(pathPosAll.c_str(), ios::out | ios::app);
  // myfile2 << nseconds2;
  /////////////////////////////////
  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;
    
    ////////////////////////////////
    // myfile2 << " " << pose.v[0] << " " << pose.v[1] << " " << pose.v[2];
    ////////////////////////////////

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);
    
    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);

    int index = 0;
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      if(j==0)
        p_vector[index] = 0.0;

      if(!(obs_range == obs_range))
      {
        //cout << "obs==obs " << endl;
        index += 1;
        continue;
      }

      // Here uncomment to consider only last third of the nb of beams
      // if(i < data->range_count*2.0/3.0)
      // {
      //   p_vector[index] = 0.0;
      //   index += 1;
      //   continue;
      // }

      // Variable used for Adaptive glass method. Allows to discard some measurement if they are at limit between glass and wall.
      bool obs_consider = true;

      // Compute the range according to the map
      if (self->localization_method == STANDARD)
        // map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
        //                          pose.v[2] + obs_bearing, data->range_max);
        cells_index = map_find_cells(self->map, pose.v[0], pose.v[1],
                                   pose.v[2] + obs_bearing, data->range_max);
        map_range = compute_range(self->map, pose.v[0], pose.v[1],
                                cells_index.i_first, cells_index.j_first, data->range_max);
      
      if (self->localization_method == FIXED_THRESH)
      {
        cells_index = map_find_cells(self->map, pose.v[0], pose.v[1],
                                   pose.v[2] + obs_bearing, data->range_max);
        map_range = compute_range(self->map, pose.v[0], pose.v[1],
                                cells_index.i_first, cells_index.j_first, data->range_max);
        map_range_behind = compute_range(self->map, pose.v[0], pose.v[1],
                                         cells_index.i_second, cells_index.j_second, data->range_max);
        p_glass = get_glass_prob(self->map, cells_index.i_first, cells_index.j_first);
        inc_angle = compute_incindent_angle(self->map, pose.v[2] + obs_bearing, cells_index.i_first, cells_index.j_first, 20);
        std_glass = compute_std(inc_angle, map_range);
        p_can_see = compute_p_can_see_thresh(inc_angle, map_range, self->thresh_val);
        if(p_glass==-1)
          p_glass=0;
      }
      if (self->localization_method == ADAPTIVE)
      {
        cells_index = map_find_cells(self->map, pose.v[0], pose.v[1],
                                   pose.v[2] + obs_bearing, data->range_max);
        map_range = compute_range(self->map, pose.v[0], pose.v[1],
                                cells_index.i_first, cells_index.j_first, data->range_max);
        map_range_behind = compute_range(self->map, pose.v[0], pose.v[1],
                                         cells_index.i_second, cells_index.j_second, data->range_max);
        p_glass = get_glass_prob(self->map, cells_index.i_first, cells_index.j_first);
        inc_angle = compute_incindent_angle(self->map, pose.v[2] + obs_bearing, cells_index.i_first, cells_index.j_first, 5);
        std_glass = compute_std(inc_angle, map_range);
        p_can_see = compute_p_can_see_thresh(inc_angle, map_range, angle_thresh);
        //p_can_see = compute_p_can_see_range(inc_angle, map_range, angle_thresh);
        if(p_glass==-1)
          p_glass=0;

        // In a window of nb_near pixels around the first obstacle's cell. Check if there is a non-glass obstacle.
        // If yes --> then set obs_consider to false and don't consider this measurement for adapting the threshold value in the Adaptive method. 
        int nb_near = 2;
        for(int ii=-nb_near; ii<=nb_near; ii++)
        {
          for(int jj=-nb_near; jj<=nb_near; jj++)
          {
            if(i+ii>=0 && i+ii<self->map->size_x && j+jj>=0 && j+jj<self->map->size_y)
            {
              double p_glass_near = get_glass_prob(self->map, cells_index.i_first+ii, cells_index.j_first+jj);
              if(p_glass_near<0.2 && p_glass_near!=-1)
                {
                  obs_consider = false;
                  // Test to try if when considering glass beside wall as glass
                  // p_glass = 0.2;
                }
            }
          }
        }
      }
      pz = 0.0;

      // Part 1a: good, but noisy, hit non glass
      z = obs_range - map_range;
      if (self->localization_method == STANDARD)
        pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));
      if (self->localization_method == FIXED_THRESH)
      {
        pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit)) * (1 - p_glass);
        // Part 1b: good, but noisy, hit glass
        pz += self->z_hit * exp(-(z * z) / (4 * self->sigma_hit * self->sigma_hit)) * p_glass * p_can_see;

        // Part 1c: good, but noisy, hit behind
        z_behind = obs_range - map_range_behind;
        pz += self->z_hit * exp(-(z_behind * z_behind) / (2 * self->sigma_hit_behind * self->sigma_hit_behind)) * p_glass * (1 - p_can_see);
      }
      if (self->localization_method == ADAPTIVE)
      {
        pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit)) * (1 - p_glass);
        // Part 1b: good, but noisy, hit glass
        pz += self->z_hit * exp(-(z * z) / (4 * self->sigma_hit * self->sigma_hit)) * p_glass * p_can_see;

        // Part 1c: good, but noisy, hit behind
        z_behind = obs_range - map_range_behind;
        pz += self->z_hit * exp(-(z_behind * z_behind) / (2 * self->sigma_hit_behind * self->sigma_hit_behind)) * p_glass * (1 - p_can_see);
      
        // Adaptive Method, a P controller is used for getting to the right value of angle_thresh.
        uint64_t seconds = ros::Time::now().toSec();
        // if(seconds > 1547626969+60)
        //   Kp = Kp/5;
        // Values of extrema that bound the error
        double err_bound = 15;
        double err_min = 0;

        if(p_glass > 0.4 && obs_consider && abs(map_range - map_range_behind) > 0.3)
        {
          // Here if angle > alpha_thresh and detect glass
          if(abs(z) < 2*self->sigma_hit && p_can_see < 0.2)
          {
            double err = inc_angle/M_PI*180.0 - angle_thresh;
            // Bound the error
            if(err > err_bound)
              err = err_bound;
            if(err < err_min)
              err = err_min;
            angle_thresh += self->Kp*err;
          }
          // Here if angle < alpha_thresh and don't detect glass
          else if(abs(z_behind) < 2*self->sigma_hit && p_can_see > 0.8)
          {
            double err = inc_angle*M_PI/180.0 - angle_thresh;
            // Bound the error
            if(err < -err_bound)
              err = -err_bound;
            if(err > err_min)
              err = -err_min;
            if(abs(err) > err_min)
              angle_thresh += self->Kp*err;
          } 
        }
      }

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short * obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if (obs_range < data->range_max)
        pz += self->z_rand * 1.0 / data->range_max;

      // TODO: outlier rejection for short readings
      if (pz > 1.0 || !(pz==pz) || pz<0.0)
      {
        // cout << "inc_angle: " << inc_angle << " , range: " << map_range << " " << endl;
        // cout << "p_glass: " << p_glass << " , p_can_see: " << p_can_see << " " << endl;
        if(self->localization_method == ADAPTIVE || self->localization_method == FIXED_THRESH)
        {
          cout << "pz_hit: " << self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit)) * (1-p_glass) << " " << endl;
          cout << "pz_hit_glas: " << self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit)) * p_glass * p_can_see << " " << endl;
          cout << "pz_hit_behind: " << self->z_hit * exp(-(z_behind * z_behind) / (2 * 4 * self->sigma_hit * self->sigma_hit)) * p_glass * (1 - p_can_see) << " " << endl;
          cout << "exp: " << exp(-(z_behind * z_behind) / (2 * 4 * self->sigma_hit * self->sigma_hit)) << " " << endl;
          cout << "p_glass: " << p_glass << " " << endl;
          cout << "p_can_see: " << p_can_see << " " << endl;
        }
        else
        {
          cout << "pz_hit: " << self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit)) << " " << endl;
        }
        
        cout << "pz_short: " << self->z_short * self->lambda_short * exp(-self->lambda_short * obs_range) << " " << endl;
        cout << "pz_max: " << self->z_max * 1.0 << " " << endl;
        cout << "pz_rand: " << self->z_rand * 1.0 / data->range_max << " " << endl;
        cout << "pz: " << pz << " " << endl;

      }

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      p_vector[index] = (p_vector[index]*j + pz) / (j+1);
      index += 1;

      // p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;

      // if(index==3+1 && self->localization_method != STANDARD)
      // {
      //   first_range = map_range;
      //   second_range = map_range_behind;
      //   laser_range = obs_range;
      //   prob = p_glass;
      //   i_angle = inc_angle;
      //   // i_angle = obs_bearing;
      //   if(i_angle>0)
      //     i_angle = i_angle * 180.0 / 3.14;
      //   psee = p_can_see;
      //   phit = self->z_hit * exp(-(z_behind * z_behind) / (2 * self->sigma_hit * self->sigma_hit));
      // }
    }

    sample->weight *= pow(p, 0.8);
    total_weight += sample->weight;

  }
  // cout << "1st: " << first_range << ", 2nd: " << second_range << ", laser: " << laser_range << " " << endl;
  // cout << "p_glass: " << prob << ", inc angle: " << i_angle << " " << endl;
  // cout << "from cells: " << first_range << ", from range: " << rangeTestKeep << " " << endl;

  if(save_file)
  {
    //Write pz to file
    uint64_t nseconds = ros::Time::now().toNSec();
    ofstream myfile;
    myfile.open(pathProb.c_str(), ios::out | ios::app);
    myfile << nseconds;
    for(int ii=0; ii<self->max_beams; ii++)
    {
      myfile << " " << p_vector[ii];
    }
    myfile << "\n";
    
    ofstream myfile2;
    myfile2.open(pathThresh, ios::out | ios::app);
    myfile2 << nseconds << " " << angle_thresh << "\n";
  }

  return (total_weight);
}

double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t *set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser *)data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0 / data->range_max;

    step = (data->range_count - 1) / (self->max_beams - 1);

    // Step size must be at least 1
    if (step < 1)
      step = 1;

    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max)
        continue;

      // Check for NaN
      if (obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if (!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return (total_weight);
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data, pf_sample_set_t *set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser *)data->sensor;

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams));

  // Step size must be at least 1
  if (step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0 / data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles
  //such as humans

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;

  //we only do beam skipping if the filter has converged
  if (do_beamskip && !set->converged)
  {
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles)
  bool *obs_mask = new bool[self->max_beams]();

  int beam_ind = 0;

  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping
  bool realloc = false;

  if (do_beamskip)
  {
    if (self->max_obs < self->max_beams)
    {
      realloc = true;
    }

    if (self->max_samples < set->sample_count)
    {
      realloc = true;
    }

    if (realloc)
    {
      self->reallocTempData(set->sample_count, self->max_beams);
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max)
      {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range)
      {
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if (!MAP_VALID(self->map, mi, mj))
      {
        pz += self->z_hit * max_dist_prob;
      }
      else
      {
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
        if (z < beam_skip_distance)
        {
          obs_count[beam_ind] += 1;
        }
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings

      if (!do_beamskip)
      {
        log_p += log(pz);
      }
      else
      {
        self->temp_obs[j][beam_ind] = pz;
      }
    }
    if (!do_beamskip)
    {
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }

  if (do_beamskip)
  {
    int skipped_beam_count = 0;
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
    {
      if ((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold)
      {
        obs_mask[beam_ind] = true;
      }
      else
      {
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to
    //the right solution
    bool error = false;

    if (skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold))
    {
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true;
    }

    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
      {
        if (error || obs_mask[beam_ind])
        {
          log_p += log(self->temp_obs[j][beam_ind]);
        }
      }

      sample->weight *= exp(log_p);

      total_weight += sample->weight;
    }
  }

  delete[] obs_count;
  delete[] obs_mask;
  return (total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs)
  {
    for (int k = 0; k < max_samples; k++)
    {
      delete[] temp_obs[k];
    }
    delete[] temp_obs;
  }
  max_obs = new_max_obs;
  max_samples = fmax(max_samples, new_max_samples);

  temp_obs = new double *[max_samples]();
  for (int k = 0; k < max_samples; k++)
  {
    temp_obs[k] = new double[max_obs]();
  }
}
