/*
 * rrt_server_module.h
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#ifndef RRT_SERVER_MODULE_H
#define RRT_SERVER_MODULE_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <random>
#include <chrono>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "rrt_search_module.h"

using namespace Eigen;
using namespace std;
using namespace std::chrono;

#define dmax std::numeric_limits<double>::max();

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace rrt_server
{
    class rrt_server_node
    {
        private:
            rrt_server::rrt_utility ru; 

            double _min_height, _max_height;
            double _protected_zone, _sub_runtime_error, _runtime_error;
            vector<Eigen::Vector4d> _no_fly_zone;

        public:
            pcl::PointCloud<pcl::PointXYZ>::Ptr debug_save_local_obs;

            rrt_server_node()
            {
                debug_save_local_obs = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>());
            }

            ~rrt_server_node(){}

            vector<Eigen::Vector3d> find_rrt_path(
                vector<Eigen::Vector3d> previous_input, pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl,
                Eigen::Vector3d start, Eigen::Vector3d end, double step_size);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr get_debug_local_pcl()
            {return debug_save_local_obs;}

            void reset_parameters(vector<Eigen::Vector4d> no_fly_zone,
                double min_height, double max_height, double protected_zone,
                double sub_runtime_error, double runtime_error)
            {
                no_fly_zone.clear();
                _no_fly_zone = no_fly_zone;
                _min_height = min_height;
                _max_height = max_height;
                _protected_zone = protected_zone;

                _sub_runtime_error = sub_runtime_error;
                _runtime_error = runtime_error;
            }

    };
}

#endif