/*
 * test_rrt_module.cpp
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

#include "rrt_server_module.h"
#include <cmath>
#include <random>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

int main()
{
    rrt_server::rrt_server_node rrt_server;

    std::random_device dev;
    std::mt19937 generator(dev());
    std::uniform_real_distribution<double> dis_middle(-1.0, 1.0);
    std::uniform_real_distribution<double> dis_normal(0.0, 1.0);
    double map_size = 6.0;
    double _min_height = 0.0;
    double _max_height = 5.0;
    double step_size = 5.0;
    double protected_zone = 0.5;

    bool exit = false;
    int iter = 0;
    time_point<std::chrono::system_clock> runtime_start = system_clock::now();
    while (!exit)
    {
        iter++;
        Eigen::Vector3d start = 
            Eigen::Vector3d(dis_normal(generator) * map_size, dis_normal(generator) * map_size, 
            dis_normal(generator) * map_size);
        Eigen::Vector3d end = 
            Eigen::Vector3d(dis_middle(generator) * map_size, dis_middle(generator) * map_size, 
            dis_normal(generator) * map_size);

        std::cout << "[test_server_module]" << std::endl << "START: " << 
                KGRN << start.transpose() << KNRM << std::endl <<
                "END: " << KGRN << end.transpose() << KNRM << std::endl <<
                "DISTANCE: " << KGRN << (start-end).norm() << KNRM << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl (new pcl::PointCloud<pcl::PointXYZ>());
        vector<Eigen::Vector4d> no_fly_zone;

        vector<Eigen::Vector3d> path = rrt_server.find_rrt_path(
            vector<Eigen::Vector3d>(), obs_pcl, start, end, 
            no_fly_zone, _min_height, _max_height, step_size, protected_zone);

        std::cout << "[test_server_module]" << 
            " total runs(" << KBLU << iter << KNRM << ") and timer(" << KBLU <<
            duration<double>(system_clock::now() - runtime_start).count() <<
             "s" << KNRM << ")"<< std::endl;

        if (path.empty())
            exit = true;
    }

    return 0;
}