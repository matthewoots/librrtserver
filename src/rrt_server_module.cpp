/*
 * rrt_server_module.cpp
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

namespace rrt_server
{
    vector<Eigen::Vector3d> rrt_server_node::find_rrt_path(
        vector<Eigen::Vector3d> previous_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl, 
        Eigen::Vector3d start, Eigen::Vector3d end, 
        vector<Eigen::Vector4d> no_fly_zone,
        double min_height, double max_height,
        double step_size, double protected_zone)
    {
        vector<Eigen::Vector3d> path;

        // Find the origin of the transformed frame
        Eigen::Vector3d _origin = (start + end) / 2;

        // Do the preparation for transformation
        // Find the translation vector and the yaw angle
        Eigen::Vector3d tmp_vect = end - start;
        double yaw = atan2(tmp_vect.y(), tmp_vect.x()) / M_PI * 180;

        Eigen::Vector3d rotation = Eigen::Vector3d(0,0,yaw);

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();

        Eigen::Vector3d translation = Vector3d(_origin.x(), _origin.y(), 0);
        Eigen::Vector3d transformed_translation = 
            ru.rotate_translation_with_rpy(rotation, translation);

        transform.translation() = transformed_translation;

        transform.rotate(AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX())
            * AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY())
            * AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ()));

        // Print the transformation
        // std::cout << "[rrt_server_module]" << 
        //    " Affine3d:\n" << transform.matrix() << std::endl;
        
        // We will align in the X axis
        Eigen::Vector3d transformed_start = ru.transform_vector(
            start, rotation, translation, "forward");

        Eigen::Vector3d transformed_end = ru.transform_vector(
            end, rotation, translation, "forward");

        double _xybuffer = 1.0, _zbuffer = 1.0;
        double _passage_size = 10.0; // Additional buffer for the Y axis
        // Map size and origin should be determined and isolated 
        // In this method we can rotate the boundary so that we can minimize the space
        Eigen::Vector3d map_size = Eigen::Vector3d(
            abs(transformed_start.x() - transformed_end.x()) + _xybuffer,
            abs(transformed_start.y() - transformed_end.y()) + _xybuffer + _passage_size,
            abs(transformed_start.z() - transformed_end.z()) + _zbuffer
        );

        int total_points_original, total_points_crop;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped (new pcl::PointCloud<pcl::PointXYZ>());
        // Executing the transformation if there is a pcl
        if(!obs_pcl->empty())
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud (*obs_pcl, *transformed_cloud, transform);

            // We can crop the pointcloud to the dimensions that we are using
            // Origin will already to (0,0,0)
            transformed_cropped = 
                ru.pcl_ptr_box_crop(transformed_cloud, Vector3d(0,0,_origin.z()), map_size);
            
            // For debug purposes
            // Receive the debug cropped and transformed point
            debug_save_local_obs->points.clear();
            debug_save_local_obs->points = transformed_cropped->points;
        
            size_t num_points_original = obs_pcl->size();;
            total_points_original = static_cast<int>(num_points_original);
            size_t num_points_crop = debug_save_local_obs->size();
            total_points_crop = static_cast<int>(num_points_crop);
            
        }
        else 
            total_points_original = total_points_crop = 0;

        // std::cout << "[rrt_server_module] total_points_original " << 
        //         KGRN << total_points_original << KNRM << 
        //         " compared to debug_save_local_obs " <<
        //         KGRN << total_points_crop  << KNRM << std::endl;
        

        std::vector<Eigen::Vector3d> transformed_extracted_path;
        time_point<std::chrono::system_clock> fail_timer_start = system_clock::now();

        rrt_server::rrt_search_node rsn(previous_input);
        rsn.initialize_start_end(transformed_start, transformed_end);
        rsn.initialize_boundaries(min_height, max_height, no_fly_zone);
        rsn.initialize_map_characteristics(transformed_cropped, 
            map_size, Eigen::Vector3d(0,0,_origin.z()));
        rsn.initialize_node_characteristics(0.025, step_size, protected_zone, 
            rotation, translation);

        int iter = 0;
        while (1)
        {
            iter++;
            // time_point<std::chrono::system_clock> module_time_start = system_clock::now();

            vector<Eigen::Vector3d> temporary_path = rsn.run_rrt_module();
            
            // auto test_time_diff = duration<double>(system_clock::now() - module_time_start).count();
            // std::cout << "[rrt_server_module]" << 
            //     " tries(" << iter << ") runtime for module: " << 
            //     KGRN << test_time_diff << KNRM << "s" << std::endl;

            if (!temporary_path.empty())
            {
                transformed_extracted_path = temporary_path;
                break;
            }
            if (duration<double>(system_clock::now() - 
                fail_timer_start).count() > 0.08)
                break;
        }

        if (transformed_extracted_path.empty())
        {
            std::cout << "[rrt_server_module]" << 
                KRED << " error in finding path!" << KNRM << std::endl;
            return path;
        }

        for (int j = transformed_extracted_path.size()-1; j >= 0 ; j--)
        {
            Eigen::Vector3d transformed_vector = transformed_extracted_path[j];
            
            Eigen::Vector3d original_vector = ru.transform_vector(
                transformed_vector, rotation, translation, "backward");

            path.push_back(original_vector);
        }

        return path;
    }
}