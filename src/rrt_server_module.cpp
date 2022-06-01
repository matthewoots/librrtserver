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
        vector<Eigen::Vector3d> previous_input, pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl, 
        Eigen::Vector3d start, Eigen::Vector3d end, double step_size)
    {
        vector<Eigen::Vector3d> path;

        // Find the origin of the transformed frame
        Eigen::Vector3d _origin = (start + end) / 2;

        // Do the preparation for transformation
        // Find the translation vector and the yaw angle
        Eigen::Vector3d vect = end - start;
        Eigen::Vector3d rot_vec = vect / vect.norm();

        Eigen::Vector3d rotation = ru.deg_euler_rotation_pitch_yaw(rot_vec);
        
        // We will align in the X axis
        Eigen::Vector3d transformed_start = ru.transform_vector(
            start, rotation, _origin, "forward");

        Eigen::Vector3d transformed_end = ru.transform_vector(
            end, rotation, _origin, "forward");

        /** @brief Print the transformed start and end points */
        std::cout << "transformed_start and transformed_end positions \n[" << transformed_start.transpose() <<
                 "] [" << transformed_end.transpose() << "]" << std::endl;

        double _xybuffer = 0.0, _zbuffer = 3.0;
        double _passage_size = 10.0; // Additional buffer for the Y axis
        // Map size and origin should be determined and isolated 
        // In this method we can rotate the boundary so that we can minimize the space
        Eigen::Vector3d map_size = Eigen::Vector3d(
            abs(transformed_start.x() - transformed_end.x()) + _xybuffer,
            abs(transformed_start.y() - transformed_end.y()) + _xybuffer + _passage_size,
            abs(transformed_start.z() - transformed_end.z()) + _zbuffer
        );

        Eigen::Affine3d cloud_transform = 
            ru.get_point_cloud_transform(rotation, _origin, "forward");

        /** @brief Print the transformation */
        // std::cout << "[rrt_server_module]" << 
        //    " Affine3d:\n" << cloud_transform.matrix() << std::endl;

        int total_points_original, total_points_crop;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped (new pcl::PointCloud<pcl::PointXYZ>());
        // Executing the transformation if there is a pcl
        if(!obs_pcl->empty())
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud (*obs_pcl, *transformed_cloud, cloud_transform);

            // We can crop the pointcloud to the dimensions that we are using
            // Origin will already to (0,0,0)
            transformed_cropped = 
                ru.pcl_ptr_box_crop(transformed_cloud, Vector3d(0,0,0), map_size);
            
            // For debug purposes
            // Receive the debug cropped and transformed point
            if (!debug_save_local_obs->empty())
                debug_save_local_obs->points.clear();
            pcl::PointCloud<pcl::PointXYZ>::Ptr debug_save_local_obs_transform (new pcl::PointCloud<pcl::PointXYZ>());
            debug_save_local_obs_transform->points = transformed_cropped->points;
            
            Eigen::Affine3d cloud_reverse_transform = 
                ru.get_point_cloud_transform(rotation, _origin, "backward");
            
            pcl::transformPointCloud (
                *debug_save_local_obs_transform, *debug_save_local_obs, 
                cloud_reverse_transform);
        
            size_t num_points_original = obs_pcl->size();;
            total_points_original = static_cast<int>(num_points_original);
            if (!debug_save_local_obs->empty())
            {
                size_t num_points_crop = debug_save_local_obs->size();
                total_points_crop = static_cast<int>(num_points_crop);
            }
            else
                total_points_crop = 0;
            
        }
        else 
            total_points_original = total_points_crop = 0;

        std::cout << "[rrt_server_module] total_points_original " << 
                KGRN << total_points_original << KNRM << 
                " compared to debug_save_local_obs " <<
                KGRN << total_points_crop  << KNRM << std::endl;
        

        std::vector<Eigen::Vector3d> transformed_extracted_path;
        time_point<std::chrono::system_clock> fail_timer_start = system_clock::now();

        rrt_server::rrt_search_node rsn(previous_input);
        rsn.initialize_start_end(transformed_start, transformed_end);
        rsn.initialize_boundaries(_min_height, _max_height, _no_fly_zone);
        rsn.initialize_map_characteristics(transformed_cropped, 
            map_size, Eigen::Vector3d::Zero());
        rsn.initialize_node_characteristics(_sub_runtime_error, step_size, _protected_zone, 
            rotation, _origin);

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
                fail_timer_start).count() > _runtime_error)
                break;
        }

        if (transformed_extracted_path.empty())
        {
            std::cout << "[rrt_server_module]" << 
                KRED << " error in finding path!" << KNRM << std::endl;
            return path;
        }

        for (int j = 0; j < transformed_extracted_path.size() ; j++)
        {
            Eigen::Vector3d transformed_vector = transformed_extracted_path[j];
            
            Eigen::Vector3d original_vector = ru.transform_vector(
                transformed_vector, rotation, _origin, "backward");

            path.push_back(original_vector);
        }

        return path;
    }
}