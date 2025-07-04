#ifndef PATH_SAMPLER_H_
#define PATH_SAMPLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

namespace path_sampler{


    class PathSampler{
    
        public:
            PathSampler();
            ~PathSampler();
            nav_msgs::Path getSampledPlan(double sampling_distance, const nav_msgs::Path& original_global_plan);

        private:

        struct GlobalPlan // Store the information of the plan
        {
            std::vector<geometry_msgs::PoseStamped> local_plan;  //!< Store the current global plan
            std::vector<geometry_msgs::PoseStamped> sampled_local_plan; //!< Sample in the distance the global plan
            int current_index = -1;
            int prev_index = 0;
        } _local_plan;

        double getDistancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 );
        void samplePlan(double sampling_distance);
    };
};

#endif