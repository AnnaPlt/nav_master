#include "../include/nav_master/pathSampler.h"

namespace path_sampler{
    

PathSampler::PathSampler() {}

PathSampler::~PathSampler() {}


void PathSampler::samplePlan(double sampling_distance)
{
    _local_plan.sampled_local_plan = {};

    int previous_index = 0;
    if (sampling_distance > 0.0f)
    {
        for(int i = 0; i < (int)_local_plan.local_plan.size(); i++) {
            
            if (getDistancePoints(_local_plan.local_plan[i].pose.position, _local_plan.local_plan[previous_index].pose.position) > sampling_distance) {
                _local_plan.sampled_local_plan.push_back(_local_plan.local_plan[i]);
                previous_index = i;
            }
        }
    }

    _local_plan.sampled_local_plan.push_back(_local_plan.local_plan[_local_plan.local_plan.size() - 1]);

}



double PathSampler::getDistancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 ){

    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

nav_msgs::Path PathSampler::getSampledPlan(double sampling_distance, const nav_msgs::Path& original_global_plan)
{
    _local_plan.local_plan = original_global_plan.poses;
    samplePlan(sampling_distance);

    nav_msgs::Path sampled_path;
    sampled_path.header = original_global_plan.header;
    sampled_path.poses = _local_plan.sampled_local_plan;

    return sampled_path;
}

}