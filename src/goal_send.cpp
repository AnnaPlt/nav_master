#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h> // o altro tipo se goal_reached è diverso
#include <vector>
#include <sensor_msgs/Joy.h>

class PlanFollower
{
public:
    PlanFollower(ros::NodeHandle& nh)
    {
        pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
        sub_joy_ = nh.subscribe("joy", 1, &PlanFollower::joyCallback, this);
        sub_goal_reached_ = nh.subscribe("goal_reached", 1, &PlanFollower::goalReachedCallback, this);
        current_goal_idx_ = 0;

        // Inizializza il piano manualmente coi punti che mi hai dato
        initPlan();
    }

private:
    ros::Publisher pub_goal_;
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_goal_reached_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    size_t current_goal_idx_;

    void initPlan()
    {
        ros::Time now = ros::Time::now();

        geometry_msgs::PoseStamped p;


        p.header.seq = 1;
        p.header.frame_id = "odom";
        p.pose.position.x = 2.959700107574463;
        p.pose.position.y = 1.5337283611297607;
        p.pose.position.z = 0.0013933181762695312;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
        plan_.push_back(p);

        p.header.seq = 2;
        p.header.frame_id = "odom";
        p.pose.position.x = 4.507513523101807;
        p.pose.position.y = -0.013081401586532593;
        p.pose.position.z = 0.004578590393066406;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
        plan_.push_back(p);

        p.header.seq = 3;
        p.header.frame_id = "odom";
        p.pose.position.x = 2.9439167976379395;
        p.pose.position.y = -2.0378057956695557;
        p.pose.position.z = 0.0006704330444335938;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
        plan_.push_back(p);

        p.header.seq = 4;
        p.header.frame_id = "odom";
        p.pose.position.x = 0.9618299007415771;
        p.pose.position.y = -0.03381189703941345;
        p.pose.position.z = 0.0017881393432617188;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
        plan_.push_back(p);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if(msg->buttons[1] == 1){
            ROS_INFO("Invia primo goal.");
            current_goal_idx_ = 0;
            sendNextGoal();
        }
    }

    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        current_goal_idx_++;
        if (current_goal_idx_ < plan_.size())
        {
            ROS_INFO("Goal raggiunto. Invia goal successivo.");
            sendNextGoal();
        }
        else
        {
            ROS_INFO("Tutti i goal inviati. Fine.");
        }
    }

    void sendNextGoal()
    {
        if (current_goal_idx_ < plan_.size())
        {
            geometry_msgs::PoseStamped goal = plan_[current_goal_idx_];
            goal.header.stamp = ros::Time::now(); // Aggiorna timestamp
            pub_goal_.publish(goal);
            ROS_INFO("Inviato goal %lu: frame_id=%s x=%.2f y=%.2f",
                current_goal_idx_+1,
                goal.header.frame_id.c_str(),
                goal.pose.position.x,
                goal.pose.position.y);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_follower");
    ros::NodeHandle nh;
    PlanFollower pf(nh);
    ros::spin();
    return 0;
}
