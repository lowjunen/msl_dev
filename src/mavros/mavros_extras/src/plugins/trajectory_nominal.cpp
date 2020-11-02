#include <mavros/mavros_plugin.h>

#include <mavros_msgs/TrajectoryNominal.h>

namespace mavros {
namespace extra_plugins{
    
class TrajectoryNominalPlugin : public plugin::PluginBase {
public:
    TrajectoryNominalPlugin() : PluginBase(),
        nh("~")
    { }
    
    void initialize(UAS &uas_) override
    {
        PluginBase::initialize(uas_);
        trajectory_nominal_sub = nh.subscribe("trajectory_nominal_sub",10,&TrajectoryNominalPlugin::trajectory_nominal_cb,this);
    }

    Subscriptions get_subscriptions()
    {
         return {/* RX disabled */ };
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber trajectory_nominal_sub;

    void trajectory_nominal_cb(const mavros_msgs::TrajectoryNominal::ConstPtr &req) {
        mavlink::common::msg::TRAJECTORY_NOMINAL traj{};

        traj.timestamp = req->timestamp;
        std::copy(req->f_out.begin(), req->f_out.end(), traj.f_out.begin());

        UAS_FCU(m_uas)->send_message_ignore_drop(traj);
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TrajectoryNominalPlugin, mavros::plugin::PluginBase)
