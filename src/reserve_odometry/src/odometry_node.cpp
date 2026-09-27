#include "reserve_odometry/pipeline.hpp"
#include "reserve_odometry/output_frame.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>
#include <tram_vehicle_msgs/msg/driver_controller_command.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <iostream>
#include <memory>
using namespace reserve_odometry;
using Velocity=tram_vehicle_msgs::msg::VelocitySensor;
using Command=tram_vehicle_msgs::msg::DriverControllerCommand;
using Fix=sensor_msgs::msg::NavSatFix;
class OdometryNode final:public rclcpp::Node {
 std::unique_ptr<Pipeline>engine_;
 rclcpp::Subscription<Velocity>::SharedPtr front_,rear_;
 rclcpp::Subscription<Command>::SharedPtr cmd_;
 rclcpp::Subscription<Fix>::SharedPtr master_,rover_;
 rclcpp::Publisher<Velocity>::SharedPtr vel_;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_;
 rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr diagnostic_;
 rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr state_status_;
 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::string frame_,child_;std::int64_t last_cmd_arrival_=0,last_any_arrival_=0,last_header_=0,last_clock_=0,first_input_arrival_=0;
 bool have_input_=false;std::size_t published_=0;double x0_=0.,y0_=0.,z0_=0.,yaw0_=0.;
 bool map_output_=false,body_velocity_output_=false,publish_relative_=true;OutputFrame output_frame_;
 static std::int64_t ns(const builtin_interfaces::msg::Time&s){return std::int64_t(s.sec)*1000000000LL+s.nanosec;}
 std::int64_t arrival(){auto t=get_clock()->now().nanoseconds();
   if(last_clock_>0&&t<last_clock_){engine_->reset();last_cmd_arrival_=last_any_arrival_=last_header_=first_input_arrival_=0;have_input_=false;RCLCPP_WARN(get_logger(),"ROS clock moved backward: estimator reset");}
   last_clock_=t;return t;
 }
 void publish(std::int64_t stamp,std::int64_t record_time){
   auto begin=std::chrono::steady_clock::now();Output out;if(!engine_->step(stamp,record_time,out))return;
   auto p=out.position;auto body=out.body;
   if(!out.initialized){p={{x0_+out.distance*std::cos(yaw0_),y0_+out.distance*std::sin(yaw0_),z0_}};body=body_pose(p,{{std::cos(yaw0_),std::sin(yaw0_),0}});}
   if(out.initialized&&map_output_){p=output_frame_.position(p);body=output_frame_.pose(body);}
   Velocity velocity;velocity.header.stamp=static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(stamp,RCL_ROS_TIME));velocity.header.frame_id=child_;velocity.velocity=out.velocity;
   if(out.initialized&&body_velocity_output_)velocity.velocity=out.body_velocity[0];
   nav_msgs::msg::Odometry odom;odom.header=velocity.header;odom.header.frame_id=out.initialized?frame_:"odom_relative";odom.child_frame_id=child_;
   odom.pose.pose.position.x=p[0];odom.pose.pose.position.y=p[1];odom.pose.pose.position.z=p[2];
   double yaw=std::atan2(body.forward[1],body.forward[0]),pitch=-std::atan2(body.forward[2],std::hypot(body.forward[0],body.forward[1]));
   odom.pose.pose.orientation.x=-std::sin(yaw*.5)*std::sin(pitch*.5);odom.pose.pose.orientation.y=std::cos(yaw*.5)*std::sin(pitch*.5);odom.pose.pose.orientation.z=std::sin(yaw*.5)*std::cos(pitch*.5);odom.pose.pose.orientation.w=std::cos(yaw*.5)*std::cos(pitch*.5);
   odom.twist.twist.linear.x=out.body_velocity[0];odom.twist.twist.linear.y=out.body_velocity[1];odom.twist.twist.linear.z=out.body_velocity[2];odom.twist.twist.angular.x=out.body_angular_velocity[0];odom.twist.twist.angular.y=out.body_angular_velocity[1];odom.twist.twist.angular.z=out.body_angular_velocity[2];
   // Heuristic conservative variances, not calibrated confidence intervals.
   double sigma_s=1.+.003*out.distance+out.init_rms+(out.initialized?0.:10.)+((out.flags&3)?3.:0.)+((out.flags&32)?5.:0.);
   if(out.flags&(256|512))sigma_s=std::max(sigma_s,10.);
   if(out.flags&1024)sigma_s=std::max(sigma_s,25.);
   for(int i:{0,7,14})odom.pose.covariance[i]=sigma_s*sigma_s;
   for(int i:{21,28,35})odom.pose.covariance[i]=out.initialized?.03:1e6;
   odom.twist.covariance[0]=(out.flags&3)?2.25:.0025;for(int i:{7,14,21,28,35})odom.twist.covariance[i]=1e6;
   vel_->publish(velocity);if(out.initialized||publish_relative_)pos_->publish(odom);std_msgs::msg::UInt8 diagnostic;diagnostic.data=std::uint8_t(out.flags);diagnostic_->publish(diagnostic);
   std_msgs::msg::UInt16 state;state.data=std::uint16_t(out.flags);state_status_->publish(state);
   if(++published_%20==0){double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();std_msgs::msg::Float64MultiArray tm;tm.data={us,double(out.route),out.route_s,out.distance,out.init_rms,double(out.gnss_accepted),double(out.gnss_rejected),out.gnss_last_innovation,double(out.flags),out.slip?1.:0.,out.slip_ratio,double(out.slip_events)};timing_->publish(tm);}
 }
 public:
 OdometryNode():Node("reserve_odometry"){
   Settings c;c.max_speed=declare_parameter<double>("max_speed",40.);c.wheel_divisor=declare_parameter<double>("wheel_divisor",3.6);c.init_seconds=declare_parameter<double>("initial_gnss_seconds",3.);c.command_timeout=declare_parameter<double>("command_timeout",.5);
   c.master_x=declare_parameter<double>("master_x",-9.873);c.rover_x=declare_parameter<double>("rover_x",2.563);c.antenna_height=declare_parameter<double>("antenna_height",3.);
   c.output_x=declare_parameter<double>("output_point_x",0.);c.output_z=declare_parameter<double>("output_point_z",0.);
   c.velocity_x=declare_parameter<double>("velocity_point_x",-9.873);c.velocity_z=declare_parameter<double>("velocity_point_z",3.);
   c.compensate_velocity_lever_arm=declare_parameter<bool>("compensate_velocity_lever_arm",false);
   c.align_initial_position=declare_parameter<bool>("align_initial_position",false);c.speed_scale=declare_parameter<double>("speed_scale",1.);
   c.allow_degraded_initialization=declare_parameter<bool>("allow_degraded_initialization",true);
   c.sparse_gnss_correction=declare_parameter<bool>("sparse_gnss_correction",false);
   c.gnss_xy_residual_correction=declare_parameter<bool>("gnss_xy_residual_correction",false);
   c.gnss_gain=declare_parameter<double>("gnss_correction_gain",.25);c.gnss_max_step=declare_parameter<double>("gnss_max_step",5.);
   c.gnss_innovation_gate=declare_parameter<double>("gnss_innovation_gate",30.);c.gnss_residual_gate=declare_parameter<double>("gnss_residual_gate",8.);c.gnss_max_age=declare_parameter<double>("gnss_max_age",1.);
   c.adhesion_accel_limit=declare_parameter<double>("adhesion_accel_limit",0.);c.adhesion_decel_limit=declare_parameter<double>("adhesion_decel_limit",3.2);c.slip_release_rate=declare_parameter<double>("slip_release_rate",.2);c.slip_release_hold=declare_parameter<double>("slip_release_hold",3.);c.slip_max_latch=declare_parameter<double>("slip_max_latch",10.);
   c.offmap_departure=declare_parameter<double>("offmap_departure",0.);
   c.offmap_angle=declare_parameter<double>("offmap_angle",10.);c.offmap_min_span=declare_parameter<double>("offmap_min_span",5.);
   c.offmap_horizon=declare_parameter<double>("offmap_horizon",3.);c.offmap_max_distance=declare_parameter<double>("offmap_max_distance",100.);
   map_output_=declare_parameter<bool>("map_output",false);body_velocity_output_=declare_parameter<bool>("body_velocity_output",false);publish_relative_=declare_parameter<bool>("publish_relative_position",true);
   output_frame_.yaw=declare_parameter<double>("map_rotation_yaw",0.);auto translation=declare_parameter<std::vector<double>>("map_translation",std::vector<double>{0.,0.,0.});
   if(translation.size()!=3||!std::isfinite(output_frame_.yaw))throw std::runtime_error("Invalid output map transform");
   std::copy(translation.begin(),translation.end(),output_frame_.translation.begin());if(!finite(output_frame_.translation))throw std::runtime_error("Invalid output map translation");
   c.origin={{declare_parameter<double>("origin_lat",55.805),declare_parameter<double>("origin_lon",37.425),declare_parameter<double>("origin_alt",170.)}};
   frame_=declare_parameter<std::string>("frame_id","map_enu");child_=declare_parameter<std::string>("child_frame_id","base_link");
   x0_=declare_parameter<double>("initial_x",0.);y0_=declare_parameter<double>("initial_y",0.);z0_=declare_parameter<double>("initial_z",0.);yaw0_=declare_parameter<double>("initial_yaw",0.);
   auto paths=declare_parameter<std::vector<std::string>>("route_files",std::vector<std::string>{});bool closed=declare_parameter<bool>("closed_route",false);double length=declare_parameter<double>("bogie_separation",7.55);
   std::vector<Route>routes;for(const auto&p:paths)routes.push_back(Route::load(p,length,closed));engine_=std::make_unique<Pipeline>(c,std::move(routes));
   if(paths.empty())RCLCPP_WARN(get_logger(),"No route: relative along-track output, frame odom_relative");
   vel_=create_publisher<Velocity>("/result/velocity",10);pos_=create_publisher<nav_msgs::msg::Odometry>("/result/position",10);diagnostic_=create_publisher<std_msgs::msg::UInt8>("/result/slip_status",10);timing_=create_publisher<std_msgs::msg::Float64MultiArray>("/result/diagnostics",10);
   state_status_=create_publisher<std_msgs::msg::UInt16>("/result/state_status",10);
   auto qos=rclcpp::SensorDataQoS().keep_last(100);
   auto wheel=[this](int channel,Velocity::ConstSharedPtr m){auto a=arrival();auto h=ns(m->header.stamp);if(engine_->wheel(channel,h,m->velocity,a)){last_header_=std::max(last_header_,h);last_any_arrival_=a;if(!have_input_)first_input_arrival_=a;have_input_=true;}};
   front_=create_subscription<Velocity>("/vehicle/front_bogie_velocity",qos,[wheel](Velocity::ConstSharedPtr m){wheel(0,m);});rear_=create_subscription<Velocity>("/vehicle/rear_bogie_velocity",qos,[wheel](Velocity::ConstSharedPtr m){wheel(1,m);});
   cmd_=create_subscription<Command>("/vehicle/driver_position_cmd",qos,[this](Command::ConstSharedPtr m){auto a=arrival();auto h=ns(m->header.stamp);if(!engine_->command(h,m->position,a))return;last_cmd_arrival_=last_any_arrival_=a;last_header_=std::max(last_header_,h);have_input_=true;publish(h,a);});
   master_=create_subscription<Fix>("/sensing/gnss/master/fix",qos,[this](Fix::ConstSharedPtr m){auto a=arrival();engine_->fix(0,ns(m->header.stamp),a,{{m->latitude,m->longitude,m->altitude}},m->status.status);});
   rover_=create_subscription<Fix>("/sensing/gnss/rover/fix",qos,[this](Fix::ConstSharedPtr m){auto a=arrival();engine_->fix(1,ns(m->header.stamp),a,{{m->latitude,m->longitude,m->altitude}},m->status.status);});
   timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(.05),[this](){auto now=arrival();if(!have_input_||now<=0)return;
     // A wheel can arrive before the first /clock tick. Anchor the startup
     // grace to valid ROS time so the timer cannot jump ahead of queued commands.
     if(first_input_arrival_<=0){first_input_arrival_=now;return;}
     if(now-first_input_arrival_<100000000LL)return;
     if(last_cmd_arrival_>0&&now-last_cmd_arrival_<100000000LL)return;
     // Never integrate wall-clock decades when --clock was omitted on bag replay.
     if(std::llabs(now-last_header_)>30000000000LL){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Input and clock differ >30s; use bag --clock and use_sim_time:=true");return;}
     publish(now,now);
   });
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<OdometryNode>());}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
