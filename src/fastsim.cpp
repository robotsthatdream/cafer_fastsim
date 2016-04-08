#define USE_SDL
#include <sstream>
#include <unistd.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "tf/transform_broadcaster.h"

#include "fastsim.hpp"

// custom services msgs
#include "fastsim/Teleport.h"
#include "fastsim/UpdateDisplay.h"
#include "fastsim/CIS.h"
#include "fastsim/RobotColor.h"

#include "fastsim/ISW.h"

#include <tbb/tbb.h>
#include <ros/callback_queue.h>
#include <cafer_core/component.hpp>

using namespace fastsim;

namespace fastsim {

  CAFER_CLIENT(FastsimToCafer){
    using AbstractClient::AbstractClient; // C++11 requirement to inherit the constructor
    
  // ROS handles
  boost::shared_ptr<ros::Subscriber> speed_left_s;
  boost::shared_ptr<ros::Subscriber> speed_right_s;

  boost::shared_ptr<ros::Publisher> left_bumper_p;
  boost::shared_ptr<ros::Publisher> right_bumper_p;
  boost::shared_ptr<ros::Publisher> collision_p;
  boost::shared_ptr<ros::Publisher> radars_p;
  boost::shared_ptr<ros::Publisher> lasers_p;
  boost::shared_ptr<ros::Publisher> laser_scan_p;
  boost::shared_ptr<ros::Publisher> isw_sensor_p;
  boost::shared_ptr<ros::Publisher> isw_p;
  boost::shared_ptr<ros::Publisher> odom_p;
  boost::shared_ptr<ros::Publisher> clock_p;

  boost::shared_ptr<ros::ServiceServer> service_teleport;
  boost::shared_ptr<ros::ServiceServer> service_display;
  boost::shared_ptr<ros::ServiceServer> service_cis;
  boost::shared_ptr<ros::ServiceServer> service_robot_color;

  // Sensor data (updated by the callbacks)
  tbb::concurrent_bounded_queue<std_msgs::Float32> speed_left;
  tbb::concurrent_bounded_queue<std_msgs::Float32> speed_right;

  float speed_left_current;
  float speed_right_current;

  boost::shared_ptr<Robot> robot;
  boost::shared_ptr<Map> map;

  ros::Time sim_time;
  float sim_dt;
  bool initialized=false;

public:

  ~FastsimToCafer(void) {disconnect_from_ros();}

  void init(void) {

    // load ROS config
    sim_time = ros::Time::now();

    std::string settings_name;
    cafer_core::ros_nh->param("settings", settings_name, std::string("envs/example.xml"));
    std::string path;
    cafer_core::ros_nh->param("path", path, std::string("."));
    bool sync = false;
    cafer_core::ros_nh->param("sync", sync, false);

    double freq;
    cafer_core::ros_nh->param("frequency", freq, 30.0);
    sim_dt=1./freq;
    ROS_WARN_STREAM("changing path to "<<path);
    ROS_WARN_STREAM("settings is "<<settings_name);
    ROS_WARN_STREAM("sync is "<<sync);
    chdir(path.c_str());

    // fastsim config
    Settings settings(settings_name);
    robot = settings.robot();
    map = settings.map();
    display = settings.display();


    connect_to_ros();
    initialized=true;
  }

  bool is_initialized() {
    return initialized;
  }

  void connect_to_ros(void) {
    left_bumper_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Bool>("left_bumper",10)));
    right_bumper_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Bool>("right_bumper",10)));
    collision_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Bool>("collision",10)));
    radars_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Int16MultiArray>("radars",10)));
    lasers_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Float32MultiArray>("lasers",1)));
    laser_scan_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::LaserScan>("laser_scan",1)));
    isw_sensor_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Int8MultiArray>("light_sensors",10)));
    isw_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<fastsim::ISW>("isw",10)));
    odom_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<nav_msgs::Odometry>("odom",10)));
    clock_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<rosgraph_msgs::Clock>("clock",10)));

    speed_left_s.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("speed_left",10,&FastsimToCafer::speed_left_cb,this)));
    speed_right_s.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("speed_right",10,&FastsimToCafer::speed_right_cb,this)));

    service_teleport.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService("teleport",&FastsimToCafer::teleport_cb,this)));
    service_display.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService("display",&FastsimToCafer::display_cb,this)));
    service_cis.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService("cis",&FastsimToCafer::cis_cb,this)));
    service_robot_color.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService("robot_color",&FastsimToCafer::robot_color_cb,this)));

  }

  void disconnect_from_ros(void) {
    speed_left_s.reset();
    speed_right_s.reset();

    left_bumper_p.reset();
    right_bumper_p.reset();
    collision_p.reset();
    radars_p.reset();
    lasers_p.reset();
    laser_scan_p.reset();
    isw_sensor_p.reset();
    isw_p.reset();
    odom_p.reset();
    clock_p.reset();

    service_teleport.reset();
    service_display.reset();
    service_cis.reset();
    service_robot_color.reset();
  }

  void update(void) {
    if (teleport)
      {
        ROS_INFO("Teleporting to (%f,%f,%f)", teleport_x, teleport_y, teleport_theta);
	teleport = false;
        robot->set_pos(Posture(teleport_x, teleport_y, teleport_theta));
        robot->move(0, 0, map);
        //k = 0;
      }

    if (cis){
      while(cis_number.size()!=0){
        if(cis_state[0]==-1){
          map->get_illuminated_switches()[cis_number[0]]->set_on(!map->get_illuminated_switches()[cis_number[0]]->get_on());
        }
        else if (cis_state[0]==0){
          map->get_illuminated_switches()[cis_number[0]]->set_on(false);
        }
        else if (cis_state[0]==1){
          map->get_illuminated_switches()[cis_number[0]]->set_on(true);
        }
        cis_number.erase(cis_number.begin());
        cis_state.erase(cis_state.begin());
      }
      cis=false;
    }

    if(change_robot_color) {
      robot->set_color(robot_color);
      change_robot_color=false;

    }

    if (!sync || (new_speed_left && new_speed_right))
      {
        //++k;
        robot->move(sp_left, sp_right, map);
        new_speed_left = false;
        new_speed_right = false;
      }

    // bumpers
    std_msgs::Bool msg_left_bumper, msg_right_bumper;
    msg_left_bumper.data = robot->get_left_bumper();
    msg_right_bumper.data = robot->get_right_bumper();
    left_bumper_p->publish(msg_left_bumper);
    right_bumper_p->publish(msg_right_bumper);

    publish_collision(robot);

    // other publishers
    publish_lasers(robot);
    publish_laser_scan(tf, robot, sim_time);
    publish_radars(robot);
    publish_odometry(tf, robot, sim_time, sim_dt);
    publish_isw_sensor(robot);
    publish_isw(map);

    // and the clock
    rosgraph_msgs::Clock msg_clock;
    msg_clock.clock = sim_time;
    clock_p->publish(msg_clock);

    if (display)
      {
        if (!d)
          d = boost::make_shared<Display>(map, *robot);
        d->update();
      }

    sim_time = ros::Time::now();

 }

  // speed left & right
  float sp_left = 0;
  float sp_right = 0;
  bool display = true;
  bool teleport = false;
  float teleport_x = -1;
  float teleport_y = -1;
  float teleport_theta = 0;
  bool new_speed_left = false;
  bool new_speed_right = false;
  int robot_color=0;
  bool change_robot_color=false;

  // change illuminated switch service
  bool cis = false;
  std::vector<int> cis_number;
  std::vector<int> cis_state; // 0:false, 1: true, -1: change state

  boost::shared_ptr<Display> d;

  tf::TransformBroadcaster tf;

  // the callbacks
  void speed_left_cb(const std_msgs::Float32::ConstPtr& msg) {
    new_speed_left = true;
    sp_left = msg->data;
    //ROS_INFO("Getting new speed_left: %f",sp_left);
  }
  void speed_right_cb(const std_msgs::Float32::ConstPtr& msg) {
    new_speed_right = true;
    sp_right = msg->data;
  }
  bool display_cb(fastsim::UpdateDisplay::Request &req,
		  fastsim::UpdateDisplay::Response &res) {
    display = req.state;
    res.ack = true;
    return true;
  }
  bool robot_color_cb(fastsim::RobotColor::Request &req,
		      fastsim::RobotColor::Response &res) {
    robot_color = req.color;
    change_robot_color=true;
    res.ack = true;
    return true;
  }
  bool teleport_cb(fastsim::Teleport::Request &req,
		   fastsim::Teleport::Response &res) {
    teleport = true;
    teleport_x = req.x;
    teleport_y = req.y;
    teleport_theta = req.theta;
    res.ack = true;
    return true;
  }
  bool cis_cb(fastsim::CIS::Request &req, fastsim::CIS::Response &res) {
    cis=true;
    cis_number.push_back(req.number);
    cis_state.push_back(req.state);
    res.ack=true;
    return true;
  }


  void publish_collision(const boost::shared_ptr<Robot>& robot) {
    std_msgs::Bool b;
    b.data=robot->get_collision();
    collision_p->publish(b);
  }

  void publish_radars(const boost::shared_ptr<Robot>& robot) {
    if (!robot->get_radars().empty()) {
      std_msgs::Int16MultiArray msg_radar;
      for (size_t i = 0; i < robot->get_radars().size(); ++i)
	msg_radar.data.push_back(robot->get_radars()[i].get_activated_slice());
      radars_p->publish(msg_radar);
    }
  }

  void publish_lasers(const boost::shared_ptr<Robot>& robot) {
    if (!robot->get_lasers().empty()) {
      // basic lasers
      std_msgs::Float32MultiArray laser_msg;
      for (size_t i = 0; i < robot->get_lasers().size(); ++i)
	laser_msg.data.push_back(robot->get_lasers()[i].get_dist());
      lasers_p->publish(laser_msg);
    }
  }


  void publish_laser_scan(tf::TransformBroadcaster& tf,
			  const boost::shared_ptr<Robot>& robot,
			  const ros::Time& sim_time) {
    if (!robot->get_laser_scanners().empty()) {
      // ROS laser scan
      sensor_msgs::LaserScan scan_msg;
      const LaserScanner& scanner = robot->get_laser_scanners()[0];
      scan_msg.angle_min = scanner.get_angle_min();
      scan_msg.angle_max = scanner.get_angle_max();
      scan_msg.angle_increment = scanner.get_angle_increment();
      scan_msg.range_min = 0.0;
      scan_msg.range_max = scanner.get_range_max();
      for(size_t i = 0; i < scanner.get_lasers().size(); i++)  {
	// we ignore the value if there is nothing
	// I don't know if is right...
	// see: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
	if (scanner.get_lasers()[i].get_dist() > 0)
	  scan_msg.ranges.push_back(scanner.get_lasers()[i].get_dist());
	else
	  scan_msg.ranges.push_back(0);
      }
      scan_msg.header.frame_id = "base_laser_link";;
      scan_msg.header.stamp = sim_time;
      laser_scan_p->publish(scan_msg);

      // now the tf
      tf::Quaternion laserQ;
      laserQ.setRPY(0.0, 0.0, robot->get_pos().theta());
      tf::Transform txLaser =
	tf::Transform(laserQ, tf::Point(0,//robot->get_pos().x(),
					0,//robot->get_pos().y(),
					0.15));
      tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
					    "base_link", "base_laser_link"));
      tf::Transform txIdentity(tf::createIdentityQuaternion(),
			       tf::Point(0, 0, 0));
      tf.sendTransform(tf::StampedTransform(txIdentity, sim_time,
					    "base_footprint", "base_link"));
    }
  }

  void publish_isw_sensor(const boost::shared_ptr<Robot>& robot) {
    if (!robot->get_light_sensors().empty()) {
      // basic lasers
      std_msgs::Int8MultiArray isw_msg;
      for (size_t i = 0; i < robot->get_light_sensors().size(); ++i) {
	isw_msg.data.push_back(robot->get_light_sensors()[i].get_activated());
      }
      isw_sensor_p->publish(isw_msg);
    }
  }

  void publish_isw(const boost::shared_ptr<Map>& map) {
    fastsim::ISW isw_msg;
    typedef boost::unordered_map<int, int> mapindex;
    mapindex index;
    for (size_t i = 0; i <map->get_illuminated_switches().size(); ++i) {
      if (index.find(map->get_illuminated_switches()[i]->get_color())==index.end())
	index[map->get_illuminated_switches()[i]->get_color()]=0;
      else {
	index[map->get_illuminated_switches()[i]->get_color()]++;
      }
      isw_msg.index=index[map->get_illuminated_switches()[i]->get_color()];
      isw_msg.x=map->get_illuminated_switches()[i]->get_x();
      isw_msg.y=map->get_illuminated_switches()[i]->get_y();
      isw_msg.color=map->get_illuminated_switches()[i]->get_color();
      isw_msg.on=map->get_illuminated_switches()[i]->get_on();
      isw_p->publish(isw_msg);
    }
  }



  void publish_odometry(tf::TransformBroadcaster& tf,
			const boost::shared_ptr<Robot>& robot,
			const ros::Time& sim_time,
			float sim_dt) {
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = robot->get_pos().x();
    msg.pose.pose.position.y = robot->get_pos().y();
    msg.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(robot->get_pos().theta());
    msg.twist.twist.linear.x = robot->get_vx() / sim_dt;
    msg.twist.twist.linear.y = robot->get_vy() / sim_dt;
    msg.twist.twist.angular.z = robot->get_va() / sim_dt;
    msg.header.frame_id = "odom";
    msg.header.stamp = sim_time;

    odom_p->publish(msg);

    tf::Quaternion odomQ;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, odomQ);
    tf::Transform txOdom(odomQ,
			 tf::Point(msg.pose.pose.position.x,
				   msg.pose.pose.position.y, 0.0));
    tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
					  "odom", "base_footprint"));
  } 


};

}



int main(int argc, char **argv) {

  cafer_core::init(argc,argv,"fastsim");
  std::string mgmt_topic;
  cafer_core::ros_nh->param("management_topic", mgmt_topic, std::string("fastsim_mgmt"));
  cafer_core::Component<fastsim::FastsimToCafer> cafer(mgmt_topic,"fastsim");
  //cafer.get_client().init();
  //cafer.ack_creation();
  while (ros::ok()) {
    cafer.get_client().update();
    cafer.spin();
    cafer.sleep();
  }

  return 0;
}
