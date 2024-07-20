#include <stdlib.h>
#include <ros/ros.h>
#include "Location.h"
#include "Graph.h"
#include "Robot.h"
#include "path_planning.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "sdpo_ratf_ros_path_planning/boxes_info.h"
#include <unordered_map>
#include <XmlRpcException.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h> // Include standard message type


std::vector<double> x_trajectory;
std::vector<double> y_trajectory;
double x,y,vx,vy,vx_ref,vy_ref,yaw;

double k_speed;
double k_correction;
double k_w, kd_w, kd_v;
double dist_tresh;
double box_dist;
double rotate_dist;
double slow_zone;
double speed_divisor;

bool mach_b;
uint64_t k;
std::string udp_srv_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
}

void CbSwitchState(const std_msgs::Bool::ConstPtr& msg){
    rob.switch_on = msg->data;
}

void getParameters(ros::NodeHandle nh){
    nh.getParam("k_speed", k_speed);
    nh.getParam("k_correction", k_correction);
    nh.getParam("k_w", k_w);
    nh.getParam("dist_tresh", dist_tresh);
    nh.getParam("box_dist", box_dist);
    nh.getParam("rotate_dist", rotate_dist);
    nh.getParam("kd_v", kd_v);
    nh.getParam("kd_w", kd_w);
    nh.getParam("slow_zone", slow_zone);
    nh.getParam("speed_divisor", speed_divisor);
    nh.getParam("mach_b", mach_b);
    nh.getParam("x_trajectory",x_trajectory);
    nh.getParam("y_trajectory",y_trajectory);
    //nh.getParam("box_order", srv_msg);
}



float normalizeAngle(float angle){
    float new_angle;
    int num;
    
    if(angle > M_PI){
        num = angle/M_PI;
        new_angle = angle-(num*2*M_PI);
    }
    else if (angle < -M_PI){
        num = angle/M_PI;
        new_angle = angle - (num*2*M_PI);
    }
    else{
        new_angle = angle;
    }

    return new_angle;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdpo_path_planning");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    tf::TransformListener tf_listen_;
    ROS_INFO("Launching Path Planning");
    ros::Publisher vel_pub, boxes_pub;
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    boxes_pub = nh.advertise<sdpo_ratf_ros_path_planning::boxes_info>("boxes_position",1);
    ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("x", 10);
    ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("y", 10);
    ros::Publisher rx0_pub = nh.advertise<std_msgs::Float64>("rx0", 10);
    ros::Publisher ry0_pub = nh.advertise<std_msgs::Float64>("ry0", 10);
    //ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);
    getParameters(nh_priv);
    
    //Coordinate Frame IDs
    nh_priv.param<string>("map_frame_id", map_frame_id_,
                                    "map");
    // ROS_INFO("[sdpo_ratf_ros_localization] Map frame ID: %s",
    //         map_frame_id_.c_str());

    nh_priv.param<string>("base_frame_id", base_frame_id_,
                                    "base_footprint");
    // ROS_INFO("[sdpo_ratf_ros_localization] Base footprint frame ID: %s",
    //         base_frame_id_.c_str());

    geometry_msgs::Twist vel_msg;
    ros::Rate rate(25);
    
    tf_listen_.waitForTransform(map_frame_id_, base_frame_id_, ros::Time::now(),ros::Duration(5.0));
    XmlRpc::XmlRpcValue nodes_list;
    XmlRpc::XmlRpcValue edges_list;
    XmlRpc::XmlRpcValue trajectories;
    std::string udp_server_ip;
    int udp_server_port;
    try {
        nh_priv.getParam("edges",edges_list);
        nh_priv.getParam("nodes",nodes_list);
        nh_priv.getParam("ip_address", udp_server_ip);
        nh_priv.getParam("port", udp_server_port);
        nh_priv.getParam("trajectories", trajectories);
    } catch (XmlRpc::XmlRpcException& e) {
        ROS_INFO("[sdpo_ratf_ros_path_planning] Exception: %s",
                e.getMessage().c_str());
    }
    
    std::unordered_map<std::string, double> params = {
        {"k_speed", k_speed},
        {"k_corr", k_correction},
        {"k_w", k_w},
        {"udp_port", udp_server_port},
        {"kd_v", kd_v},
        {"kd_w", kd_w},
        {"dist_thresh", dist_tresh},
        {"speed_divisor", speed_divisor},
        {"rotate_dist", rotate_dist},
        {"slow_zone", slow_zone}
    };

    rob = Robot(params);
    int N=rob.mpc.N;
    std::vector<double> rx(N), ry(N);
    x = x_trajectory[0];
    y = y_trajectory[0];
    vx_ref = 0;
    vy_ref = 0;
    std::cout << x << std::endl;
    int j = 0;
    while(ros::ok())
    {   
        tf::StampedTransform tf_base2map;
        try {
            tf_listen_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                tf_base2map);
        } catch (tf::TransformException& e) {
            throw std::runtime_error(e.what());
        }
        tf::Quaternion quat;
        tf::Vector3 pos;
        double yaw;
        pos = tf_base2map.getOrigin();
        quat = tf_base2map.getRotation();
        yaw = normalizeAngle(tf::getYaw(quat));
        // rob.updatePosition(pos[0], pos[1], yaw);
        x = pos[0];
        y = pos[1];
        //std::cout << " x: " << x << " y " << y << std::endl;
        rob.theta = yaw;
        rob.theta_ref = 0;
        rob.rotate();

        ++k;
        int k_mod = k % x_trajectory.size();
        int offset=0;
        
        for (int i = 0; i < N; ++i) 
        {
            rx[i] = x_trajectory[(k_mod + i + offset) % x_trajectory.size()];
            ry[i] = y_trajectory[(k_mod + i + offset) % y_trajectory.size()];
        }
        
        k =k+ offset;
        Eigen::VectorXd x_bar;
        if (rob.mpc.augmented)
        {   
            x_bar.resize(6); 
            x_bar << x, y,vx, vy, vx_ref, vy_ref;
        }
        else
        {   
            x_bar.resize(4); 
            x_bar << x, y, vx_ref, vy_ref;
        }
        // Eigen::VectorXd x_bar(4);
        // x_bar << x, y, vx_ref, vy_ref;
        
        //x_bar << 0, 0, 0, 0, 0, 0;

        // Construct r
        Eigen::MatrixXd r(2 * N, 1);
        for (int i = 0; i < N; ++i) {
            r(2 * i) = rx[i];
            r(2 * i + 1) = ry[i];
            //std::cout << " x: " << r(2*i) << " y " << r(2*i+1) << std::endl; 
        }
        
        rob.mpc.compute(x_bar,r);
        Eigen::VectorXd delta_u = rob.mpc.compute(x_bar, r);

        //std::cout << "Delta_u: " << delta_u.transpose() << std::endl;
        vx_ref+=delta_u(0);
        vy_ref+=delta_u(1);

        
        vel_msg.linear.x= vx_ref * cos(yaw) + vy_ref * sin(yaw);
        vel_msg.linear.y= -vx_ref * sin(yaw) + vy_ref * cos(yaw);
        vel_msg.angular.z= rob.omega;
        vel_pub.publish(vel_msg);
        std_msgs::Float64 x_msg;
        x_msg.data = x;
        x_pub.publish(x_msg);

        std_msgs::Float64 y_msg;
        y_msg.data = y;
        y_pub.publish(y_msg);

        std_msgs::Float64 rx0_msg;
        rx0_msg.data = rx[0];
        rx0_pub.publish(rx0_msg);

        std_msgs::Float64 ry0_msg;
        ry0_msg.data = ry[0];
        ry0_pub.publish(ry0_msg);

        rate.sleep();
        ros::spinOnce();
        
    }
}



Graph<Location> create_graph(XmlRpc::XmlRpcValue nodes_list,XmlRpc::XmlRpcValue edges_list){
    Graph<Location> g;
    for (int32_t i = 0; i < nodes_list.size(); ++i) 
        {
        int id = int(nodes_list[i]["id"]);
        double x = double(nodes_list[i]["x"]);
        double y = double(nodes_list[i]["y"]);
        Location l=Location(id,double(x)*0.001,double(y)*0.001);
        bool success=g.addVertex(l);
        if (success)
            std::cout << "id " << id << " x : " << double(x)*0.001 << " y : " << double(y)*0.001 << std::endl;
        else
            std::cout << "failed at id " << id<< std::endl;
        }
        

    for (int32_t i = 0; i < edges_list.size(); ++i) 
        {
        int source = int(edges_list[i]["source"]);
        int target = int(edges_list[i]["target"]);
        bool success1=g.addEdge(g.findVertexById(source)->getInfo(),g.findVertexById(target)->getInfo());
        bool success2=g.addEdge(g.findVertexById(target)->getInfo(),g.findVertexById(source)->getInfo()); //bidirectional graph
        if (success1 && success2)
            std::cout << "Edge from " << source << " to " << target <<  std::endl;
        else
            std::cout << "FAILED AT "  << "Edge from " << source << " to " << target <<  std::endl;

         }
    return g;

}



void use_magnet(bool logic_level)
{
    std_srvs::SetBool srv_msg;
    srv_msg.request.data = logic_level;
    if(magnet_client.call(srv_msg)){
        //ROS_INFO("Magnet not active");
    }
    else{
        //ROS_ERROR("Magnet desactivation failed");
    }
}






