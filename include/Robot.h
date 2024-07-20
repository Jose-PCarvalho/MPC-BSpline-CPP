#include "Pid.h"
#include "MPC.h"
#include "Location.h"
#include "Manager.h"
#include "Graph.h"
#include <string>
#include "udpClient.h"
#include <unordered_map>

class Robot{
    
public:

    Robot(){};  
    Robot(const std::unordered_map<std::string, double>& params, std::string udp_ip); 
    Robot(const std::unordered_map<std::string, double>& params); 

    Location closest_id(); 

    Manager man = Manager();
    Graph<Location> graph;
    float normalizeAngle(float angle);

    udp::UdpClient *comms;

    //Controllers
    PID pid_x;
    PID pid_y;
    PID pid_theta;
    MPC mpc;
    void follow(Line l,bool final);
    void rotate();
    void transformToRobRef();
    void updatePosition(double x, double y, double theta);
    void state_machine();
    void path_supervisor();
    void decision_making();
    void calculate_outputs();
    bool init_comms();
    std::string receive_box_msg();
    std::string init();
    std::string init_nocomms();
        
    
    

    //path planning
    void create_path(int loc_id);
    vector<Line> line_list;
    vector<Line> trimmed_list;
    vector<Location> path;

    //gains
    double k_speed=0.4;
    double k_corr=1;
    double k_w=2;
    double speed_divisor = 2;
    
    //State Variables
    Location position;
    double theta;
    int line_state=0;
    int current_box=0;
    bool connected_server=false;
    bool magnet =false;
    bool switch_on=false;
    bool following= true;
    bool fetching= true;
    bool can_rotate=true;


    //Outputs
    double v_x;
    double v_y;
    double V;
    double Vn;
    double omega;

    //References
    double theta_ref=0;


    double dist_thresh;
    Location last_loc;
    double rotate_dist;
    double slow_zone;

   
};