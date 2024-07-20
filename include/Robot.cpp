#include "Robot.h"
#include <eigen3/Eigen/Dense>


    Robot::Robot(const std::unordered_map<std::string, double>& params, std::string udp_ip){
        this->position= Location(21, -0.6950+0.05, -0.3550);
        k_speed = params.at("k_speed");
        k_corr = params.at("k_corr");
        k_w = params.at("k_w");
        double kd_v = params.at("kd_v");
        double kd_w = params.at("kd_w");
        dist_thresh = params.at("dist_thresh");
        speed_divisor = params.at("speed_divisor");
        rotate_dist = params.at("rotate_dist");
        slow_zone = params.at("slow_zone");
        this->pid_x=PID(k_corr,kd_v,0,k_speed);
        this->pid_y=PID(k_corr,kd_v,0,k_speed);
        this->pid_theta=PID(k_w,kd_w,0,5); //0.6
        comms = new::udp::UdpClient(udp_ip, params.at("udp_port"));
    }
    Robot::Robot(const std::unordered_map<std::string, double>& params){
        this->position= Location(21, -0.6950+0.05, -0.3550);
        k_speed = params.at("k_speed");
        k_corr = params.at("k_corr");
        k_w = params.at("k_w");
        double kd_v = params.at("kd_v");
        double kd_w = params.at("kd_w");
        dist_thresh = params.at("dist_thresh");
        speed_divisor = params.at("speed_divisor");
        rotate_dist = params.at("rotate_dist");
        slow_zone = params.at("slow_zone");
        this->pid_x=PID(k_corr,kd_v,0,k_speed);
        this->pid_y=PID(k_corr,kd_v,0,k_speed);
        this->pid_theta=PID(k_w,kd_w,0,1); //0.6
        this->mpc = MPC();
        
    }

    void Robot::follow(Line l,bool final){
        Location intr=l.dist2Line(this->position);
        Eigen::Vector2d r2end(l.getXf()-position.getX(),l.getYf()-position.getY());
        r2end.normalize();
        Eigen::Vector2d r = l.getVec();
        r.normalize();
        if (r.dot(r2end)<-0.9){
            r=-1*r;
        }
        Eigen::Vector2d res;
        double error_x=intr.getX()-this->position.getX();
        double error_y=intr.getY()-this->position.getY();
        Eigen::Vector2d nominal = l.getSpeed(intr,k_speed,final);
        res.x()=r.x()*nominal.x()+this->pid_x.compute(error_x);
        res.y()=r.y()*nominal.y()+this->pid_y.compute(error_y);
        //res.normalize();
        //res=res*k_speed;
        v_x=res.x();
        v_y=res.y();
        this->transformToRobRef();
    
    }

    void Robot::rotate(){
        this->omega=this->pid_theta.compute(normalizeAngle(theta_ref-theta));
    }

    void Robot::transformToRobRef(){
        this->V = (this->v_x*cos(this->theta)+this->v_y*sin(this->theta));
        this->Vn = (-this->v_x*sin(this->theta)+this->v_y*cos(this->theta));
    }

    void Robot::updatePosition(double x, double y, double theta){
        this->position.setLocation(x,y);
        this->theta=this->normalizeAngle(theta);
    }
    void  Robot::create_path(int loc_id){
        Location closest = this->closest_id();
        path=this->graph.dijkstraShortestPath(closest,this->graph.findVertexById(loc_id)->getInfo()); 
         for (auto i: path){
                std::cout << i.getID() << " ";
            }
            // ROS_INFO("PATH");
            line_list= generateLines(path);
            // for (auto i: line_list){
            //     std::cout <<"X: " << i.getXi() << ' ' << i.getXf() << "\n";
            //     std::cout <<"Y: "<< i.getYi() << ' ' << i.getYf() << "  next\n \n";}
            // ROS_INFO("Not Trimmed");
            trimmed_list= trimLines(line_list);
            // for (auto i: trimmed_list){
            //     std::cout <<"X: " << i.getXi() << ' ' << i.getXf() << "\n";
            //     std::cout <<"Y: "<< i.getYi() << ' ' << i.getYf() << "  next\n \n";}
            // ROS_INFO("Trimmed");
        this->theta_ref=normalizeAngle(trimmed_list.back().orientation);    
        std::cout << "THETA_REF  " << this->theta_ref<< std::endl;
    }

    Location Robot::closest_id(){
        Location closest = this->graph.findClosestLocation(this->position);
        return closest;
    }

    float Robot::normalizeAngle(float angle){
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
    
    void Robot::state_machine()
    {   
        Line current_line=trimmed_list[line_state];
        if (following)
        {
           path_supervisor();
        }
        else
        {  
            decision_making();
        }
        calculate_outputs();
    }

    void Robot::path_supervisor(){

        Line current_line=trimmed_list[line_state];
        
        if (line_state + 1 == trimmed_list.size() and fetching)
            magnet=true;

        if(current_line.getDist2End(position)<dist_thresh)
        {
            line_state++; 
            if (line_state < trimmed_list.size())
            {
                current_line=trimmed_list[line_state];
            }
            else 
            {
                following=false;   
                std::cout << "Reached Goal  " << current_box <<std::endl;
                if (current_box==4)
                {
                    V=0;
                    Vn=0;
                    omega=0;
                    following=false;
                    can_rotate=false;

                }         
            }               
        }
        
    }

    void Robot::decision_making()
    {
        Location box_location=graph.findVertexById(man.box_list[man.current_box].location)->getInfo();
        Location box_destiny=graph.findVertexById(man.box_list[man.current_box].destiny)->getInfo();

        
        if(fetching and switch_on)
        {   
            fetching=false;                  
            line_state=0; 
            magnet=true;  
            bool calculated=man.calculate_destinies(man.current_box);
            create_path(man.box_list[man.current_box].destiny);
            following=true;
            can_rotate=false;
            last_loc=graph.findVertexById(man.box_list[man.current_box].location)->getInfo();
            
        }
        else if(!fetching and (getDistBetweenLocations(position,box_location) < dist_thresh))
        {   
            line_state=0; 
            fetching= true;
            last_loc=graph.findVertexById(man.box_list[man.current_box].destiny)->getInfo();
            man.complete_operation();
            if (man.current_box==4){ //finished
                magnet=false;     
                create_path(5);
                current_box=4;
                following=true;
                can_rotate=false;
                theta_ref*=-1;
                
            } 
            else{                 
                create_path(man.box_list[man.current_box].location);
                following=true;
                can_rotate=false;
                magnet=false;

            }
            
        }
        
    }

    void Robot::calculate_outputs()
    {
        Line current_line=trimmed_list[line_state];

        if (following)
        {   
            bool final = line_state +1 == trimmed_list.size();
            follow(current_line,final);
            if(final && current_line.getDist2End(position)<slow_zone){

                if(std::abs(normalizeAngle(theta_ref-theta)) > 0.2){
                    V=0;
                    Vn=0;
                }
            }
            if (can_rotate){
                rotate();
            }
            else{
                if (getDistBetweenLocations(last_loc,position)>rotate_dist)
                {
                    can_rotate=true;
                }
            }
        }
    }

    bool Robot::init_comms(){
        
        while (!comms->is_connected){
            try{
                comms->connect();
            }catch(const char *msg){
                std::cout << msg << std::endl;
                return false;
            }
        
        }
        return true;

        std::string udp_srv_msg = "STOP";
        int retries = 2;

        while(!udp_srv_msg.compare("STOP")){
            try{
                comms->sendMsg("IWP");
                udp_srv_msg = comms->rcvMsg();

                if(udp_srv_msg.empty() && retries==0){
                    std::cout << "UDP Server not responding" << std::endl;
                    return 0; 
                }

                if(udp_srv_msg.empty()){
                    udp_srv_msg = "STOP";
                }

                retries--;
                
            }catch(const char *msg){
                std::cout << msg << std::endl;
                return 0;
            }    
        }

    
    }

    std::string Robot::receive_box_msg()
    {
        std::string udp_srv_msg = "STOP";
        int retries = 2;

        while(!udp_srv_msg.compare("STOP")){
            try{
                comms->sendMsg("IWP");
                udp_srv_msg = comms->rcvMsg();

                if(udp_srv_msg.empty() && retries==0){
                    std::cout << "UDP Server not responding" << std::endl;
                    return 0; 
                }

                if(udp_srv_msg.empty()){
                    udp_srv_msg = "STOP";
                }

                retries--;
                
            }catch(const char *msg){
                std::cout << msg << std::endl;
                return 0;
            }    
        }
        comms->disconnect();
        std::cout << "BOX ORDER " << udp_srv_msg  << std::endl;
        man.get_boxes(udp_srv_msg);
        return udp_srv_msg;
    }

    std::string Robot::init()
    {
        bool success = init_comms();
        if (!success) 
            return "Fail";
        
        std::string udp_srv_msg = receive_box_msg();
        man.current_box = man.findPriorityBox(); 
        create_path(man.box_list[man.current_box].location);
        following=true;
        last_loc= Location(0,0,0.1);

        return udp_srv_msg;

    }
    std::string Robot::init_nocomms()
    {
        
        
        std::string udp_srv_msg = "BBBB";
        man.get_boxes(udp_srv_msg);
        man.current_box = man.findPriorityBox(); 
        create_path(man.box_list[man.current_box].location);
        last_loc= Location(0,0,0.1);

        return udp_srv_msg;

    }
    