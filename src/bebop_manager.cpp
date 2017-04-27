#define TF_EULER_DEFAULT_ZYX

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"
#include "quadrotor_code/Status.h"
#include "quadrotor_code/Neighbor.h"
#include <geodesy/utm.h>
#include "sensor_msgs/NavSatFix.h"
#include <tf/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <fstream>
using namespace std;
#define R 7
#define sendT 1
#define perror 0  //value
#define verror 0 //rate
double PI=acos(-1);
double stopdist = 1.5;

double delta_dis=5; //无人机之间的间距
int stand_vx=0, stand_vy=0;//作为标准的无人机的虚拟坐标
double stand_x, stand_y;//作为标准的无人机的实际坐标

class OdomHandle
{
    public:
    ros::Subscriber sub_yaw,sub_vel,sub_fix;
    ros::Publisher pub;
    ros::Publisher neighbor_pub;
    ros::Publisher stop_pub;
    double _px,_py,_vx,_vy;
    double _pz,_vz,_theta;
    int _r_id; 
    bool yawrcv,velrcv,fixrcv;
    
    double start_x,start_y;
    int vx,vy;
    double delta_x,delta_y;
    int count;
   
    OdomHandle(int r_id)
    {
        ros::NodeHandle n;
        //double xerror,yerror;
        yawrcv = false;
        velrcv = false;
        fixrcv = false;
        stringstream ss;
        ss<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/AttitudeChanged";
        sub_yaw = n.subscribe(ss.str(), 1000, &OdomHandle::yawcb,this);
 
        stringstream ss1;
        //ss1<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/SpeedChanged";
        ss1<<"/uav"<<r_id<<"/velctrl/input";
        sub_vel = n.subscribe(ss1.str(), 1000, &OdomHandle::velcb2,this);

        stringstream ss2;
        ss2<<"/uav"<<r_id<<"/fix";
        sub_fix = n.subscribe(ss2.str(), 1000, &OdomHandle::fixcb,this);
        
        stringstream ss3;
        ss3<<"/uav"<<r_id<<"/position";
        pub = n.advertise<quadrotor_code::Status>(ss3.str(),1000);
        
        stringstream ss4;
        ss4<<"/uav"<<r_id<<"/neighbor";
        neighbor_pub = n.advertise<quadrotor_code::Neighbor>(ss4.str(),1000);
        
        stringstream ss5;
        ss5<<"/uav"<<r_id<<"/stop";
        stop_pub = n.advertise<std_msgs::Int32>(ss5.str(),1000);
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _pz=0;
        _vz=0;
        _r_id = r_id;
        vx=r_id/3;
        vy=r_id%3;
        count = 0;
      
    }
    //NEU -> NWU
    void yawcb(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr & msg)
    {
        double msgtheta = msg->yaw;
        _theta = -msgtheta;
        /*
        if (msgtheta >= 0)
        {
             _theta = PI/2 - msgtheta;
        }
        else
        {
            if(msgtheta > -PI/2)
            {
                _theta = PI/2 - msgtheta;
            }
            else
            {
                _theta = - 3*PI/2 -msgtheta;
            }
        }*/
        yawrcv = true;
    }

    void velcb(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr & msg)
    {
        _vx = msg->speedX;
        _vy = -msg->speedY;//NEU -> NWU
        velrcv = true;
    }
    
    void velcb2(const geometry_msgs::Twist::ConstPtr & msg)
    {
        _vx = msg->linear.x;
        _vy = -msg->linear.y;//NEU -> NWU
        velrcv = true;
    }
    
    void fixcb(const sensor_msgs::NavSatFix::ConstPtr & msg)
    {
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = msg->latitude;
        geo_pt.longitude = msg->longitude;
        geo_pt.altitude = msg->altitude;
        geodesy::UTMPoint utm_pt(geo_pt);
        _py = utm_pt.easting;
        _px = utm_pt.northing;
        
        if(count<30)
        {
            start_x=utm_pt.northing;
            start_y=utm_pt.easting;

            if(vx==stand_vx && vy==stand_vy && count==15)
            {
                stand_x=start_x;
                stand_y=start_y;
            }

            if(count==29)
            {
                delta_x=((vx-stand_vx)*delta_dis+stand_x)-start_x;
                delta_y=((vy-stand_vy)*delta_dis+stand_y)-start_y;
                cout<<"uav"<<_r_id<<" aligned"<<endl;
            }

            count++;
        }
        else
        {
           _py = _py+delta_y - stand_y;
           _px = _px+delta_x - stand_x;
        }
        
        fixrcv = true;
        _py = - _py;//NEU -> NWU
    }
};

static vector<OdomHandle*> odom_list;
int robotnum=50;
vector<vector<int> > adj_list;

double dist(int i,int j)
{
    double re=pow(odom_list[i]->_px-odom_list[j]->_px,2)+pow(odom_list[i]->_py-odom_list[j]->_py,2);
    //if(i==4)
    //cout<<re<<endl;
    return sqrt(re);
    
}

int main(int argc, char** argv)
{

   ros::init(argc,argv,"sim_manager");
   ros::NodeHandle n;
   srand(time(0));
   bool param_ok = ros::param::get ("~robotnum", robotnum);
   //ofstream fout("/home/liminglong/czx/traject.txt");
   //ofstream fout2("/home/liminglong/czx/velocity.txt");
   ros::Publisher posepub = n.advertise<geometry_msgs::PoseArray>("/swarm_pose",1000);
   for(int i=0;i<robotnum;i++)
   {
      OdomHandle *p=new OdomHandle(i);
      odom_list.push_back(p);
      adj_list.push_back(vector<int>());
   }
   //neighbor_list.push_back(NeighborHandle(1));
   ros::Rate loop_rate(20);
   int count = 1;
   while(ros::ok())
   {
      geometry_msgs::PoseArray sendpose;
       sendpose.header.frame_id="world";
      for(int i=0;i<robotnum;i++)
      {
          geometry_msgs::Pose p;
         
          
          p.position.x = odom_list[i]-> _px;
          p.position.y = odom_list[i]-> _py;
          p.position.z = odom_list[i]-> _pz;
          
          tf::Quaternion q(odom_list[i]->_theta,0,0);
          p.orientation.x = q.x();
          p.orientation.y = q.y();
          p.orientation.z = q.z();
          p.orientation.w = q.w();
          sendpose.poses.push_back(p);
      }
      posepub.publish(sendpose);
      
      ros::spinOnce();
      for(int i=0;i<robotnum;i++)
      {
           for(int j=i+1;j<robotnum;j++)
           {
                if(dist(i,j)>0&&dist(i,j)<R)
                {
                    adj_list[i].push_back(j);
                    adj_list[j].push_back(i);
                }
                if(dist(i,j) < stopdist)
                {
                    std_msgs::Int32 stopmsg;
                    for(int k=0;k<robotnum;k++)
                        odom_list[k]->stop_pub.publish(stopmsg);
                }
           }
           quadrotor_code::Status sendmsg;
           sendmsg.px = odom_list[i]-> _px;
           sendmsg.py = odom_list[i]-> _py;
           sendmsg.vx = odom_list[i]-> _vx;
           sendmsg.vy = odom_list[i]-> _vy;
           sendmsg.theta =  odom_list[i]-> _theta;
           odom_list[i]->pub.publish(sendmsg);
      }
      
      for(int i=0;i<robotnum;i++)
      {
           quadrotor_code::Neighbor sendmsg;
           sendmsg.data = adj_list[i];
           odom_list[i]->neighbor_pub.publish(sendmsg);
           adj_list[i]=vector<int>();
      }
      /*if(count%10==0)
      {
          fout2<<count/10*0.5;
          for(int i=0;i<robotnum;i++)
          {
              double vx=odom_list[i]->_vx;
              double vy=odom_list[i]->_vy;
              double px=odom_list[i]->_px;
              double py=odom_list[i]->_py;
              fout2<<' '<<sqrt(vx*vx+vy*vy);
              fout<<px<<' '<<py<<' ';
          }
          fout2<<endl;
          fout<<endl;
      }
      */
      count++;
      loop_rate.sleep();
   }
   return 0;
}
