#include "Planner.hpp"
#include "IESEKF_Double_Armor.hpp"
#include <iostream>
#include <stdexcept>
using namespace mathutils;
int n=0;
void check(bool ok,const char* msg){++n;if(!ok)throw std::runtime_error(msg);}
int main(int argc,char**argv){try{
 using F=IESEKF_Double_Armor<4,11>;
 using X=F::Matrix_x1d;using P=F::Matrix_xxd;using Z=F::Matrix_z1d;
 F::Matrix_zxd H=F::Matrix_zxd::Zero();
 H(0,0)=H(1,2)=H(2,4)=H(3,6)=1;
 X initial=X::Zero();initial(6)=179*M_PI/180;
 F f([](const X&x){return x;},
     [H](const X&x,int){return (H*x).eval();},
     [](const X&){return P::Identity().eval();},
     [H](const X&,int){return H;},
     [](const X&){return P::Zero().eval();},
     [](const Z&){return (F::Matrix_zzd::Identity()*1e-6).eval();},
     P::Identity(),initial,
     [](const X&a,const X&b){return (a+b).eval();},
     [](const X&a,const X&b){return (a-b).eval();},5,1e-12);
 f.predict();Z z;z<<3,.2,.3,-179*M_PI/180;
 auto state=f.update(z,0);
 check(std::abs(state(0)-3)<1e-4,"IEKF linear iteration");
 check(std::abs(limit_rad(state(6)-z(3)))<1e-5,"IEKF yaw wrap");
 check((f.get_P()-f.get_P().transpose()).norm()<1e-12,"IEKF covariance");
 f.reset(initial);check((f.get_P()-P::Identity()).norm()<1e-12,"IEKF reset");
 for(bool drag:{false,true}){
  auto s=solve_ballistic_trajectory({0,4,.5},10,drag);
  check(s.flight_time>0,"ballistic reachable");
  const double h=10*std::sin(s.pitch)*s.flight_time-.5*g*s.flight_time*s.flight_time;
  check(std::abs(h-.5)<1e-10,"ballistic height consistency");
  const double k=.5*1.225*(2*M_PI*.021*.021)/2/.041;
  const double d=drag?std::log1p(k*10*std::cos(s.pitch)*s.flight_time)/k:10*std::cos(s.pitch)*s.flight_time;
  check(std::abs(d-4)<1e-10,"ballistic distance consistency");
 }
 check(cal_fly_time({0,218000,0},10,false)<0,"reject impossible range");
 check(cal_fly_time({0,3,0},0,false)<0,"reject zero speed");
 check(cal_fly_time({0,3,NAN},10,false)<0,"reject NaN");
 check((near_radial_point({0,3,.3},.26)-Pos3D(0,2.74,.3)).norm()<1e-12,"radial point");
 RobotCommand cmd;cmd.yaw_speed=10;cmd.pitch_speed=-5;
 auto c=command_extrapolation(cmd,.1);
 check(std::abs(c.yaw_angle-1)<1e-6&&std::abs(c.pitch_angle+.5)<1e-6,"PC angle extrapolation");
 CoordTransformer::Init(std::string(argv[1])+"/asset/camParam/cam_param_hero.yml",false);
 auto&ct=CoordTransformer::Get();
 for(auto att:{Attitude(0,0,0),Attitude(20,15,0),Attitude(-15,5,3)}){
  Pos3D pc(.2,.1,3);
  check((ct.pw_to_pc(ct.pc_to_pw(pc,att.R_world2imu()),att.R_world2imu())-pc).norm()<1e-10,"coordinate inverse");
 }

 cv::FileStorage fs(std::string(argv[1])+"/asset/camParam/cam_param_hero.yml",cv::FileStorage::READ);
 cv::Mat Rcv,K,D;fs["R_c2i"]>>Rcv;fs["K"]>>K;fs["D"]>>D;
 Eigen::Matrix3d Rci,Rwa;cv::cv2eigen(Rcv,Rci);Rwa<<1,0,0,0,0,1,0,-1,0;
 for(auto att:{Attitude(0,0,0),Attitude(20,15,0),Attitude(-15,5,3)}){
  const Pos3D center(0,3,.3), camera=ct.pw_to_pc(center,att.R_world2imu());
  const Eigen::Matrix3d Rca=Rci.transpose()*att.R_world2imu()*Rwa;
  cv::Mat mat,rv,tv;cv::eigen2cv(Rca,mat);cv::Rodrigues(mat,rv);cv::eigen2cv(camera,tv);
  std::vector<cv::Point2d> pixels;cv::projectPoints(pw_blue_small,rv,tv,K,D,pixels);
  cv::Point2f corners[4];for(int i=0;i<4;++i)corners[i]=pixels[i];
  float cyaw=0;Eigen::Vector4d z;
  check(ct.pnp_get_measurement(corners,2,0,att.yaw()*M_PI/180,att.R_world2imu(),cyaw,z),"PnP valid");
  check((z.head<3>()-Eigen::Vector3d(3,0,.3)).norm()<1e-4,"stationary target world position");
  check(std::abs(limit_rad(z(3)))<.002,"stationary target world normal");
  auto pixel=ct.pc_to_pu(camera);std::vector<cv::Point2d> projected;
  cv::projectPoints(std::vector<cv::Point3d>{{camera.x(),camera.y(),camera.z()}},cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,D,projected);
  check(std::hypot(pixel.x()-projected[0].x,pixel.y()-projected[0].y)<1e-8,"distorted overlay projection");
 }
 predict::Planner planner(argv[2],false);
 Target target;target.predictor_state=TrackingState::DETECTING;
 target.armor_x_measurement<<2;target.armor_y_measurement<<3;target.armor_z_measurement<<1;
 target.armor_x_state<<2,0;target.armor_y_state<<3,0;target.armor_z_state<<1,0;
 for(auto att:{Attitude(0,0,0),Attitude(20,15,0)}){
  const auto&p=planner.make_plan(target,10,att.yaw()*M_PI/180,att.pitch()*M_PI/180,att.R_world2imu(),predict::Planner::TP::clock::now());
  check(std::abs(p.target_yaw-std::atan2(-2.,3.))<1e-9,"absolute yaw independent of pose");
  check(std::abs(p.target_pitch-std::atan2(1.,std::hypot(2.,3.)))<1e-9,"full horizontal pitch");
 }
 predict::Planner runtime(std::string(argv[1])+"/asset/plannerParam/planner_param_hero.yml",false);
 target.armor_y_measurement<<218000;
 const auto&p=runtime.make_plan(target,10,0,0,Attitude().R_world2imu(),predict::Planner::TP::clock::now());
 check(p.aimed_target_type==AimedTargetType::NONE&&p.target_distance==0,"planner rejects impossible range");
 CoordTransformer::Destroy();
 std::cout<<"PASS "<<n<<" checks; no camera or UART opened\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}
