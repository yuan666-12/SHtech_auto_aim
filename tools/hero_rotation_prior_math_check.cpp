#include "rotation_speed_prior.hpp"
#include <iostream>
#include <stdexcept>
int checks=0;
void require(bool ok,const char *message){++checks;if(!ok)throw std::runtime_error(message);}
int main(int argc,char **argv){try{
    if(argc>1){bool rejected=false;try{rotation_speed::Config::from_environment(true);}catch(const std::invalid_argument &){rejected=true;}
        require(rejected,"invalid period is rejected before runtime hardware initialization");std::cout<<"PASS invalid configuration\n";return 0;}
    using namespace rotation_speed;
    Config c;c.enabled=true;Prior p(c);const double speed=c.magnitude();
    require(std::abs(speed-4.487989505128276)<1e-12,"1.4 s period has correct rad/s units");
    for(int i=0;i<100;++i)p.observe(-speed,true,false,.02);
    require(!p.active(),"untrusted startup cannot fix direction");
    for(int i=0;i<10;++i)p.observe(-speed,true,true,.02);
    require(!p.active(),"short opposite startup does not confirm");
    int events=0;
    for(int i=0;i<40;++i)events+=p.observe(speed,true,true,.02);
    require(p.active() && p.direction()==1 && events==1,"positive direction confirms exactly once");
    for(int i=0;i<200;++i)events+=p.observe(speed,true,true,.02);
    require(events==1,"same period is not reapplied as repeated independent data");
    p.observe(speed,false,true,.02);
    require(p.active(),"short losses preserve confirmed direction");
    p.observe(speed,false,false,.02);
    require(!p.active(),"loss of model trust clears direction");
    for(int i=0;i<40;++i)p.observe(-speed,true,true,.02);
    require(p.active() && p.direction()==-1,"fresh negative direction confirms");
    p.observe(speed,true,true,.02);
    require(!p.active(),"contradictory speed releases the old confirmation");
    for(int i=0;i<100;++i)p.observe(.1,true,true,.02);
    require(!p.active(),"near-zero startup cannot pick a direction");
    for(double dt:{1e-6,1e-4,.01,.02,.1,1.}) {
        auto q=p.noise(dt);
        require(q.phase>0 && q.speed>0 && q.phase*q.speed>=q.cross*q.cross,
                "time-scaled angular noise is positive semidefinite");
    }
    double pp=0,pw=0,ww=0;
    for(int i=0;i<10;++i) {
        const double dt=.02;const auto q=p.noise(dt);
        pp+=2*dt*pw+dt*dt*ww+q.phase;pw+=dt*ww+q.cross;ww+=q.speed;
    }
    auto q=p.noise(.2);
    require(std::abs(pp-q.phase)<1e-15 && std::abs(pw-q.cross)<1e-15 &&
            std::abs(ww-q.speed)<1e-15,"Q does not depend on splitting one interval into frames");
    require(!Config::from_environment(false).enabled,"free target model never enables the speed prior");
    std::cout<<"PASS "<<checks<<" direction, prior lifetime and covariance checks\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
