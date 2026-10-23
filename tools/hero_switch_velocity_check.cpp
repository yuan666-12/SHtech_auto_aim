#include "Tracker.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
namespace {
int checks=0;
void require(bool value,const char *message) {
    ++checks;
    if (!value) throw std::runtime_error(message);
}
struct Fixture {
    predict::Tracker tracker;
    predict::Tracker::TP time=predict::Tracker::TP::clock::now()-10s;
    Eigen::Vector4d measurement{2.74,0.,.1,0.};
    Fixture() { tracker.reset_target(measurement,time); }
    const Target &step(int count=1) {
        time+=33ms;
        return tracker.track(measurement,Eigen::Vector4d::Zero(),count,2,time,0.);
    }
    void seed_rotation(double velocity,UpdatingModelType mode) {
        // Seed a converged yaw estimate to isolate the production switch path.
        auto &state=const_cast<Target &>(tracker.get_target());
        state.yaw_state[1]=velocity;
        state.updating_model_type=mode;
    }
    void next_plate(int index) {
        const double yaw=mathutils::limit_rad(index*M_PI_2);
        measurement<<3.-.26*std::cos(yaw),-.26*std::sin(yaw),.1,yaw;
    }
};
void check_switches(double velocity,UpdatingModelType mode) {
    Fixture f;
    f.seed_rotation(velocity,mode);
    for (int plate=1;plate<=5;++plate) {
        const Eigen::Vector3d previous=f.measurement.head<3>();
        f.next_plate(plate);
        require((f.measurement.head<3>()-previous).norm()>.2,"fixture crosses the spatial plate-switch threshold");
        const auto &state=f.step();
        require(std::abs(state.yaw_state[1]-velocity)<1e-12,"plate switch must preserve signed angular velocity");
        require(state.updating_model_type==mode,"plate switch must not force a rotating target into low-speed mode");
        require(state.vehicle_model_rotation_count==std::min(plate,3),"plate-switch history must accumulate and saturate at three");
        require(state.vehicle_model_trust==(plate>=3),"trust is earned only after three spatial switches");
        require(state.tracked_state.allFinite() && state.yaw_state.allFinite(),"model stays finite across repeated switches");
        require(state.observation_updated && !state.filter_reset && !state.measurement_rejected,"switch remains a valid observation without a full target reset");
    }
    const auto observed=f.tracker.get_target().last_observation_time;
    const auto &lost=f.step(0);
    require(lost.vehicle_model_trust && lost.vehicle_model_rotation_count==3,"one missing observation preserves trusted switch history");
    require(lost.last_observation_time==observed && !lost.observation_updated,"missing observation does not refresh freshness");
    f.next_plate(6);
    const auto &resumed=f.step();
    require(resumed.vehicle_model_trust && resumed.vehicle_model_rotation_count==3,"switch after a missing frame keeps vehicle confidence");
    f.time+=33ms;
    const auto &reset=f.tracker.reset_target(f.measurement,f.time);
    require(reset.yaw_state[1]==0.,"real target reset clears angular velocity");
    require(!reset.vehicle_model_trust && reset.vehicle_model_rotation_count==0,"real target reset clears vehicle confidence and history");
}
}
int main() {
    try {
        for (double velocity:{1.2,-1.2})check_switches(velocity,UpdatingModelType::BOTH);
        for (double velocity:{3.2,-3.2})check_switches(velocity,UpdatingModelType::VEHICLE_MODEL);
        Fixture slow;slow.seed_rotation(.3,UpdatingModelType::BOTH);slow.next_plate(1);
        const auto &low=slow.step();
        require(std::abs(low.yaw_state[1]-.3)<1e-12,"genuinely slow switch also retains its nonzero speed estimate");
        require(low.updating_model_type==UpdatingModelType::ARMOR_MODEL && !low.vehicle_model_trust,
                "existing low-speed model-selection policy remains intact");
        Fixture invalid;invalid.seed_rotation(std::numeric_limits<double>::quiet_NaN(),UpdatingModelType::BOTH);invalid.next_plate(1);
        require(invalid.step().yaw_state.allFinite(),"invalid old velocity cannot propagate through a switch");
        std::cout<<"PASS "<<checks<<" switch-velocity/model-history/reset assertions; no camera or UART opened\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr<<"FAIL after "<<checks<<" checks: "<<error.what()<<'\n';
        return 1;
    }
}
