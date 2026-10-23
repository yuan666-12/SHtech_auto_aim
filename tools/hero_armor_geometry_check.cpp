#include "common.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
#define private public
#include "CoordTransformer.hpp"
#undef private
namespace {
int checks=0;
void require(bool value,const char *message) {++checks;if(!value)throw std::runtime_error(message);}
cv::FileStorage yaml(const std::string &body) {
    return cv::FileStorage("%YAML:1.0\n---\n"+body,cv::FileStorage::READ|cv::FileStorage::MEMORY);
}
void projection_checks(mathutils::CoordTransformer &transformer) {
    const auto model=transformer.armor_model(2,1);
    for(double distance:{2.,4.,6.})for(double yaw:{-.5,-.2,.2,.5}) {
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(model,cv::Vec3d(0,yaw,0),cv::Vec3d(.1,.03,distance),transformer.F_MAT,transformer.C_MAT,pixels);
        cv::Point2f points[4];for(int i=0;i<4;++i)points[i]=pixels[i];
        float camera_yaw=0;Eigen::Vector4d z;
        require(transformer.pnp_get_measurement(points,2,1,0,Eigen::Matrix3d::Identity(),camera_yaw,z),"measured geometry solves its known projected observations");
        const Eigen::Vector3d world(z[1],z[0],z[2]);
        require((transformer.pw_to_pc(world,Eigen::Matrix3d::Identity())-Eigen::Vector3d(.1,.03,distance)).norm()<.002,
                "known camera position is recovered within two millimetres across distance and yaw");
    }
}
}
int main(int argc,char **argv) {
    try {
        if(argc<2)throw std::runtime_error("repository root required");
        setenv("AUTOAIM_IGNORE_ARMOR_ID","1",1);
        mathutils::CoordTransformer transformer(std::string(argv[1])+"/asset/camParam/cam_param_hero.yml",false);
        const auto small=transformer.armor_model(1,1);
        require(std::abs(small[3].x-small[0].x-.1275)<1e-12,"Hero uses measured light-bar centre spacing");
        require(std::abs(small[1].y-small[0].y-.055)<1e-12,"Hero uses measured luminous length");
        require(transformer.armor_model(2,0)[0]==small[0],"red and blue share the same measured fixture dimensions");
        projection_checks(transformer);
        setenv("AUTOAIM_IGNORE_ARMOR_ID","0",1);
        require(std::abs(transformer.armor_model(1,1)[3].x-.114)<1e-12,"large-armor policy retains its original model");
        require(std::abs(transformer.armor_model(2,1)[3].x-.06375)<1e-12,"small-armor policy uses measured configuration");
        mathutils::CoordTransformer fallback(false);
        auto empty=yaml("other: 1\n");fallback.load_armor_geometry(empty);
        require(std::abs(fallback.armor_model(2,1)[3].x-.0675)<1e-12,"unconfigured devices retain the old dimensions");
        for(const std::string &body:{"armor_small_width_m: 0.1275\n","armor_small_width_m: -1\narmor_small_height_m: .055\n","armor_small_width_m: 0.1275\narmor_small_height_m: .Nan\n"}) {
            bool rejected=false;try{auto invalid=yaml(body);fallback.load_armor_geometry(invalid);}catch(const std::exception&){rejected=true;}
            require(rejected,"partial, negative and nonfinite dimensions are rejected");
        }
        std::cout<<"PASS "<<checks<<" measured-geometry/projection/fallback checks; no hardware opened\n";
        return 0;
    }catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
