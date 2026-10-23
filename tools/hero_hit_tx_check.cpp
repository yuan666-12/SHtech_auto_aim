// Exercise the real driver with in-memory packets and no UART descriptor.
#include "common.hpp"
#include "hit_diagnostics.hpp"
#include <atomic>
#include <cstring>
#include <thread>
#define private public
#include "xuc_driver.hpp"
#undef private
int main(int argc,char **argv) {
    if(argc!=2)return 1;
    setenv("AUTOAIM_HIT_LOG_FILE",argv[1],1);setenv("AUTOAIM_XUC_DRY_RUN","1",1);
    setenv("AUTOAIM_TRACK_ONLY","1",1);
    Log console;screen=&console;
    XucDriver driver("no-hardware");
    fire_control::Input in;in.now=in.rx_time=fire_control::Clock::now();
    fire_control::Gate gate;
    fire_control::Diagnostics d;d.valid=d.synchronized=true;d.input=in;d.decision=gate.evaluate(in);
    d.fire_config=gate.config();d.center_config=gate.center_config();d.planned_yaw=4.5;d.planned_pitch=10;
    driver.set_fire_diagnostics(d);
    driver.transmit_cmd(3,0,10,0,0,0,3,2,2); // Constructed packet, track-only forces fire byte zero.
    driver.transmit_cmd(3,0,NAN,0,0,0,3,2,2); // Invalid command must be captured without a write.
    setenv("AUTOAIM_XUC_DRY_RUN","0",1);
    driver.transmit_cmd(3,0,10,0,0,0,3,2,2); // Link is disabled, so early return is also captured.
    if(driver.stm32_fd_!=-1)throw std::runtime_error("unexpected hardware open");
    std::cout<<"PASS actual TX diagnostic hooks: dry-run, invalid command, disabled link; no UART opened\n";
}
