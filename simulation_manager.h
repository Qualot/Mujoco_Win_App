#pragma once
#include "viewer.h"
#include "udp_sender.h"
#include "udp_receiver.h"
#include <vector>
#include <string>
#include <cstdint>

// SimulationManager class manages MuJoCo simulation, UDP send/receive, and control updates
class SimulationManager {
public:
    SimulationManager(const char* modelFile,
                      const std::string& sendIp,
                      uint16_t sendPort,
                      uint16_t recvPort);

    void stepAndSend(double timestep = 1.0/60.0);
    bool shouldClose() const;

    mjModel* model() { return viewer.model(); }
    mjData*  data()  { return viewer.data(); }

private:
    Viewer viewer;
    UDPSender sender;
    UDPReceiver receiver;

    std::vector<int> joint_ids;
    std::vector<int> tendon_ids;
    std::vector<int> actuator_ids;

    std::vector<int> joint_qpos_ids;
    std::vector<int> joint_qvel_ids;

    std::vector<int> tension_sensor_ids;  // Tendon actuator force sensor indices in sensordata

    std::vector<std::string> joint_names;
    std::vector<std::string> joint_names_rpy; // expand free joints and ball joints into separate roll/pitch/yaw entries
    std::vector<std::string> tendon_names;

    std::vector<double> processTenJ(int rows, int cols, int start_id);
    std::vector<uint8_t> serializeData();
    void initializeIds();
    void loadModelNames();
};
