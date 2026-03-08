#include "simulation_manager.h"
#include <nlohmann/json.hpp>
#include <cstring>
#include <iostream>
#include <cmath>

inline double round4(double val) {
    return std::round(val * 10000.0) / 10000.0;
}

// Constructor initializes viewer, UDP sender, and UDP receiver
SimulationManager::SimulationManager(const char* modelFile,
                                     const std::string& sendIp,
                                     uint16_t sendPort,
                                     uint16_t recvPort)
    : viewer(modelFile),
      sender(sendIp, sendPort),
      receiver(recvPort)
{
    loadModelNames();
    initializeIds();
    receiver.startReceiveThread();
}

// Step simulation, receive control via UDP, apply it, and send state via UDP
void SimulationManager::stepAndSend(double timestep) {
    // --- Receive latest UDP control input ---
    std::vector<uint8_t> buf;
    if (receiver.getLatestPacket(buf)) {
        try {
            std::string s(buf.begin(), buf.end());
            auto j = nlohmann::json::parse(s);

            if (j.contains("effort")) {
                auto ctrl = j["effort"].get<std::vector<double>>();
                mjData* d = viewer.data();
                mjModel* m = viewer.model();
                int nu = m->nu;
                for (int i = 0; i < nu && i < (int)ctrl.size(); i++) {
                    d->ctrl[i] = ctrl[i];
                }
            }
        } catch (std::exception& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
    }

    // --- Step simulation ---
    viewer.stepSimulation(timestep);
    mj_tendon(viewer.model(), viewer.data());

    // --- Serialize and send state via UDP ---
    std::vector<uint8_t> outbuf = serializeData();
    sender.send(outbuf);

    viewer.render();
    viewer.pollEvents();
}


// Process tendon Jacobian: slice and flatten
std::vector<double> SimulationManager::processTenJ(int rows, int cols, int start_id) {
    mjData* d = viewer.data();
    std::vector<double> result;
    for (int j = start_id; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            result.push_back(round4(d->ten_J[i*cols + j]));
        }
    }
    return result;
}

// Serialize simulation state into JSON and convert to byte vector
std::vector<uint8_t> SimulationManager::serializeData() {
    mjData* d = viewer.data();
    mjModel* m = viewer.model();

    nlohmann::json j;
    j["sim_time"] = d->time;
    j["joint_names"] = joint_names;
    j["tendon_names"] = tendon_names;
    j["qpos"] = std::vector<double>(d->qpos, d->qpos + m->nq);
    j["qvel"] = std::vector<double>(d->qvel, d->qvel + m->nv);
    j["joint_qpos_ids"] = joint_qpos_ids;
    j["joint_qvel_ids"] = joint_qvel_ids;
    j["length"] = std::vector<double>(d->ten_length, d->ten_length + tendon_names.size());
    j["ten_J_processed"] = processTenJ(tendon_names.size(), m->nv, joint_qvel_ids[0]);

    // --- joint_qpos / joint_qvel ---
    std::vector<double> joint_qpos_values(joint_qpos_ids.size());
    std::vector<double> joint_qvel_values(joint_qvel_ids.size());
    for (size_t i = 0; i < joint_qpos_ids.size(); i++) {
        joint_qpos_values[i] = (joint_qpos_ids[i] >= 0) ? d->qpos[joint_qpos_ids[i]] : 0.0;
        joint_qvel_values[i] = (joint_qvel_ids[i] >= 0) ? d->qvel[joint_qvel_ids[i]] : 0.0;
    }
    j["joint_qpos"] = joint_qpos_values;
    j["joint_qvel"] = joint_qvel_values;

    // --- tendon actuator force (tension) ---
    std::vector<double> tension_values(tension_sensor_ids.size());
    for (size_t i = 0; i < tension_sensor_ids.size(); i++) {
        if (tension_sensor_ids[i] >= 0) {
            tension_values[i] = round4(d->sensordata[tension_sensor_ids[i]]);
        } else {
            tension_values[i] = 0.0; // sensor not found
        }
    }
    j["tension"] = tension_values;  // store as array in JSON

    std::string s = j.dump();
    return std::vector<uint8_t>(s.begin(), s.end());
}

// Initialize joint, tendon, actuator, and sensor IDs
void SimulationManager::initializeIds() {
    mjModel* m = viewer.model();

    // --- Initialize joints ---
    for (const auto& name : joint_names) {
        int jid = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        joint_ids.push_back(jid);
        joint_qpos_ids.push_back(jid >= 0 ? m->jnt_qposadr[jid] : -1);
        joint_qvel_ids.push_back(jid >= 0 ? m->jnt_dofadr[jid] : -1);
    }

    // --- Initialize tendons, actuators, and corresponding tension sensors ---
    for (const auto& name : tendon_names) {
        // Tendon ID
        tendon_ids.push_back(mj_name2id(m, mjOBJ_TENDON, name.c_str()));

        // Actuator ID
        actuator_ids.push_back(mj_name2id(m, mjOBJ_ACTUATOR, name.c_str()));

        // Corresponding tendon actuator force sensor ID
        int sid = mj_name2id(m, mjOBJ_SENSOR, name.c_str());
        if (sid >= 0 && m->sensor_type[sid] == mjSENS_TENDONACTFRC) {
            tension_sensor_ids.push_back(m->sensor_adr[sid]);
        } else {
            tension_sensor_ids.push_back(-1); // sensor not found
        }
    }
}

// Load joint and tendon names from the model for JSON output
void SimulationManager::loadModelNames()
{
    mjModel* m = viewer.model();
    joint_names.clear();
    tendon_names.clear();

    // joints
    for (int i = 0; i < m->njnt; i++) {
        const char* name = mj_id2name(m, mjOBJ_JOINT, i);
        if (name) {
            joint_names.emplace_back(name);
        }
    }

    // tendons
    for (int i = 0; i < m->ntendon; i++) {
        const char* name = mj_id2name(m, mjOBJ_TENDON, i);
        if (name) {
            tendon_names.emplace_back(name);
        }
    }

    for (auto& j : joint_names)
        std::printf("joint: %s, ", j.c_str());

    for (auto& t : tendon_names)
        std::printf("tendon: %s, ", t.c_str());
    std::printf("\n");

}

// Return whether the viewer window should close
bool SimulationManager::shouldClose() const {
    return viewer.shouldClose();
}
