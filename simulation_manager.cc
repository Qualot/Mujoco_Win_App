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
    j["joint_names_rpy"] = joint_names_rpy;
    j["tendon_names"] = tendon_names;
    j["qpos"] = std::vector<double>(d->qpos, d->qpos + m->nq);
    j["qpos_per_joint"] = serializeJointQposPerJoint();
    j["joint_rpy"] = serializeJointRPY();
    j["qpos_rpy_theta_l"] = serializeQposRPYTheta();
    j["qvel"] = std::vector<double>(d->qvel, d->qvel + m->nv);
    j["qvel_rpy_theta_l"] = serializeQvelRPYTheta();
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
    joint_names_rpy.clear();
    tendon_names.clear();

    // joints
    for (int i = 0; i < m->njnt; i++) {
        const char* name = mj_id2name(m, mjOBJ_JOINT, i);
        if (name) {
            joint_names.emplace_back(name);

            int jtype = m->jnt_type[i];
            if (jtype == mjJNT_FREE) {
                // joint_names_rpy.push_back(std::string(name) + "_tx");
                // joint_names_rpy.push_back(std::string(name) + "_ty");
                // joint_names_rpy.push_back(std::string(name) + "_tz");
                joint_names_rpy.push_back(std::string(name) + "_rx");
                joint_names_rpy.push_back(std::string(name) + "_ry");
                joint_names_rpy.push_back(std::string(name) + "_rz");
            }
            else if (jtype == mjJNT_BALL) {
                joint_names_rpy.push_back(std::string(name) + "_rx");
                joint_names_rpy.push_back(std::string(name) + "_ry");
                joint_names_rpy.push_back(std::string(name) + "_rz");
            }else
            {
                joint_names_rpy.push_back(std::string(name));
            }


        }
    }

    // tendons
    for (int i = 0; i < m->ntendon; i++) {
        const char* name = mj_id2name(m, mjOBJ_TENDON, i);
        if (name) {
            tendon_names.emplace_back(name);
        }
    }

    
    std::printf("joint: ");
    for (auto& j : joint_names)
        std::printf("%s, ", j.c_str());

    std::printf("\n");
    std::printf("tendon: ");
    for (auto& t : tendon_names)
        std::printf("%s, ", t.c_str());

    std::printf("\n\n");

}

// Serialize joint qpos values per joint into JSON object with joint names as keys
nlohmann::json SimulationManager::serializeJointQposPerJoint()
{
    mjModel* m = viewer.model();
    mjData*  d = viewer.data();

    nlohmann::json j;

    for (int jid = 0; jid < m->njnt; jid++)
    {
        const char* name = mj_id2name(m, mjOBJ_JOINT, jid);
        if (!name) continue;

        int start = m->jnt_qposadr[jid];

        int end;
        if (jid < m->njnt - 1)
            end = m->jnt_qposadr[jid + 1];
        else
            end = m->nq;

        std::vector<double> q;

        for (int i = start; i < end; i++)
            q.push_back(round4(d->qpos[i]));

        j[name] = q;
    }

    return j;
}

#include "mujoco/mujoco.h"  // mju_quat2Mat
#include <cmath>
#include <vector>

// --- quaternion 2 RPY (ZYX, ROS compatible) ---
static std::vector<double> quat2rpy_zyx(const mjtNum* quat)
{
    mjtNum R[9];
    mju_quat2Mat(R, quat);

    // ZYX (Yaw-Pitch-Roll) order
    double yaw   = std::atan2(R[1], R[0]);
    double pitch = std::asin(-R[2]);
    double roll  = std::atan2(R[5], R[8]);

    // round to 4 decimal places for cleaner JSON output
    auto round4 = [](double val) { return std::round(val * 10000.0) / 10000.0; };

    return { round4(roll), round4(pitch), round4(yaw) };
}

// --- SimulationManager member function ---
nlohmann::json SimulationManager::serializeJointRPY()
{
    mjModel* m = viewer.model();
    mjData*  d = viewer.data();

    nlohmann::json j;

    for (int jid = 0; jid < m->njnt; jid++)
    {
        const char* name = mj_id2name(m, mjOBJ_JOINT, jid);
        if (!name) continue;

        int jtype = m->jnt_type[jid];

        // free or ball joint only
        if (jtype == mjJNT_FREE || jtype == mjJNT_BALL)
        {
            int start = m->jnt_qposadr[jid];
            const mjtNum* quat;

            if (jtype == mjJNT_FREE)
                quat = d->qpos + start + 3;  // free joint: pos3 + quat4
            else
                quat = d->qpos + start;      // ball joint: quat4

            j[name] = quat2rpy_zyx(quat);
        }
    }

    return j;
}

// --- Serialize qpos values but convert free/ball joints to RPY angles ---
std::vector<double> SimulationManager::serializeQposRPYTheta()
{
    mjModel* m = viewer.model();
    mjData*  d = viewer.data();

    std::vector<double> result;

    // Lambda for rounding values to 4 decimal places
    auto round4 = [](double val) { return std::round(val * 10000.0) / 10000.0; };

    for (int jid = 0; jid < m->njnt; jid++)
    {
        int jtype = m->jnt_type[jid];
        int start = m->jnt_qposadr[jid];

        if (jtype == mjJNT_FREE || jtype == mjJNT_BALL)
        {
            // For free or ball joints, extract quaternion
            const mjtNum* quat;
            if (jtype == mjJNT_FREE)
                quat = d->qpos + start + 3; // free joint: first 3 are position, next 4 are quaternion
            else
                quat = d->qpos + start;     // ball joint: quaternion only

            // Convert quaternion to rotation matrix
            mjtNum R[9];
            mju_quat2Mat(R, quat);

            // Convert rotation matrix to ZYX (Yaw-Pitch-Roll) Euler angles
            mjtNum euler[3];
            euler[0] = std::atan2(R[5], R[8]);  // roll (X-axis)
            euler[1] = std::asin(-R[2]);        // pitch (Y-axis)
            euler[2] = std::atan2(R[1], R[0]);  // yaw (Z-axis)

            // Append RPY to result vector
            result.push_back(round4(euler[0]));
            result.push_back(round4(euler[1]));
            result.push_back(round4(euler[2]));
        }
        else if (jtype == mjJNT_HINGE || jtype == mjJNT_SLIDE)
        {
            // For hinge or slide joints, append scalar qpos
            result.push_back(round4(d->qpos[start]));
        }
        else
        {
            // For other joint types, you may skip or append 0.0
            // result.push_back(0.0);
        }
    }

    return result;
}

// --- Serialize qvel values but convert free/ball joints to RPY rates ---
std::vector<double> SimulationManager::serializeQvelRPYTheta()
{
    mjModel* m = viewer.model();
    mjData*  d = viewer.data();

    std::vector<double> result;

    // Lambda for rounding values to 4 decimal places
    auto round4 = [](double val) { return std::round(val * 10000.0) / 10000.0; };

    for (int jid = 0; jid < m->njnt; jid++)
    {
        int jtype = m->jnt_type[jid];
        int qpos_start = m->jnt_qposadr[jid];
        int qvel_start = m->jnt_dofadr[jid];

        if (jtype == mjJNT_FREE)
        {
            // free joint: first 3 linear velocities, next 3 angular velocities
            // We only take angular velocity (RPY rates)
            result.push_back(round4(d->qvel[qvel_start + 0])); // roll rate
            result.push_back(round4(d->qvel[qvel_start + 1])); // pitch rate
            result.push_back(round4(d->qvel[qvel_start + 2])); // yaw rate
        }
        else if (jtype == mjJNT_BALL)
        {
            // ball joint: 3 angular velocity DOF
            result.push_back(round4(d->qvel[qvel_start + 0]));
            result.push_back(round4(d->qvel[qvel_start + 1]));
            result.push_back(round4(d->qvel[qvel_start + 2]));
        }
        else if (jtype == mjJNT_HINGE || jtype == mjJNT_SLIDE)
        {
            // scalar joint velocity
            result.push_back(round4(d->qvel[qvel_start]));
        }
        else
        {
            // other joint types: optional skip or zero
            // result.push_back(0.0);
        }
    }

    return result;
}

// Return whether the viewer window should close
bool SimulationManager::shouldClose() const {
    return viewer.shouldClose();
}
