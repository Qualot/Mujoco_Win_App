#include "simulation_manager.h"
#include <cstdio>
#include <cstdlib>

int main(int argc, const char** argv) {
    if (argc != 5) {
        printf("USAGE: app modelfile send_ip send_port recv_port\n");
        return EXIT_FAILURE;
    }

    const char* modelFile = argv[1];
    std::string sendIp = argv[2];
    uint16_t sendPort = static_cast<uint16_t>(std::stoi(argv[3]));
    uint16_t recvPort = static_cast<uint16_t>(std::stoi(argv[4]));

    // Initialize simulation manager
    SimulationManager sim(modelFile, sendIp, sendPort, recvPort);

    mjData* d = sim.data();
    mjModel* m = sim.model();

    int ct_verbose = 0;
    int ct_verbose_max = 10;

    while (!sim.shouldClose()) {
        sim.stepAndSend();

        if (ct_verbose == 0) {
            std::printf("time: %f, ", d->time);
            std::printf("qpos: %f %f %f\n", d->qpos[0], d->qpos[1], d->qpos[2]);
        }

        ct_verbose = (ct_verbose + 1) % ct_verbose_max;
    }

    return 0;
}
