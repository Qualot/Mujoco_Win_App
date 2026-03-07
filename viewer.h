#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// Viewer class encapsulating MuJoCo + GLFW visualization
class Viewer {
public:
    explicit Viewer(const char* filename);
    ~Viewer();

    // ======== Public API for external use ========

    // Step the simulation for the given timestep (default: 1/60 sec)
    void stepSimulation(double timestep = 1.0/60.0);

    // Render the current scene
    void render();

    // Process pending events (keyboard, mouse, window close)
    void pollEvents();

    // Reset simulation to keyframe 0
    void reset();

    // Reset simulation to keyframe 0
    void resetHigh();

    // Reset simulation to keyframe 1
    void resetLow();

    // Check if window should close
    bool shouldClose() const;

    // Access raw MuJoCo structures
    mjData* data();
    mjModel* model();

    // Pause flag (checked from SimulationManager)
    bool paused = false;   // toggle with 'p'

private:
    // MuJoCo structures
    mjModel* m = nullptr;
    mjData* d = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    // GLFW window
    GLFWwindow* window = nullptr;

    // Mouse state
    bool button_left   = false;
    bool button_middle = false;
    bool button_right  = false;
    double lastx = 0;
    double lasty = 0;

    // ======== GLFW static callbacks ========
    static void keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int act, int mods);
    static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    // ======== Instance event handlers ========
    void onKeyboard(int key, int act);
    void onMouseButton(int button, int act);
    void onMouseMove(double xpos, double ypos);
    void onScroll(double yoffset);
};
