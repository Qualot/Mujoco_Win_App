#include "viewer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

Viewer::Viewer(const char* filename) {
    // Load model
    char error[1000] = "Could not load binary model";
    if (std::strlen(filename)>4 && !std::strcmp(filename+std::strlen(filename)-4, ".mjb")) {
        m = mj_loadModel(filename, 0);
    } else {
        m = mj_loadXML(filename, 0, error, 1000);
    }
    if (!m) {
        mju_error("Load model error: %s", error);
    }

    // Create data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // Create window
    window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize camera/scene/context
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_defaultPerturb(&pert); // Added: Initialize perturbation structure

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Register callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyboardCallback);
    glfwSetCursorPosCallback(window, mouseMoveCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);
}

Viewer::~Viewer() {
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

// ======== Public API ========

void Viewer::stepSimulation(double timestep) {
    if (paused) return;

    mjtNum simstart = d->time;
    while (d->time - simstart < timestep) {
        // Apply perturbation forces to mjData before stepping
        mjv_applyPerturbPose(m, d, &pert, 0);
        mjv_applyPerturbForce(m, d, &pert);
        
        mj_step(m, d);
    }
}

void Viewer::render() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // Update scene with perturbation state (handles highlights)
    mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
    
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
}

void Viewer::pollEvents() {
    glfwPollEvents();
}

void Viewer::reset() {
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
}

void Viewer::resetHigh() {
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
}

void Viewer::resetLow() {
    mj_resetDataKeyframe(m, d, 1);
    mj_forward(m, d);
}

bool Viewer::shouldClose() const {
    return glfwWindowShouldClose(window);
}

mjData* Viewer::data() { return d; }
mjModel* Viewer::model() { return m; }

// ======== Static callbacks ========

void Viewer::keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (self) self->onKeyboard(key, act);
}

void Viewer::mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (self) self->onMouseButton(button, act);
}

void Viewer::mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (self) self->onMouseMove(xpos, ypos);
}

void Viewer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (self) self->onScroll(yoffset);
}

// ======== Instance event handlers ========

void Viewer::onKeyboard(int key, int act) {
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) reset();
    if (act==GLFW_PRESS && key==GLFW_KEY_H) resetHigh();
    if (act==GLFW_PRESS && key==GLFW_KEY_L) resetLow();
    if (act==GLFW_PRESS && key==GLFW_KEY_P) {
        paused = !paused;
        std::printf("Paused: %s\n", paused ? "ON" : "OFF");
    }
    if (act==GLFW_PRESS && key==GLFW_KEY_O) {
        mj_step(m, d);
        std::printf("One step advanced\n");
    }
}

void Viewer::onMouseButton(int button, int act) {
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);

    // Double-click detection
    if (act == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT) {
        double now = glfwGetTime();
        if (now - last_click_time < 0.3) {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // Variables to receive selection details
            int geomid = -1;
            int flexid = -1;
            int skinid = -1;
            mjtNum selpnt[3];

            // Standard 11-argument call for MuJoCo 3.x
            // Returns the ID of the selected body
            int bodyid = mjv_select(m, d, &opt, 
                                    (mjtNum)viewport.width / viewport.height, 
                                    (mjtNum)lastx / viewport.width, 
                                    (mjtNum)(viewport.height - lasty) / viewport.height, 
                                    &scn, selpnt, &geomid, &flexid, &skinid);
            
            if (bodyid >= 0) {
                // Set perturbation targets
                pert.active = bodyid; // Correct member name: 'active'
                pert.select = bodyid; // Correct member name: 'select'
                
                // Initialize perturbation based on selection point
                mjv_initPerturb(m, d, &scn, &pert);
            } else {
                // Reset if nothing selected
                pert.active = 0;
            }
        }
        last_click_time = now;
    }

    glfwGetCursorPos(window, &lastx, &lasty);
}

void Viewer::onMouseMove(double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_ctrl  = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // Perturbation logic: check if a body is active and Ctrl is pressed
    if (mod_ctrl && pert.active > 0) {
        mjtMouse action_pert;
        if (button_right)      action_pert = mjMOUSE_MOVE_V;   // Translation
        else if (button_left)  action_pert = mjMOUSE_ROTATE_V; // Rotation
        else                   action_pert = mjMOUSE_ZOOM;

        mjv_movePerturb(m, d, action_pert, dx/height, dy/height, &scn, &pert);
    } 
    // Standard Camera logic
    else {
        mjtMouse action_cam;
        if (button_right)      action_cam = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (button_left)  action_cam = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else                   action_cam = mjMOUSE_ZOOM;

        mjv_moveCamera(m, action_cam, dx/height, dy/height, &scn, &cam);
    }
}

void Viewer::onScroll(double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}