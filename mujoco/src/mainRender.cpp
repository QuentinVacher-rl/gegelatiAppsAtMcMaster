

#include <math.h>
#include <iostream>
#include <numeric>
#include <thread>
#include <atomic>
#include <chrono>
#include <inttypes.h>
#include <getopt.h>

#include "mainRender.h"
#include <glfw3.h>

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void InitVisualization(mjModel* task_m, mjData* task_d) {
    m = task_m;
    d = task_d;

    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    // GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    std::cout << "WINDOW1:";
    std::cout << glfwWindowShouldClose(window) << std::endl;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    std::cout << "WINDOW2:" << glfwWindowShouldClose(window) << std::endl;
}

void StepVisualization() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

int main(int argc, char ** argv) {
    
    char option;
	char dotPath[150];
    char paramFile[150];
    uint64_t seed=0;
    
    strcpy(dotPath, "logs/out_best.0.p0.v1.c0.dot");
    strcpy(paramFile, "params/params_0.json");
    while((option = getopt(argc, argv, "s:p:d:")) != -1){
        switch (option) {
            case 's': seed= atoi(optarg); break;
            case 'p': strcpy(paramFile, optarg); break;
            case 'd': strcpy(dotPath, optarg); break;
            default: std::cout << "Unrecognised option. Valid options are \'-s seed\' \'-p paramFile.json\' \'-d dot path\'." << std::endl; exit(1);
        }
    }

	std::cout << "Start Mujoco Rendering application." << std::endl;
    // Create the instruction set for programs
	Instructions::Set set;
	fillInstructionSet(set);

	// Set the parameters for the learning process.
	// (Controls mutations probability, program lengths, and graph size
	// among other things)
	// Loads them from the file params.json
	Learn::LearningParameters params;
	File::ParametersParser::loadParametersFromJson(paramFile, params);

	// Instantiate the LearningEnvironment
	MujocoAntWrapper mujocoAntLE(std::string("none"));

	// Instantiate and init the learning agent
	Learn::LearningAgent la(mujocoAntLE, set, params);
	la.init(seed);

    auto &tpg = *la.getTPGGraph();
    Environment env(set, mujocoAntLE.getDataSources(), params.nbRegisters, params.nbProgramConstant, params.useMemoryRegisters);
    File::TPGGraphDotImporter dotImporter(dotPath, env, tpg);
    dotImporter.importGraph();

    TPG::TPGExecutionEngine tee(env, NULL, false, 8);

    mujocoAntLE.reset(seed, Learn::LearningMode::TESTING);

    InitVisualization(mujocoAntLE.m_, mujocoAntLE.d_);
    StepVisualization();


    uint64_t nbActions = 0;
    while (!mujocoAntLE.isTerminal() && nbActions < params.maxNbActionsPerEval) {
        // Get the actions
        std::vector<double> actionsID =
            tee.executeFromRoot(*tpg.getRootVertices()[0], mujocoAntLE.getInitActions(),
                                1,
                                mujocoAntLE.getActivationFunction()).second;
        // Do it
        mujocoAntLE.doActions(actionsID);
        // Count actions
        nbActions++;
        StepVisualization();

    }

    // cleanup
	for (unsigned int i = 0; i < set.getNbInstructions(); i++) {
		delete (&set.getInstruction(i));
	}



}
