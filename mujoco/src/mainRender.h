// TODO(skelly): move visualization code the MujocoEnv.h
#ifndef RENDER_MUJOCO_h
#define RENDER_MUJOCO_h

#include <GLFW/glfw3.h>
#include <mujoco.h>


#include "mujocoAntWrapper.h"
#include <gegelati.h>
#include "instructions.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/******************************************************************************/
// MuJoCo data structures
mjModel* m = NULL;  // MuJoCo model
mjData* d = NULL;   // MuJoCo data
mjvCamera cam;      // abstract camera
mjvOption opt;      // visualization options
mjvScene scn;       // abstract scene
mjrContext con;     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
int frame_count = 0;

GLFWwindow* window = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods);

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos);

void InitVisualization(mjModel* task_m, mjData* task_d);

void saveFrame(int width, int height, const char* filename);

void StepVisualization(bool isSaveFrame, const char* filePath="");



#endif