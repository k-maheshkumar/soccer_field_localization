#include "main.h"
#include "controller.h"
#include "Localization.hpp"
#include "robot_defs.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

std::unique_ptr<soccer_field_localization::Localization> localization;
bool visualize{false};

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current
 * robot position estimate.
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
    localization->getState(estimatePosn);
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the
 * robot in local robot coordinates (observed by odometry model), which
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    localization->motionUpdate(delta);
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all
 * marker obervations (marker index and position of marker in robot
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    localization->sensorUpdate(observations);
}

/**
 * myinit()
 * Initialization function that takes as input the initial
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, FieldLocation markerLocations[NUM_LANDMARKS])
{
    std::array<FieldLocation, NUM_LANDMARKS> fieldLocations;
    for (size_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        fieldLocations[i] = markerLocations[i];
    }

    localization = std::make_unique<soccer_field_localization::Localization>(robotParams, fieldLocations);
    localization->init(robotState);
}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    if (!visualize)
    {
        return;
    }
    
    // Draw red colored points at specified global locations on field
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 1.0);
    const auto drawFunction{[&](const RobotState& state) {
        int pixelX, pixelY;
        global2pixel(state.x, state.y, pixelX, pixelY);
        glVertex2i(pixelX, pixelY);
    }};

    localization->visualize(drawFunction);

    glEnd();
}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
// int mykeyboard(unsigned char key)
int mykeyboard(unsigned char key)
{
    if (key == 'p' || key == 'P')
    {
        visualize = !visualize;
    }

    return 0;
}

/**
 * Main entrypoint for the program.
 */
int main(int argc, char** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);

    return 0;
}
