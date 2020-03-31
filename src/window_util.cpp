//
//  window_util.cpp
//
//  Created by Fons Kuijk on 14-02-19.
//

#include "window_util.hpp"

window_util::window_util(int width, int height, const char* title) : _width(width), _height(height)
{
	glfwInit();
	win = glfwCreateWindow(width, height, title, nullptr, nullptr);
	if (!win)
		throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
	glfwMakeContextCurrent(win);
	glfwSetWindowUserPointer(win, this);
	glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods) {
		auto s = (window_util*)glfwGetWindowUserPointer(win);
        if (button == GLFW_MOUSE_BUTTON_LEFT) s->on_left_mouse(action == GLFW_PRESS);
        if (button == GLFW_MOUSE_BUTTON_RIGHT) s->on_left_mouse(action == GLFW_PRESS);
	});

	glfwSetScrollCallback(win, [](GLFWwindow * win, double xoffset, double yoffset) {
		auto s = (window_util*)glfwGetWindowUserPointer(win);
		s->on_mouse_scroll(xoffset, yoffset);
	});

	glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y) {
		auto s = (window_util*)glfwGetWindowUserPointer(win);
		s->on_mouse_move(x, y);
	});

	glfwSetCharCallback(win, [](GLFWwindow * win, unsigned int key) {
		auto s = (window_util*)glfwGetWindowUserPointer(win);
        s->on_character(key);
	});
}

glfw_state* window_util::app_state() { return &_appstate; }
float window_util::width() const { return float(_width); }
float window_util::height() const { return float(_height); }

// OpenGL commands that prep screen
void window_util::prepare_gl(float x, float y, float z, float pointSize)
{
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(0.2, 0.4, 0.6, 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, (double(_width)) / (double(_height)), 0.01, 10.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(x, y, z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	//glTranslatef(0.0, 0.0, _appstate.offset*0.05f);
	//glRotated(_appstate.pitch, 1, 0, 0);
	//glRotated(_appstate.yaw, 0, 1, 0);
	//glTranslatef(x, y, z);
	glPushMatrix();
	if (pointSize > 0) {
		glPointSize(pointSize);
	} else {
		glPointSize(float(_width) / float(640));
	}
	glEnable(GL_DEPTH_TEST);
    // Draw floor
    glBegin(GL_LINES);
    glColor3f(0.5, 0.5, 0.5);
    for (float pos = -5; pos < 6; pos += 2) {
        glVertex3f(-5, 0, pos);
        glVertex3f(5, 0, pos);
        glVertex3f(pos, 0, -5);
        glVertex3f(pos, 0, 5);
    }
    glEnd();
    // Draw axes
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    glEnd();
    // Setup for drawing points
	glBegin(GL_POINTS);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

	// OpenGL cleanup
void window_util::cleanup_gl()
{
	glEnd();
	glPopMatrix();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
    glfwSwapBuffers(win);
	glPushMatrix();
}

window_util::~window_util()
{
	glfwDestroyWindow(win);
	glfwTerminate();
}

window_util::operator GLFWwindow*() { return win; }

window_util::operator bool()
{
	glPopMatrix();

	auto res = !glfwWindowShouldClose(win);

	glfwPollEvents();
	glfwGetFramebufferSize(win, &_width, &_height);

	// Clear the framebuffer
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, _width, _height);

	// Draw the images
	glPushMatrix();
	glfwGetWindowSize(win, &_width, &_height);
	glOrtho(0, _width, _height, 0, -1, +1);

	return res;
}
