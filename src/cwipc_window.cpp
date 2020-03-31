#include <thread>
#include <chrono>
#include <condition_variable>

#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "window_util.hpp"

class cwipc_sink_window_impl : public cwipc_sink {
private:
    std::string m_title;
    window_util *m_window;
    cwipc_point *m_points;
    int m_npoints;
    float m_pointsize;
    float m_eye_angle;
    float m_eye_distance;
    float m_eye_height;
    bool m_mouse_pressed;
    float m_mouse_x;
    float m_mouse_y;
    std::string m_chars_wanted;
    char m_last_char;
    std::condition_variable m_last_char_cv;
    std::mutex m_last_char_mutex;
public:
    cwipc_sink_window_impl(const char *title)
    :   m_title(title),
        m_window(nullptr),
        m_points(nullptr),
        m_npoints(0),
        m_pointsize(0),
        m_eye_angle(0), m_eye_distance(1.8), m_eye_height(2),
        m_mouse_pressed(false),
        m_mouse_x(-1),
        m_mouse_y(-1),
        m_chars_wanted(""),
        m_last_char('\0')
    {
        m_window = new window_util(640, 480, m_title.c_str());
        m_window->on_left_mouse = std::bind(&cwipc_sink_window_impl::on_left_mouse, this, std::placeholders::_1);
        m_window->on_right_mouse = std::bind(&cwipc_sink_window_impl::on_right_mouse, this, std::placeholders::_1);
        m_window->on_mouse_move = std::bind(&cwipc_sink_window_impl::on_mouse_move, this, std::placeholders::_1, std::placeholders::_2);
        m_window->on_mouse_scroll = std::bind(&cwipc_sink_window_impl::on_mouse_scroll, this, std::placeholders::_1, std::placeholders::_2);
        m_window->on_character = std::bind(&cwipc_sink_window_impl::on_character, this, std::placeholders::_1);
    }
    
    ~cwipc_sink_window_impl() {
        delete m_window;
    }
    
    void free() {
        delete m_window;
        ::free(m_points);
        m_points = nullptr;
        m_npoints = 0;
    }
    
    bool feed(cwipc *pc, bool clear) {
        if (m_window == nullptr) return false;
        if (clear) {
            ::free(m_points);
            m_points = nullptr;
            m_npoints = 0;
        }
        if (pc) {
            size_t bufferSize = pc->get_uncompressed_size();
            cwipc_point *newBuffer = (cwipc_point *)realloc(m_points, m_npoints*sizeof(cwipc_point)+bufferSize);
            if (newBuffer == nullptr) return false;
            m_points = newBuffer;
            newBuffer = m_points + m_npoints;
            int nNewPoints = pc->copy_uncompressed(newBuffer, bufferSize);
            m_npoints += nNewPoints;
            m_pointsize = pc->cellsize();
        }
        m_window->prepare_gl(m_eye_distance*sin(m_eye_angle), m_eye_height, m_eye_distance*cos(m_eye_angle), m_pointsize);
        for (int i=0; i<m_npoints; i++) {
            glColor3ub(m_points[i].r, m_points[i].g, m_points[i].b);
            glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
        }
        m_window->cleanup_gl();
        if (!*m_window) return false;
        return true;
    }
    
    bool caption(const char *caption) {
        if (m_window == nullptr) return false;
        GLFWwindow *glfwWin = *m_window;
        std::string newTitle;
        if (caption) {
            newTitle = m_title + " - " + std::string(caption);
        } else {
            newTitle = m_title;
        }
        glfwSetWindowTitle(glfwWin, newTitle.c_str());
        return true;
    }
    
    char interact(const char *prompt, const char *responses, int32_t millis) {
        if (m_window == nullptr) return '\0';
        caption(prompt);
        m_chars_wanted = responses;
        std::chrono::system_clock::time_point until = std::chrono::system_clock::now();
        if (millis >= 0) {
            until += std::chrono::milliseconds(millis);
        } else {
            until += std::chrono::hours(24);
        }
        while (std::chrono::system_clock::now() < until && m_chars_wanted.find(m_last_char) == std::string::npos) {
            std::unique_lock<std::mutex> lock(m_last_char_mutex);
            m_last_char_cv.wait_for(lock, std::chrono::milliseconds(1), [this]{return this->m_last_char; });
            if (m_last_char) break;
            feed(nullptr, false);
        }
        char rv = m_last_char;
        m_last_char = '\0';
        return rv;
    }

    
private:
    void on_left_mouse(bool pressed) {
        m_mouse_pressed = pressed;
    }
    
    void on_right_mouse(bool pressed) {
    }
    
    void on_mouse_scroll(double deltax, double deltay) {
        m_eye_height += deltay / 10;
    }
    
    void on_mouse_move(double x, double y) {
        if (m_mouse_pressed) {
            float delta_x = x - m_mouse_x;
            float delta_y = y - m_mouse_y;
            m_eye_angle += delta_x / 100;
            m_eye_distance += delta_y / 100;
        }
        m_mouse_x = x;
        m_mouse_y = y;
    }
    
    void on_character(int c) {
        if (m_chars_wanted.find(c) == std::string::npos) return;
        m_last_char = c;
        m_last_char_cv.notify_one();
    }
    

    
};

cwipc_sink *
cwipc_window(const char *title, char **errorMessage, uint64_t apiVersion)
{
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            *errorMessage = (char *)"cwipc_window: incorrect apiVersion";
        }
        return NULL;
    }
    return new cwipc_sink_window_impl(title);
}

