#include <chrono>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <string.h>

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netdb.h>
#define closesocket(x) close(x)
#endif

#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"

#ifdef _WIN32
#include <Windows.h>
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {
    HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
    SetThreadDescription(threadHandle, name);
}
#else
inline void _cwpic_setThreadName(std::thread* thr, const wchar_t* name) {}
#endif
class cwipc_source_proxy_impl : public cwipc_tiledsource {
private:
    int m_listen_socket;
    int m_socket;
    std::thread *m_server_thread;
    bool m_running;
    std::mutex m_pc_mutex;
    std::condition_variable m_pc_fresh;
    cwipc *m_pc;
public:
    cwipc_source_proxy_impl(int _socket)
    :   m_listen_socket(_socket),
        m_socket(-1)
    {
        m_running = true;
        m_server_thread = new std::thread(&cwipc_source_proxy_impl::_server_main, this);
        _cwipc_setThreadName(m_server_thread, L"cwipc_proxy::server_thread");
    }

    ~cwipc_source_proxy_impl() {
    	free();
    }

    void free() {
        m_running = false;
        if (m_listen_socket >= 0) closesocket(m_listen_socket);
        m_listen_socket = -1;
        if (m_socket >= 0) closesocket(m_socket);
        m_socket = -1;
        m_server_thread->join();
    }
    
    void _server_main() {
        m_socket = accept(m_listen_socket, NULL, 0);
        closesocket(m_listen_socket);
        m_listen_socket = -1;
        if (m_socket < 0) {
            perror("cwipc_proxy: accept");
            closesocket(m_listen_socket);
            m_running = false;
            return;
        }
        while(m_running) {
            //
            // Get the header and check it
            //
            struct cwipc_point_packetheader header;
            if (!_recvall((void *)&header, sizeof(header))) break;
            if (header.magic != CWIPC_POINT_PACKETHEADER_MAGIC) {
                std::cerr << "cwpic_proxy: bad magic number: " << header.magic << std::endl;
                break;
            }
            //
            // Get the pointcloud data and convert it
            //
            cwipc_point* points = (cwipc_point *)malloc(header.dataCount);
            if (points == NULL) {
                std::cerr << "cwpic_proxy: malloc failed" << std::endl;
                break;
            }
            if (!_recvall((void *)points, header.dataCount)) break;
            char *errorMessage = NULL;
            cwipc *pc = cwipc_from_points(points, header.dataCount, header.dataCount/sizeof(cwipc_point), header.timestamp, &errorMessage, CWIPC_API_VERSION);
            ::free(points);
            if (pc == NULL) {
                std::cerr << "cwipc_proxy: cwipc_from_points: " << errorMessage << std::endl;
                break;
            }
            pc->_set_cellsize(header.cellsize);
            //
            // Forward the pointcloud to the consumer
            //
            std::unique_lock<std::mutex> mylock(m_pc_mutex);
            if (m_pc) {
                m_pc->free();
                m_pc = NULL;
            }
            m_pc = pc;
            m_pc_fresh.notify_all();
            //
            // Send acknowledgement (if connection still open)
            //
            if (send(m_socket, (const char *)&header.timestamp, sizeof(header.timestamp), 0) < 0) {
                if (m_socket >= 0) perror("cwipc_proxy: send");
                break;
            }
        }
        if (m_socket >= 0) {
            closesocket(m_socket);
            m_socket = -1;
        }
        std::unique_lock<std::mutex> mylock(m_pc_mutex);
        m_running = false;
        m_pc_fresh.notify_all();
    }
    
    bool _recvall(void *buffer, size_t size) {
        int status = recv(m_socket, (char *)buffer, size, MSG_WAITALL);
        if (status < 0) {
            perror("cwipc_proxy: recv");
        }
        return status == size;
    }
    
    bool eof() {
    	return m_listen_socket < 0 && m_socket < 0;
    }
    
    bool available(bool wait) {
        std::unique_lock<std::mutex> mylock(m_pc_mutex);
        if (wait) {
            m_pc_fresh.wait(mylock, [this]{return m_pc != NULL || !m_running; });
        }
    	return m_pc != NULL;
    }

    cwipc* get() {
        std::unique_lock<std::mutex> mylock(m_pc_mutex);
        m_pc_fresh.wait(mylock, [this]{return m_pc != NULL || !m_running; });
        cwipc *rv = m_pc;
        m_pc = NULL;
        return rv;
    }

	int maxtile() { return 1; }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
        static cwipc_tileinfo proxyInfo = {{0, 0, 0}, (char *)"proxy", 1};
		switch(tilenum) {
		case 0:
			if (tileinfo) *tileinfo = proxyInfo;
			return true;
		}
		return false;
	}

};

cwipc_tiledsource *
cwipc_proxy(const char *host, int port, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_proxy: incorrect apiVersion";
		}
		return NULL;
	}
    struct addrinfo hints;
    struct addrinfo *result = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = PF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;
    char portbuf[32];
    snprintf(portbuf, 32, "%d", port);
    int status = getaddrinfo(host, portbuf, &hints, &result);
    if (status != 0) {
        *errorMessage = (char *)gai_strerror(status);
        return NULL;
    }
    int sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock < 0) {
        *errorMessage = strerror(errno);
        return NULL;
    }
    status = bind(sock, result->ai_addr, result->ai_addrlen);
    if (status < 0) {
        *errorMessage = strerror(errno);
        closesocket(sock);
        return NULL;
    }
    status = listen(sock, 1);
    if (status < 0) {
        *errorMessage = strerror(errno);
        closesocket(sock);
        return NULL;
    }
	return new cwipc_source_proxy_impl(sock);
}
