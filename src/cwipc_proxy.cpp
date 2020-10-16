#include <chrono>
#include <thread>
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

class cwipc_source_proxy_impl : public cwipc_tiledsource {
private:
    int m_listen_socket;
    int m_socket;
public:
    cwipc_source_proxy_impl(int _socket)
    :   m_listen_socket(_socket),
        m_socket(-1)
    {
        
    }

    ~cwipc_source_proxy_impl() {
    	free();
    }

    void free() {
        // xxxjack stop thread
        if (m_listen_socket >= 0) closesocket(m_listen_socket);
        m_listen_socket = -1;
        if (m_socket >= 0) closesocket(m_socket);
        m_socket = -1;
    }
    
    bool eof() {
    	return m_listen_socket < 0 && m_socket < 0;
    }
    
    bool available(bool wait) {
    	return false;
    }

    cwipc* get() {
        cwipc *rv = NULL;
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
	return new cwipc_source_proxy_impl(sock);
}
