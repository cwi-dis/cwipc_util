import sys
import socketserver
import argparse
import threading
import socket
import queue
from typing import List, cast, Tuple, Any

HEADER_SIZE = 128
HEADER_VERSION = 2
MAX_OUTPUT_QUEUE = 10


class ForwardHandler(socketserver.BaseRequestHandler):
    # transmit_thread : threading.Thread
    transmit_queue : "queue.Queue[bytes]"

    def handle(self):
        self.log_verbose("forwarder: Connected")
        self.transmit_queue = queue.Queue(MAX_OUTPUT_QUEUE)

        server : ForwardServer = cast(ForwardServer, self.server)
        server.handlers.append(self)

        # self.transmit_thread = threading.Thread(target=self.handle_transmit)
        # self.transmit_thread.start()
        self.handle_transmit()

    def log(self, msg : str):
        print(f"{sys.argv[0]}: ForwardHandler: {self.client_address}: {msg}", file=sys.stderr)

    def log_verbose(self, msg : str):
        server : ForwardServer = cast(ForwardServer, self.server)

        if server.verbose:
            self.log(msg)

    def stop(self):
        self.log_verbose("Disconnected")

        server : ForwardServer = cast(ForwardServer, self.server)
        server.handlers.remove(self)

        sock : socket.socket = self.request
        sock.close()

        self.request = None

    def transmitter_forward(self, header : bytes, payload : bytes):
        packet = header+payload

        try:
            self.transmit_queue.put(packet, block=False)
        except queue.Full:
            self.log(f"Dropped {len(packet)} byte packet")

    def handle_transmit(self):
        self.log_verbose("Transmitter started")

        while self.request != None:
            sock : socket.socket = self.request

            try:
                packet = self.transmit_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            sock.sendall(packet)
            self.log_verbose(f"Transmitted {len(packet)} byte packet")

        if self.request != None:
            sock : socket.socket = self.request
            sock.shutdown(socket.SHUT_WR)

        self.log_verbose("Transmitter stopped")

class ForwardServer(socketserver.ThreadingTCPServer):
    verbose : bool
    handlers : List[ForwardHandler]

    def __init__(self, *args : Any, **kwargs : Any):
        self.verbose = False
        self.handlers = []
        socketserver.ThreadingTCPServer.__init__(self, *args, **kwargs)


class IngestHandler(socketserver.BaseRequestHandler):

    def handle(self):
        self.log_verbose("Connected")
        self.transmit_queue = queue.Queue(MAX_OUTPUT_QUEUE)

        server : IngestServer = cast(IngestServer, self.server)
        server.handlers.append(self)

        self.handle_receive()

    def log(self, msg : str):
        print(f"{sys.argv[0]}: IngestHandler: {self.client_address}: {msg}", file=sys.stderr)

    def log_verbose(self, msg : str):
        server : ForwardServer = cast(ForwardServer, self.server)

        if server.verbose:
            self.log(msg)

    def stop(self):
        self.log_verbose("Disconnected")

        server : IngestServer = cast(IngestServer, self.server)
        server.handlers.remove(self)

            
        sock : socket.socket = self.request
        sock.close()

        self.request = None

        # Also close all outgoing connections
        forward_server = server.forwardServer
        for forwarder in forward_server.handlers:
            forwarder.stop()

    def receiver_forward(self, header : bytes, payload : bytes):
        server : IngestServer = cast(IngestServer, self.server)
        forward_server = server.forwardServer

        for forwarder in forward_server.handlers:

            forwarder.transmitter_forward(header, payload)
        else:
            self.log_verbose("Dropped packet")

    def handle_receive(self):
        self.log_verbose("Receiver started")

        while self.request != None:
            header = self._recv_all(HEADER_SIZE)

            if len(header) != HEADER_SIZE:
                self.log(f"Received short header, {len(header)} bytes: {header}")
                self.stop()
                break

            datalen, streamName = self._header_decode(header)

            if datalen < 0:
                self.stop()
                break

            payload = self._recv_all(datalen)

            if len(payload) != datalen:
                self.log(f"Received short payload, wanted {datalen} bytes, got {len(payload)}")
                self.stop()
                break

            packetLen = len(header) + len(payload)
            self.log_verbose(f"Received {packetLen} byte packet for stream {streamName}")
            self.receiver_forward(header, payload)

        # xxxjack don't understand this?
        if self.request != None:
            sock : socket.socket = self.request
            sock.shutdown(socket.SHUT_RD)

        self.log_verbose("Receiver stopped")

    def _recv_all(self, datalen : int) -> bytes:
        sock : socket.socket = self.request
        rv = b''

        while True:
            next = sock.recv(datalen)
            datalen -= len(next)
            rv += next

            if len(next) == 0 or datalen == 0:
                return rv

    def _header_decode(self, header : bytes) -> Tuple[int, str]:
        header_fields = header.decode().split(',')

        if len(header_fields) != 5 or header_fields[0] != str(HEADER_VERSION):
            self.log(f"Received invalid header: {header}")
            return -1, ""

        streamName = header_fields[1]
        dataSize = int(header_fields[3])

        return dataSize, streamName

class IngestServer(socketserver.TCPServer):
    verbose : bool
    handlers : List[IngestHandler]
    forwardServer: ForwardServer

    def __init__(self, forwardServer : ForwardServer, *args : Any, **kwargs : Any):
        self.verbose = False
        self.forwardServer = forwardServer
        self.handlers = []
        socketserver.TCPServer.__init__(self, *args, **kwargs)

def main():
    global verbose

    parser = argparse.ArgumentParser(
        description="TCP reflection server"
    )

    parser.add_argument(
        "--port", action="store", type=int, default=4303,
        help="The fanout port to serve on (default: 4303, like cwipc_forward --port)"
    )
    parser.add_argument(
        "--ingestport", action="store", type=int, default=4304,
        help="The ingestion port to serve on (default: 4304)."
    )
    parser.add_argument(
        "--host", action="store", default="",
        help="IP address or hostname to serve on (default: all interfaces)"
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Print verbose messages"
    )
    parser.add_argument("--debugpy", action="store_true", help="Pause at begin of run to wait for debugpy attaching")

    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")        

    try:
        with ForwardServer((args.host, args.port), ForwardHandler) as forward_server:
            forward_server.verbose = args.verbose

            if args.verbose:
                print(
                    f"{sys.argv[0]}: forwarder serving on {forward_server.server_address}",
                    file=sys.stderr
                )

            forward_thread = threading.Thread(target=forward_server.serve_forever, daemon=True)
            forward_thread.start()
            with IngestServer(forward_server, (args.host, args.ingestport), IngestHandler) as ingest_server:
                ingest_server.verbose = args.verbose
                if args.verbose:
                    print(
                        f"{sys.argv[0]}: ingester serving on {ingest_server.server_address}",
                        file=sys.stderr
                    )
                ingest_server.serve_forever()

    finally:
        print(f"{sys.argv[0]}: Shutting down", file=sys.stderr)


if __name__ == "__main__":
    main()
