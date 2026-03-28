#!/usr/bin/env python3
"""
Studio 16: Robot Integration
net_utils.py - Network utilities for the second terminal relay.
"""

import select as _select
import socket
import struct

_LEN_FMT = '>I'
_LEN_SIZE = struct.calcsize(_LEN_FMT)


def _sendFramed(sock, data: bytes) -> bool:
    try:
        header = struct.pack(_LEN_FMT, len(data))
        sock.sendall(header + data)
        return True
    except (OSError, BrokenPipeError) as err:
        print(f'[net_utils] send error: {err}')
        return False


def _recvExact(sock, n: int):
    buf = b''
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (OSError, ConnectionResetError) as err:
            print(f'[net_utils] recv error: {err}')
            return None
        if not chunk:
            return None
        buf += chunk
    return buf


def _recvFramed(sock):
    header = _recvExact(sock, _LEN_SIZE)
    if header is None:
        return None
    length = struct.unpack(_LEN_FMT, header)[0]
    if length == 0:
        return b''
    return _recvExact(sock, length)


def sendTPacketFrame(sock, frame: bytes) -> bool:
    return _sendFramed(sock, frame)


def recvTPacketFrame(sock):
    return _recvFramed(sock)


class TCPServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 65432, ssl_context=None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self._server_sock = None
        self.conn = None

    def start(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            self._server_sock = s
            print(f'[TCPServer] Listening on {self.host}:{self.port}')
            return True
        except OSError as err:
            print(f'[TCPServer] Could not bind to {self.host}:{self.port}: {err}')
            return False

    def accept(self, timeout: float = 5.0):
        if not self._server_sock:
            print('[TCPServer] Server not started. Call start() first.')
            return None
        self._server_sock.settimeout(timeout)
        try:
            conn, addr = self._server_sock.accept()
            if self.ssl_context:
                conn = self.ssl_context.wrap_socket(conn, server_side=True)
            conn.setblocking(False)
            self.conn = conn
            print(f'[TCPServer] Client connected from {addr}')
            return conn
        except socket.timeout:
            return None
        except OSError as err:
            print(f'[TCPServer] Accept error: {err}')
            return None

    def hasData(self) -> bool:
        if not self.conn:
            return False
        r, _, _ = _select.select([self.conn], [], [], 0)
        return bool(r)

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except OSError:
                pass
            self.conn = None
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None


class TCPClient:
    def __init__(self, host: str = 'localhost', port: int = 65432,
                 ssl_context=None, server_hostname: str = None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.server_hostname = server_hostname
        self.sock = None

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((self.host, self.port))
            if self.ssl_context:
                s = self.ssl_context.wrap_socket(
                    s,
                    server_hostname=self.server_hostname or self.host,
                )
            s.setblocking(False)
            self.sock = s
            print(f'[TCPClient] Connected to {self.host}:{self.port}')
            return True
        except (OSError, socket.timeout) as err:
            print(f'[TCPClient] Connection to {self.host}:{self.port} failed: {err}')
            return False

    def hasData(self) -> bool:
        if not self.sock:
            return False
        r, _, _ = _select.select([self.sock], [], [], 0)
        return bool(r)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
