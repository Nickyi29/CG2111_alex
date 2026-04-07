#!/usr/bin/env python3
"""
relay.py — Second terminal relay.
FIX: checkSecondTerminal() was checking _st_server.hasData() instead of
     _st_conn for incoming data. _st_server only has data when a NEW client
     is connecting — after connection all data comes through _st_conn.
     This meant arm commands from Terminal 2 were never forwarded to Arduino.
"""

import select
import ssl
from .net_utils import TCPServer, sendTPacketFrame, recvTPacketFrame

SECOND_TERM_PORT    = 65432
SECOND_TERM_TIMEOUT = 300

TLS_ENABLED   = False
TLS_CERT_PATH = 'certs/server.crt'
TLS_KEY_PATH  = 'certs/server.key'

_st_server = None
_st_conn   = None


def _make_server_ssl_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(TLS_CERT_PATH, TLS_KEY_PATH)
    return ctx


def onPacketReceived(raw_frame: bytes):
    global _st_conn
    if _st_conn is not None:
        ok = sendTPacketFrame(_st_conn, raw_frame)
        if not ok:
            print('[relay] Second terminal disconnected (send failed).')
            _st_conn = None


def checkSecondTerminal(serial_port):
    global _st_conn
    if not _st_conn:
        return

    # Check the client connection socket for incoming data
    try:
        r, _, _ = select.select([_st_conn], [], [], 0)
    except Exception:
        print('[relay] Second terminal disconnected (select error).')
        _st_conn = None
        return

    if not r:
        return

    frame = recvTPacketFrame(_st_conn)
    if frame is not None:
        serial_port.write(frame)
        serial_port.flush()
    else:
        print('[relay] Second terminal disconnected.')
        _st_conn = None


def start():
    global _st_server, _st_conn
    ssl_context = _make_server_ssl_context() if TLS_ENABLED else None
    _st_server  = TCPServer(port=SECOND_TERM_PORT, ssl_context=ssl_context)
    if _st_server.start():
        print('[relay] Waiting for second_terminal.py to connect '
              '(open a new terminal: python3 second_terminal/second_terminal.py)...')
        _st_conn = _st_server.accept(timeout=SECOND_TERM_TIMEOUT)
        if _st_conn is None:
            print(f'[relay] No second terminal connected within {SECOND_TERM_TIMEOUT}s.'
                  ' Continuing without it.')


def shutdown():
    global _st_server
    if _st_server:
        _st_server.close()
        _st_server = None
    print('[relay] Shutdown complete.')
