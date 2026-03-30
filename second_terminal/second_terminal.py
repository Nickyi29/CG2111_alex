#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py - Second operator terminal.

Updated to match the fixed pi_sensor.py constants and packet format.
"""

import select
import struct
import sys
import time
import ssl

from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------

PI_HOST = 'localhost'
PI_PORT = 65432

TLS_ENABLED = True
TLS_CERT_PATH = 'certs/server.crt'

def _make_client_ssl_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(TLS_CERT_PATH)
    ctx.check_hostname = False
    return ctx

# ---------------------------------------------------------------------------
# TPacket constants
# ---------------------------------------------------------------------------

PACKET_TYPE_COMMAND = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE = 2

COMMAND_ESTOP = 0
COMMAND_COLOR = 1
COMMAND_FORWARD = 2
COMMAND_BACKWARD = 3
COMMAND_LEFT = 4
COMMAND_RIGHT = 5
COMMAND_SPEED = 6

RESP_OK = 0
RESP_STATUS = 1
RESP_COLOR = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)
TPACKET_FMT = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1


def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = []
    params = list(params) + [0] * (PARAMS_COUNT - len(params))
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(
        TPACKET_FMT,
        packetType,
        command,
        data_padded,
        *params[:PARAMS_COUNT]
    )
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None

    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None

    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command': fields[1],
        'data': fields[2],
        'params': list(fields[3:]),
    }


_estop_active = False


def _printPacket(pkt):
    global _estop_active

    ptype = pkt['packetType']
    cmd = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            new_speed = pkt['params'][0]
            pct = round(new_speed / 255 * 100)
            print(f"[robot] Speed updated -> {new_speed}/255 ({pct}%)")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")

        elif cmd == RESP_COLOR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"[robot] Color: R={r} Hz, G={g} Hz, B={b} Hz")

        else:
            print(f"[robot] Response: unknown command {cmd}")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


def _handleInput(line: str, client: TCPClient):
    line = line.strip().lower()
    if not line:
        return

    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: E-STOP')

    elif line == 'q':
        print('[second_terminal] Quitting.')
        raise KeyboardInterrupt

    else:
        print(f"[second_terminal] Unknown: '{line}'. Valid: e (E-Stop) q (quit)")


def run():
    ssl_ctx = _make_client_ssl_context() if TLS_ENABLED else None                                   #sets up TLS handshake b/w first and second device
    client = TCPClient(host=PI_HOST, port=PI_PORT, ssl_context=ssl_ctx, server_hostname=PI_HOST)
    print(f'[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...')

    if not client.connect(timeout=10.0):
        print('[second_terminal] Could not connect.')
        print(' Make sure pi_sensor.py is running and waiting for a second terminal connection.')
        sys.exit(1)

    print('[second_terminal] Connected!')
    print('[second_terminal] Commands: e = E-Stop q = quit')
    print('[second_terminal] Incoming robot packets will be printed below.\n')

    try:
        while True:
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print('[second_terminal] Connection to pi_sensor.py closed.')
                    break

                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print('\n[second_terminal] Exiting.')
    finally:
        client.close()


if __name__ == '__main__':
    run()
