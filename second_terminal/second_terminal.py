#!/usr/bin/env python3
"""
second_terminal.py
CG2111A — Alex Robot
Second operator terminal — arm control and robot status.
"""

import os
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

TLS_ENABLED = False

_DIR = os.path.dirname(os.path.abspath(__file__))
TLS_CERT_PATH = os.path.join(_DIR, '..', 'certs', 'server.crt')


def _make_client_ssl_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(TLS_CERT_PATH)
    ctx.check_hostname = False
    return ctx


# ---------------------------------------------------------------------------
# TPacket constants
# ---------------------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP        = 0
COMMAND_COLOR        = 1
COMMAND_FORWARD      = 2
COMMAND_BACKWARD     = 3
COMMAND_LEFT         = 4
COMMAND_RIGHT        = 5
COMMAND_SPEED        = 6
COMMAND_ARM_BASE     = 7
COMMAND_ARM_SHOULDER = 8
COMMAND_ARM_ELBOW    = 9
COMMAND_ARM_GRIPPER  = 10
COMMAND_ARM_HOME     = 11
COMMAND_ARM_SPEED    = 12
COMMAND_STOP         = 13

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC      = b'\xDE\xAD'
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
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

_estop_active    = False
_waiting_for_ack = False


# ---------------------------------------------------------------------------
# Help text
# ---------------------------------------------------------------------------

def _printHelp():
    print("")
    print("========================================")
    print("  ARM CONTROLS — FREE ANGLE MODE")
    print("========================================")
    print("  Type: <joint> <degrees>")
    print("  b <deg>   Base      (30-150)")
    print("  s <deg>   Shoulder  (10-180)")
    print("  e <deg>   Elbow     (10-200)")
    print("  g <deg>   Gripper   (10-130)")
    print("")
    print("  Examples:")
    print("    b 90      base to 90 degrees")
    print("    s 150     shoulder to 150 degrees")
    print("    e 40      elbow to 40 degrees")
    print("    g 80      gripper close")
    print("    g 10      gripper open")
    print("")
    print("  PRESET KEYS:")
    print("  h         Home all joints")
    print("")
    print("  e         E-Stop (toggle)")
    print("  q         Quit")
    print("  ?         Show this help")
    print("========================================")
    print("")


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

def _printPacket(pkt):
    global _estop_active, _waiting_for_ack

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            debug_str = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
            val = pkt['params'][0]
            if debug_str in ('BASE', 'SHOULDER', 'ELBOW', 'GRIPPER', 'HOME'):
                _waiting_for_ack = False
                print(f"[robot] Arm accepted: {debug_str} -> {val} deg")
            elif debug_str in ('E-Stop ON', 'E-Stop OFF'):
                print(f"[robot] {debug_str}")
            elif val > 0:
                pct = round(val / 255 * 100)
                print(f"[robot] Speed updated -> {val}/255 ({pct}%)")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            if _estop_active:
                _waiting_for_ack = False
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")

        elif cmd == RESP_COLOR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"[robot] Color: R={r} Hz, G={g} Hz, B={b} Hz")

        else:
            print(f"[robot] Response: unknown command {cmd}")

        debug_str_raw = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug_str_raw and debug_str_raw not in (
                'BASE', 'SHOULDER', 'ELBOW', 'GRIPPER', 'HOME',
                'E-Stop ON', 'E-Stop OFF'):
            print(f"[robot] Debug: {debug_str_raw}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handler
# ---------------------------------------------------------------------------

def _sendArm(client, command, degrees, label):
    global _waiting_for_ack
    frame = _packFrame(PACKET_TYPE_COMMAND, command, params=[degrees])
    sendTPacketFrame(client.sock, frame)
    print(f'[second_terminal] Sent: {label} -> {degrees} deg')
    _waiting_for_ack = True


def _handleInput(line: str, client: TCPClient):
    global _waiting_for_ack

    line = line.strip()
    if not line:
        return

    # ---- Free angle commands: "b 90", "s 150", "e 40", "g 80" ----
    parts = line.split()
    if len(parts) == 2 and parts[0] in ('b', 's', 'e', 'g'):
        if _waiting_for_ack:
            print('[second_terminal] Still moving — wait for acceptance...')
            return
        try:
            deg = int(parts[1])
        except ValueError:
            print(f'[second_terminal] Invalid angle: {parts[1]} — must be a number')
            return

        joint = parts[0]
        if joint == 'b':
            if not (30 <= deg <= 150):
                print(f'[second_terminal] Base out of range (30-150): {deg}')
                return
            _sendArm(client, COMMAND_ARM_BASE, deg, 'BASE')

        elif joint == 's':
            if not (10 <= deg <= 180):
                print(f'[second_terminal] Shoulder out of range (10-180): {deg}')
                return
            # Slow speed for shoulder
            frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
            sendTPacketFrame(client.sock, frame)
            _sendArm(client, COMMAND_ARM_SHOULDER, deg, 'SHOULDER')

        elif joint == 'e':
            if not (10 <= deg <= 200):
                print(f'[second_terminal] Elbow out of range (10-200): {deg}')
                return
            _sendArm(client, COMMAND_ARM_ELBOW, deg, 'ELBOW')

        elif joint == 'g':
            if not (10 <= deg <= 130):
                print(f'[second_terminal] Gripper out of range (10-130): {deg}')
                return
            _sendArm(client, COMMAND_ARM_GRIPPER, deg, 'GRIPPER')
        return

    # ---- E-Stop — always allowed ----
    if line == 'e' and len(parts) == 1:
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: E-STOP')
        _waiting_for_ack = False
        return

    if line == 'q':
        print('[second_terminal] Quitting.')
        raise KeyboardInterrupt

    if line == '?':
        _printHelp()
        return

    # ---- Gate for preset keys ----
    preset_keys = {
        'h', 'j', 'r',
        'i', 'k', 'y', 't',
        'u', 'o', 'p', 'l',
        'n', 'm',
        '1', '2', '3', '4'
    }

    if line in preset_keys and _waiting_for_ack:
        print('[second_terminal] Still moving — wait for acceptance...')
        return

    # ---- HOME ----
    if line == 'h':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_HOME)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ARM HOME')
        _waiting_for_ack = True


    else:
        print(f"[second_terminal] Unknown command: '{line}' — type ? for help")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run():
    ssl_ctx = _make_client_ssl_context() if TLS_ENABLED else None
    client  = TCPClient(host=PI_HOST, port=PI_PORT,
                        ssl_context=ssl_ctx, server_hostname=PI_HOST)
    print(f'[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...')

    if not client.connect(timeout=10.0):
        print('[second_terminal] Could not connect.')
        print(' Make sure pi_sensor.py is running and waiting for a connection.')
        sys.exit(1)

    print('[second_terminal] Connected!')
    _printHelp()
    print('[second_terminal] Incoming robot status will be printed below.')
    print('[second_terminal] The arm will NOT move while E-Stop is active.\n')

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

            time.sleep(0.005)

    except KeyboardInterrupt:
        print('\n[second_terminal] Exiting.')
    finally:
        client.close()


if __name__ == '__main__':
    run()
