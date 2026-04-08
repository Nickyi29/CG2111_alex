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

_estop_active   = False
_waiting_for_ack = False


# ---------------------------------------------------------------------------
# Help text
# ---------------------------------------------------------------------------

def _printHelp():
    print("")
    print("========================================")
    print("  ARM CONTROLS")
    print("========================================")
    print("  h         Home (all joints neutral)")
    print("")
    print("  BASE (rotate left/right):")
    print("  j         Base LEFT  -> 60 deg")
    print("  r         Base RIGHT -> 120 deg")
    print("")
    print("  SHOULDER (lean forward/back):")
    print("  i         Shoulder FORWARD -> 150 deg  (slow)")
    print("  k         Shoulder BACK    -> 100 deg  (slow)")
    print("  y         Shoulder MAX FWD -> 170 deg  (slow)")
    print("  t         Shoulder MAX BACK ->  60 deg (slow)")
    print("")
    print("  ELBOW (raise/lower forearm):")
    print("  u         Elbow UP   -> 130 deg")
    print("  o         Elbow DOWN ->  50 deg")
    print("  p         Elbow HIGH -> 150 deg")
    print("  l         Elbow LOW  ->  20 deg")
    print("")
    print("  GRIPPER (open/close):")
    print("  n         Gripper OPEN  -> 60 deg")
    print("  m         Gripper CLOSE -> 10 deg")
    print("")
    print("  PICK-UP SEQUENCE:")
    print("  1         Lean arm forward (shoulder 150)")
    print("  2         Lower elbow to floor (elbow 20)")
    print("  3         Lift elbow up (elbow 120)")
    print("  4         Carry position (shoulder 100)")
    print("")
    print("  EMERGENCY:")
    print("  e         E-Stop (toggle)")
    print("  q         Quit")
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

def _handleInput(line: str, client: TCPClient):
    global _waiting_for_ack

    # preserve case — do not call .lower() so uppercase keys work
    line = line.strip()
    if not line:
        return

    arm_keys = {
        'h', 'j', 'r',
        'i', 'k', 'y', 't',
        'u', 'o', 'p', 'l',
        'n', 'm',
        '1', '2', '3', '4'
    }

    # E-Stop — always allowed regardless of ack state
    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: E-STOP')
        _waiting_for_ack = False
        return

    if line == 'q':
        print('[second_terminal] Quitting.')
        raise KeyboardInterrupt

    # Gate: block new arm command until previous ACK received
    if line in arm_keys and _waiting_for_ack:
        print('[second_terminal] Still moving — wait for acceptance...')
        return

    # ---- HOME ----
    if line == 'h':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_HOME)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ARM HOME')
        _waiting_for_ack = True

    # ---- BASE ----
    elif line == 'j':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[45])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE LEFT -> 60 deg')
        _waiting_for_ack = True

    elif line == 'r':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[135])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE RIGHT -> 120 deg')
        _waiting_for_ack = True

    # ---- SHOULDER ----
    elif line == 'i':
        # Slow down for shoulder — fighting gravity
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[150])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER FORWARD -> 150 deg (slow)')
        _waiting_for_ack = True

    elif line == 'k':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[100])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER BACK -> 100 deg (slow)')
        _waiting_for_ack = True

    elif line == 'y':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[170])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER MAX FORWARD -> 170 deg (slow)')
        _waiting_for_ack = True

    elif line == 't':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[60])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER MAX BACK -> 60 deg (slow)')
        _waiting_for_ack = True

    # ---- ELBOW ----
    elif line == 'u':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[130])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW UP -> 130 deg')
        _waiting_for_ack = True

    elif line == 'o':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[50])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW DOWN -> 50 deg')
        _waiting_for_ack = True

    elif line == 'p':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[150])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW HIGH -> 150 deg')
        _waiting_for_ack = True

    elif line == 'l':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[20])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW LOW -> 20 deg')
        _waiting_for_ack = True

    # ---- GRIPPER ----
    elif line == 'n':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_GRIPPER, params=[50])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: GRIPPER OPEN -> 40 deg')
        _waiting_for_ack = True

    elif line == 'm':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_GRIPPER, params=[125])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: GRIPPER CLOSE -> 80 deg')
        _waiting_for_ack = True

    # ---- PICK-UP SEQUENCES ----
    elif line == '1':
        # Step 1: Lean shoulder forward toward object
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[150])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: REACH FORWARD — shoulder 150 deg')
        _waiting_for_ack = True

    elif line == '2':
        # Step 2: Lower elbow to floor level
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[20])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: LOWER TO FLOOR — elbow 20 deg')
        _waiting_for_ack = True

    elif line == '3':
        # Step 3: Lift elbow back up after gripping
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[120])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: LIFT — elbow 120 deg')
        _waiting_for_ack = True

    elif line == '4':
        # Step 4: Pull shoulder back to carry position
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=[5])
        sendTPacketFrame(client.sock, frame)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[100])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: CARRY — shoulder 100 deg')
        _waiting_for_ack = True

    else:
        print(f"[second_terminal] Unknown key: '{line}'")
        _printHelp()


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
