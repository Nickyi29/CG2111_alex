#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py - Second operator terminal.
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

# Path relative to this script file — works regardless of run directory
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

_waiting_for_ack = False



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


_estop_active = False


def _printPacket(pkt):
    global _estop_active, _waiting_for_ack

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            debug_str = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
            val = pkt['params'][0]
            if debug_str in ('BASE', 'SHOULDER', 'ELBOW', 'GRIPPER', 'HOME'):
                _waiting_for_ack = False   # clear gate on ACK
                print(f"[robot] Arm accepted: {debug_str} -> {val} deg")
            elif val > 0:
                pct = round(val / 255 * 100)
                print(f"[robot] Speed updated -> {val}/255 ({pct}%)")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            if _estop_active:
                _waiting_for_ack = False  # clear gate on estop
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
# Controls help text
# ---------------------------------------------------------------------------

def _printHelp():
    print("")
    print("========================================")
    print("  ARM CONTROLS")
    print("========================================")
    print("  h         Home (safe neutral position)")
    print("")
    print("  BASE (rotate left/right):")
    print("  j         Base LEFT  -> 60 deg")
    print("  r         Base RIGHT -> 120 deg")
    print("  J         Base FAR LEFT  -> 45 deg")
    print("  R         Base FAR RIGHT -> 135 deg")
    print("")
    print("  SHOULDER (raise/lower upper arm):")
    print("  i         Shoulder UP   -> 130 deg")
    print("  k         Shoulder DOWN -> 50 deg")
    print("  I         Shoulder HIGH -> 150 deg")
    print("  K         Shoulder LOW  -> 30 deg")
    print("")
    print("  ELBOW (raise/lower forearm):")
    print("  u         Elbow UP   -> 130 deg")
    print("  o         Elbow DOWN -> 50 deg")
    print("  U         Elbow HIGH -> 150 deg")
    print("  O         Elbow LOW  -> 20 deg")
    print("")
    print("  GRIPPER (open/close claw):")
    print("  n         Gripper OPEN  -> 90 deg")
    print("  m         Gripper CLOSE -> 10 deg")
    print("")
    print("  SEQUENCES (one key pick-up helpers):")
    print("  1         Reach down to floor")
    print("  2         Grip and lift")
    print("  3         Carry position")
    print("")
    print("  EMERGENCY:")
    print("  e         E-Stop")
    print("  q         Quit")
    print("========================================")
    print("")

def _handleInput(line: str, client: TCPClient):
    global _waiting_for_ack
    line = line.strip()  # preserve case for uppercase variants
    if not line:
        return

    arm_keys = {
        'h','j','r','J','R',
        'i','k','I','K',
        'u','o','U','O',
        'n','m',
        '1','2','3'
    }

    if line.lower() == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: E-STOP')
        _waiting_for_ack = False  # always allow after estop
        return

    if line.lower() == 'q':
        print('[second_terminal] Quitting.')
        raise KeyboardInterrupt

    # Gate: block new arm command until previous ACK received
    if line in arm_keys and _waiting_for_ack:
        print('[second_terminal] Still moving — wait for acceptance...')
        return

    # ---- BASE ----
    if line == 'j':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[60])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE LEFT -> 60 deg')
        _waiting_for_ack = True

    elif line == 'r':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[120])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE RIGHT -> 120 deg')
        _waiting_for_ack = True

    elif line == 'J':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[45])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE FAR LEFT -> 45 deg')
        _waiting_for_ack = True

    elif line == 'R':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_BASE, params=[135])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: BASE FAR RIGHT -> 135 deg')
        _waiting_for_ack = True

    # ---- SHOULDER ----
elif line == 'i':
    # Slow down first, then move shoulder
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

    elif line == 'I':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[150])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER HIGH -> 150 deg')
        _waiting_for_ack = True

    elif line == 'K':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[30])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: SHOULDER LOW -> 30 deg')
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

    elif line == 'U':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[150])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW HIGH -> 150 deg')
        _waiting_for_ack = True

    elif line == 'O':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[20])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ELBOW LOW -> 20 deg')
        _waiting_for_ack = True

    # ---- GRIPPER ----
    elif line == 'n':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_GRIPPER, params=[90])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: GRIPPER OPEN -> 90 deg')
        _waiting_for_ack = True

    elif line == 'm':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_GRIPPER, params=[10])
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: GRIPPER CLOSE -> 10 deg')
        _waiting_for_ack = True

    # ---- HOME ----
    elif line == 'h':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_HOME)
        sendTPacketFrame(client.sock, frame)
        print('[second_terminal] Sent: ARM HOME')
        _waiting_for_ack = True

    # ---- SEQUENCES ----
    elif line == '1':
    # Step 1: Lean arm forward toward floor
    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[150])
    sendTPacketFrame(client.sock, frame)
    print('[second_terminal] Sent: REACH FORWARD — shoulder 150 deg')
    _waiting_for_ack = True

elif line == '2':
    # Step 2: Lower elbow to floor level
    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[20])
    sendTPacketFrame(client.sock, frame)
    print('[second_terminal] Sent: LOWER ELBOW -> 20 deg')
    _waiting_for_ack = True

elif line == '3':
    # Step 3: Lift — raise elbow back
    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_ELBOW, params=[120])
    sendTPacketFrame(client.sock, frame)
    print('[second_terminal] Sent: LIFT ELBOW -> 120 deg')
    _waiting_for_ack = True

elif line == '4':
    # Step 4: Carry — pull shoulder back
    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SHOULDER, params=[100])
    sendTPacketFrame(client.sock, frame)
    print('[second_terminal] Sent: CARRY — shoulder 100 deg')
    _waiting_for_ack = True


def run():
    ssl_ctx = _make_client_ssl_context() if TLS_ENABLED else None
    client = TCPClient(host=PI_HOST, port=PI_PORT,
                       ssl_context=ssl_ctx, server_hostname=PI_HOST)
    print(f'[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...')

    if not client.connect(timeout=10.0):
        print('[second_terminal] Could not connect.')
        print(' Make sure pi_sensor.py is running and waiting for a connection.')
        sys.exit(1)

    print('[second_terminal] Connected!')

    # Print full controls on startup
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

            time.sleep(0.02)

    except KeyboardInterrupt:
        print('\n[second_terminal] Exiting.')
    finally:
        client.close()


if __name__ == '__main__':
    run()
