/*
 * packets.h
 * CG2111A — Alex Robot
 */

#pragma once

#include <stdint.h>

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP        = 0,
    COMMAND_COLOR        = 1,
    COMMAND_FORWARD      = 2,
    COMMAND_BACKWARD     = 3,
    COMMAND_LEFT         = 4,
    COMMAND_RIGHT        = 5,
    COMMAND_SPEED        = 6,
    COMMAND_ARM_BASE     = 7,
    COMMAND_ARM_SHOULDER = 8,
    COMMAND_ARM_ELBOW    = 9,
    COMMAND_ARM_GRIPPER  = 10,
    COMMAND_ARM_HOME     = 11,
    COMMAND_ARM_SPEED    = 12,
    COMMAND_STOP         = 13,
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    RESP_COLOR  = 2,
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

#define MAGIC_HI     0xDE
#define MAGIC_LO     0xAD
#define TPACKET_SIZE ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE   (2 + TPACKET_SIZE + 1)       // 103 bytes
