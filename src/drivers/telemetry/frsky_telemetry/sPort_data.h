/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sPort_data.h
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky SmartPort telemetry implementation.
 *
 */
#ifndef _SPORT_DATA_H
#define _SPORT_DATA_H

#include <sys/types.h>
#include <stdbool.h>

/* FrSky SmartPort polling IDs captured from X4R */
#define SMARTPORT_POLL_BATV  0x1B
#define SMARTPORT_POLL_CUR   0x34
#define SMARTPORT_POLL_ALT   0x95
#define SMARTPORT_POLL_SPD   0x16
#define SMARTPORT_POLL_FUEL  0xB7
#define SMARTPORT_POLL_VSPD  0x00
#define SMARTPORT_POLL_GPS   0x83
#define SMARTPORT_POLL_NAV   0xBA
#define SMARTPORT_POLL_SP2UR 0xC6
#define SMARTPORT_POLL_ATT   0x53
#define SMARTPORT_POLL_HDIST 0x98

/* poll ids taken from yaapu: https://github.com/yaapu/FrskyTelemetryScript/wiki/FrSky-SPort-protocol-specs */
/*
#define SMARTPORT_POLL_VARIO       0x00 //Physical ID  1 - Vario2 (altimeter high precision)
#define SMARTPORT_POLL_FLVSS       0xA1 //Physical ID  2 - FLVSS Lipo sensor (can be sent with one or two cell voltages)
#define SMARTPORT_POLL_FAS         0x22 //Physical ID  3 - FAS-40S current sensor
#define SMARTPORT_POLL_GPS         0x83 //Physical ID  4 - GPS / altimeter (normal precision)
#define SMARTPORT_POLL_RPM         0xE4 //Physical ID  5 - RPM
#define SMARTPORT_POLL_SP2UH       0x45 //Physical ID  6 - SP2UART(Host)
#define SMARTPORT_POLL_SP2UR       0xC6 //Physical ID  7 - SPUART(Remote)
#define SMARTPORT_POLL_x67         0x67 //Physical ID  8 -
#define SMARTPORT_POLL_x48         0x48 //Physical ID  9 -
#define SMARTPORT_POLL_xE9         0xE9 //Physical ID 10 -
#define SMARTPORT_POLL_x6A         0x6A //Physical ID 11 -
#define SMARTPORT_POLL_xCB         0xCB //Physical ID 12 -
#define SMARTPORT_POLL_xAC         0xAC //Physical ID 13 -
#define SMARTPORT_POLL_x0D         0x0D //Physical ID 14 -
#define SMARTPORT_POLL_x8E         0x8E //Physical ID 15 -
#define SMARTPORT_POLL_x2F         0x2F //Physical ID 16 -
#define SMARTPORT_POLL_xD0         0xD0 //Physical ID 17 -
#define SMARTPORT_POLL_x71         0x71 //Physical ID 18 -
#define SMARTPORT_POLL_xF2         0xF2 //Physical ID 19 -
#define SMARTPORT_POLL_x53         0x53 //Physical ID 20 -
#define SMARTPORT_POLL_x34         0x34 //Physical ID 21 -
#define SMARTPORT_POLL_x95         0x95 //Physical ID 22 -
#define SMARTPORT_POLL_x16         0x16 //Physical ID 23 -
#define SMARTPORT_POLL_ACC         0xB7 //Physical ID 24 - IMU ACC (x,y,z)
#define SMARTPORT_POLL_x98         0x98 //Physical ID 25 -
#define SMARTPORT_POLL_PBOX        0x39 //Physical ID 26 - Power Box
#define SMARTPORT_POLL_TEMP        0xBA //Physical ID 27 - Temp
#define SMARTPORT_POLL_FUEL        0x1B //Physical ID 28 - Fuel (ArduPilot/Betaflight)
*/

/* FrSky SmartPort sensor IDs. See more here: https://github.com/opentx/opentx/blob/2.2/radio/src/telemetry/frsky.h */
#define SMARTPORT_ID_RSSI          0xf101
#define SMARTPORT_ID_RXA1          0xf102	// supplied by RX
#define SMARTPORT_ID_RXA2          0xf103	// supplied by RX
#define SMARTPORT_ID_BATV          0xf104
#define SMARTPORT_ID_SWR           0xf105   // Standing Wave Ratio
#define SMARTPORT_ID_T1            0x0400
#define SMARTPORT_ID_T2            0x0410
#define SMARTPORT_ID_RPM           0x0500
#define SMARTPORT_ID_FUEL          0x0600
#define SMARTPORT_ID_ALT           0x0100
#define SMARTPORT_ID_VARIO         0x0110   //VSPEED
#define SMARTPORT_ID_ACCX          0x0700
#define SMARTPORT_ID_ACCY          0x0710
#define SMARTPORT_ID_ACCZ          0x0720
#define SMARTPORT_ID_CURR          0x0200
#define SMARTPORT_ID_VFAS          0x0210  //Volt per Cell
#define SMARTPORT_ID_CELLS         0x0300
#define SMARTPORT_ID_HDIST         0x0420 //distance to GPS home fix, in meters
#define SMARTPORT_ID_PITCH         0x0430 //if frsky_pitch_roll = ON set this will be pitch degrees*10
#define SMARTPORT_ID_ROLL          0x0440 //if frsky_pitch_roll = ON set this will be roll degrees*10
#define SMARTPORT_ID_YAW           0x0450 //'Flight Path Vector' or 'Course over ground' in degrees*10
#define SMARTPORT_ID_GPS_LON_LAT   0x0800
#define SMARTPORT_ID_GPS_ALT       0x0820
#define SMARTPORT_ID_GPS_SPD       0x0830
#define SMARTPORT_ID_GPS_CRS       0x0840
#define SMARTPORT_ID_GPS_TIME      0x0850
#define SMARTPORT_ID_DIY_FIRST     0x5000
#define SMARTPORT_ID_DIY_LAST      0x50ff  //We have 256 possible ID's for custom values :)
#define SMARTPORT_ID_DIY_NAVSTATE  0x5000
#define SMARTPORT_ID_DIY_GPSFIX    0x5001

// Public functions
bool sPort_init(void);
void sPort_deinit(void);
void sPort_update_topics(const uint32_t now_ms);
void sPort_send_data(int uart, uint16_t id, uint32_t data);
void sPort_send_BATV(int uart);
void sPort_send_CUR(int uart);
void sPort_send_ALT(int uart);
void sPort_send_SPD(int uart);
void sPort_send_VSPD(int uart, float speed);
void sPort_send_FUEL(int uart);
void sPort_send_GPS_LON(int uart);
void sPort_send_GPS_LAT(int uart);
void sPort_send_GPS_ALT(int uart);
void sPort_send_GPS_SPD(int uart);
void sPort_send_GPS_CRS(int uart);
void sPort_send_GPS_TIME(int uart);
void sPort_send_flight_mode(int uart);
void sPort_send_GPS_info(int uart);

void sPort_send_NAV_STATE(int uart);
void sPort_send_GPS_FIX(int uart);

void sPort_send_HDIST(int uart);
void sPort_send_PITCH(int uart);
void sPort_send_ROLL(int uart);
void sPort_send_YAW(int uart);

#endif /* _SPORT_TELEMETRY_H */
