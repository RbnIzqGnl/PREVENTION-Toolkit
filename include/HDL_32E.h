/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   HDL_32E.h
 * Author: ruben
 *
 * Created on 28 de febrero de 2018, 17:32
 */

#ifndef HDL_32E_H
#define HDL_32E_H

#ifdef __cplusplus
extern "C" {
#endif




#ifdef __cplusplus
}
#endif


#define HDL_GPS_PORT                8308
#define HDL_DATA_PORT               2368
#define HDL_NUM_ROT_ANGLES         36001
#define HDL_LASER_PER_FIRING          32
#define HDL_FIRING_PER_PKT            12
#define HDL_MAX_FIRING_PER_TURN     2500


#pragma pack(push, 1)

typedef struct HDLLaserReturn {
    unsigned short distance;
    unsigned char intensity;
} HDLLaserReturn;
#pragma pack(pop)

struct HDLFiringData {
    unsigned short blockIdentifier;
    unsigned short rotationalPosition;
    HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket {
    HDLFiringData firingData[HDL_FIRING_PER_PKT];
    unsigned int gpsTimestamp;
    unsigned char blank1;
    unsigned char blank2;
};

struct HDLIMUPacket {
    unsigned char blank1[14];
    unsigned short Giro1;
    unsigned short Temp1;
    unsigned short Acel1X;
    unsigned short Acel1Y;
    unsigned short Giro2;
    unsigned short Temp2;
    unsigned short Acel2X;
    unsigned short Acel2Y;
    unsigned short Giro3;
    unsigned short Temp3;
    unsigned short Acel3X;
    unsigned short Acel3Y;
    unsigned char blank2[160];
    unsigned int gpsTimestamp;
    unsigned char blank3[4];
    unsigned char NMEA[72];
    unsigned char blank4[234];
} __attribute__((packed));

struct HDLLaserCorrection {
    double azimuthCorrection;
    double verticalCorrection;
    double distanceCorrection;
    double verticalOffsetCorrection;
    double horizontalOffsetCorrection;
    double sinVertCorrection;
    double cosVertCorrection;
    double sinVertOffsetCorrection;
    double cosVertOffsetCorrection;
};


#endif /* HDL_32E_H */

