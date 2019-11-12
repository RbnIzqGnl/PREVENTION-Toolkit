/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   prevention_definitions.h
 * Author: ruben
 *
 * Created on 30 de enero de 2019, 13:12
 */

#ifndef PREVENTION_DEFINITIONS_H
#define PREVENTION_DEFINITIONS_H

// OpenCV 
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "HDL_32E.h"
#include <stdint.h>

typedef struct {
    // direct transformations
    cv::Mat from_velodyne_to_camera;
    cv::Mat from_velodyne_to_vehicle;
    cv::Mat from_radar1_to_vehicle;
    cv::Mat from_radar2_to_vehicle;
    cv::Mat from_radar3_to_vehicle;

    // inverse transformations
    cv::Mat from_camera_to_velodyne;
    cv::Mat from_vehicle_to_velodyne;
    cv::Mat from_vehicle_to_radar1;
    cv::Mat from_vehicle_to_radar2;
    cv::Mat from_vehicle_to_radar3;

    // composite transformations
    cv::Mat from_velodyne_to_radar1;
    cv::Mat from_velodyne_to_radar2;
    cv::Mat from_velodyne_to_radar3;
    cv::Mat from_camera_to_vehicle;
    cv::Mat from_camera_to_radar1;
    cv::Mat from_camera_to_radar2;
    cv::Mat from_camera_to_radar3;

    // inverse of composite transformations
    cv::Mat from_radar1_to_velodyne;
    cv::Mat from_radar2_to_velodyne;
    cv::Mat from_radar3_to_velodyne;
    cv::Mat from_vehicle_to_camera;
    cv::Mat from_radar1_to_camera;
    cv::Mat from_radar2_to_camera;
    cv::Mat from_radar3_to_camera;

} FullExtrinsicCalibration;

typedef struct {
    uint64_t gpsTime;
    uint16_t azimuth;
    HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
} CustomHDLFiringData;

typedef struct {
    uint16_t nCloud;
    uint16_t nValid;
    CustomHDLFiringData firings[HDL_MAX_FIRING_PER_TURN];
} CustomHDLCloud;

typedef std::vector<cv::Point3f> cloud_t;

typedef struct {
    int id_l;
    int id_d;
    int type;
    int f0;
    int fe;
    int ff;
    int blk;
} event_t;

typedef std::vector<event_t> LaneChangeEvents;

typedef struct {
    int frame;
    cv::Vec4f coeffs;
} coeffs_t;
typedef std::vector<coeffs_t> GroundCoefficients;

typedef struct {
    int obj_n;
    int id;
    float x;
    float y;
    float vx;
    float vy;
    float rcs;
    int lt_poe;
    uint64_t t;
    int associated_frame;
} radar_detection_t;
typedef std::vector<radar_detection_t> RadarDetections;

typedef struct {
    int frame;
    int cloud;
    uint64_t t_0;
    uint64_t t_1;
} cam_entry_t;
typedef std::vector<cam_entry_t> CameraLog;

typedef struct {
    int cols;
    int rows;
    cv::Mat k;
    cv::Mat d;
    double fx;
    double fy;
    double sk;
    double cx;
    double cy;
    double k1;
    double k2;
    double k3;
    double p1;
    double p2;
    cv::Mat map1;
    cv::Mat map2;
    //cv::Mat RT;
} camParams_t;

typedef struct {
    int category;
    int frame;
    int ind;
    int id;
    float x0;
    float y0;
    float xf;
    float yf;
    float conf;
    std::vector<cv::Point2i> contour;
} detection_t;

typedef std::vector<detection_t> CNN_detections;

typedef struct {
    float id;
    float frame;
    float x;
    float y;
    float w;
    float h;
    float xc;
    float yc;
} entry_t;

typedef std::vector<entry_t> Labels;

#endif /* PREVENTION_DEFINITIONS_H */

