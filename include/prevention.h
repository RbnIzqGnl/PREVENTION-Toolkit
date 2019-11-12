/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   prevention.h
 * Author: ruben
 *
 * Created on 30 de enero de 2019, 11:59
 */

#ifndef PREVENTION_H
#define PREVENTION_H

#include "prevention_definitions.h"


#include <stdio.h>

// OpenCV 
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


// Calib input
camParams_t read_intrinsic_camera_calibration_file(std::string file_name);
cv::Mat read_extrinsic_camera_calibration_file(std::string file_name);
cv::Mat read_extrinsic_calibration_file(std::string file_name);

// Log input
CameraLog read_camera_log(std::string file_name);


// Image input
FILE* open_raw_video_file(std::string file_name);
int get_number_of_frames(FILE *fd, camParams_t camera_parameters);
cv::Mat read_image_from_video(FILE *fd, int n_frame, camParams_t camera_parameters);


// Cloud input
FILE* open_velodyne_video_file(std::string file_name);
int get_number_of_clouds(FILE *fd);
cloud_t read_cloud_from_file(FILE *fd, int n_cloud);
cloud_t convert_custom_cloud(CustomHDLCloud *custom_cloud);

// Radar input
RadarDetections read_radar_detections(std::string file_name);
void associate_radar_to_frames(RadarDetections *radar_detections, CameraLog *camera_log);
cloud_t get_this_frame_radar_cloud(RadarDetections *detections, int frame,  int minTh);
void draw_radar_map(cloud_t this_frame_detections);

// Compute transformations
FullExtrinsicCalibration compute_full_extrinsic_calibrations(cv::Mat l_to_c, cv::Mat l_to_v, cv::Mat r1_to_v, cv::Mat r2_to_v, cv::Mat r3_to_v);

// CNN Labels
CNN_detections read_cnn_detections(std::string file_name);
void draw_cnn_detections(cv::Mat img, CNN_detections *detections, int frame, int min_conf); 

// Manual Labels
Labels read_manual_labels(std::string file_name);
void draw_manual_labels(cv::Mat img, Labels *detections, int frame); 

// Lane change events
LaneChangeEvents read_lane_change_events(std::string file_name);
void draw_lane_change_events(cv::Mat img, LaneChangeEvents *lane_change_events, int frame);

// Cloud detections
GroundCoefficients read_ground_coefficients(std::string file_name);
float compute_vehicle_pitch(GroundCoefficients *ground_coefficients, int frame);
float compute_vehicle_roll(GroundCoefficients *ground_coefficients, int frame);

// Geometric transformations
cloud_t apply_geometric_transformation(cloud_t cloud, cv::Mat RT);
std::vector<cv::Point2d> project_cloud_to_camera(cloud_t cloud, camParams_t camera_parameters);


// Misc
void draw_img_points(cv::Mat img, std::vector<cv::Point2d> img_pts, cv::Scalar color);
cv::Mat x_axis_rotation_matrix(double ang);
cv::Mat y_axis_rotation_matrix(double ang);
cv::Mat z_axis_rotation_matrix(double ang);

#endif /* PREVENTION_H */

