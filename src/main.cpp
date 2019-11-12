/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: ruben
 *
 * Created on 30 de enero de 2019, 11:57
 */

#include <cstdlib>

#include "prevention.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    if(argc != 2)
    {
        printf("Valid use of the program is ./prevention path_to_drive\n");
        exit(-1);
    }    

    std::string base_path(argv[1]);
    std::string camera1_video_file = base_path + "video_camera1.raw";
    std::string velodyne_video_file = base_path + "video_velodyne.bin";

    std::string camera1_intrinsic_calibration_file = base_path + "../camera1_intrinsic_calibration.dat";
    std::string camera1_extrinsic_calibration_file = base_path + "../camera1_extrinsic_calibration.dat";
    std::string velodyne_extrinsic_calibration_file = base_path + "../velodyne_extrinsic_calibration.dat";
    std::string radar1_extrinsic_calibration_file = base_path + "../radar1_extrinsic_calibration.dat";
    std::string radar2_extrinsic_calibration_file = base_path + "../radar2_extrinsic_calibration.dat";
    std::string radar3_extrinsic_calibration_file = base_path + "../radar3_extrinsic_calibration.dat";

    std::string camera1_log_file = base_path + "logs/log_camera1.txt";
    std::string ego_vehicle_log_file = base_path + "logs/log_ego-vehiclelog.txt";

    std::string camera1_cnn_detections_file = base_path + "detection_camera1/detections_tracked.txt";
    std::string camera1_manual_labels_file = base_path + "detection_camera1/labels.txt";
    std::string camera1_lane_changes_file = base_path + "detection_camera1/lane_changes.txt";
    std::string velodyne_ground_file = base_path + "detection_cloud/ground_coefficients.txt";

    std::string radar1_detections_file = base_path + "detection_radar/detections_radar1.txt";
    std::string radar2_detections_file = base_path + "detection_radar/detections_radar2.txt";
    std::string radar3_detections_file = base_path + "detection_radar/detections_radar3.txt";


    // Load calibration data
    camParams_t camera1_parameters = read_intrinsic_camera_calibration_file(camera1_intrinsic_calibration_file);
    cv::Mat camera_extrinsic_RT = read_extrinsic_camera_calibration_file(camera1_extrinsic_calibration_file);
    cv::Mat velodyne_extrinsic_RT = read_extrinsic_calibration_file(velodyne_extrinsic_calibration_file);
    cv::Mat radar1_extrinsic_RT = read_extrinsic_calibration_file(radar1_extrinsic_calibration_file);
    cv::Mat radar2_extrinsic_RT = read_extrinsic_calibration_file(radar2_extrinsic_calibration_file);
    cv::Mat radar3_extrinsic_RT = read_extrinsic_calibration_file(radar3_extrinsic_calibration_file);

    FullExtrinsicCalibration extrinsic_calibrations = compute_full_extrinsic_calibrations(camera_extrinsic_RT, velodyne_extrinsic_RT, radar1_extrinsic_RT, radar2_extrinsic_RT, radar3_extrinsic_RT);
    
    // Load logs
    CameraLog camera1_log = read_camera_log(camera1_log_file);

    // Radar detections
    RadarDetections radar1_detections = read_radar_detections(radar1_detections_file);
    RadarDetections radar2_detections = read_radar_detections(radar2_detections_file);
    RadarDetections radar3_detections = read_radar_detections(radar3_detections_file);

    // Radar detections are associated to the front camera images
    associate_radar_to_frames(&radar1_detections, &camera1_log);
    associate_radar_to_frames(&radar2_detections, &camera1_log);
    associate_radar_to_frames(&radar3_detections, &camera1_log);
    
    
    // Load detections
    CNN_detections camera1_cnn_detections = read_cnn_detections(camera1_cnn_detections_file);
    Labels camera1_manual_labels = read_manual_labels(camera1_manual_labels_file);
    LaneChangeEvents camera1_lane_changes = read_lane_change_events(camera1_lane_changes_file);
    GroundCoefficients ground_coefficients = read_ground_coefficients(velodyne_ground_file);


    // Open camera1 video file
    FILE *fd_raw_video = open_raw_video_file(camera1_video_file);
    int n_frames = get_number_of_frames(fd_raw_video, camera1_parameters);


    // Open velodyne video file
    FILE *fd_velodyne_video = open_velodyne_video_file(velodyne_video_file);
    int n_clouds = get_number_of_clouds(fd_velodyne_video);


    // Show images
    int frame = 2000;
    char q = '0';
    printf("Use WASD to move between frames\n");
    printf("Use 1-5 to enable/disable visual options\n");
    printf("Use q to exit\n");

    int represent_pointclud = 1;
    int represent_radar = 1;
    int represent_detections = 1;
    int represent_lanechanges = 1;
    int represent_manual_labels = 1;

    while(frame < n_frames && frame < n_clouds)
    {
        cv::Mat img = read_image_from_video(fd_raw_video, frame, camera1_parameters);
        
        float pitch = compute_vehicle_pitch(&ground_coefficients, frame);
        float roll = compute_vehicle_roll(&ground_coefficients, frame);
        cv::Mat pitch_correction = y_axis_rotation_matrix(pitch);
        
        
        // project velodyne points over the image
        if(represent_pointclud)
        {
            cloud_t cloud = read_cloud_from_file(fd_velodyne_video, frame);
            cloud_t camera_cloud = apply_geometric_transformation(cloud, extrinsic_calibrations.from_velodyne_to_camera);
            std::vector<cv::Point2d> img_pts_camera_cloud = project_cloud_to_camera(camera_cloud, camera1_parameters);
            draw_img_points(img, img_pts_camera_cloud, cv::Scalar(0, 255, 0));
        }
        
        if(represent_radar)
        {
            cloud_t radar2_cloud = get_this_frame_radar_cloud(&radar2_detections, frame, 2);
            cloud_t vehicle_radar2_cloud = apply_geometric_transformation(radar2_cloud, extrinsic_calibrations.from_radar2_to_vehicle);
            cloud_t vehicle_pitch_corrected_radar2_cloud = apply_geometric_transformation(vehicle_radar2_cloud, pitch_correction);
            cloud_t camera_radar2_cloud = apply_geometric_transformation(vehicle_pitch_corrected_radar2_cloud, extrinsic_calibrations.from_vehicle_to_camera);

            std::vector<cv::Point2d> img_pts_camera_radar2 = project_cloud_to_camera(camera_radar2_cloud, camera1_parameters);
            draw_img_points(img, img_pts_camera_radar2, cv::Scalar(0, 0, 255));
            draw_radar_map(vehicle_pitch_corrected_radar2_cloud);
        }
        
        if(represent_detections)
            draw_cnn_detections(img, &camera1_cnn_detections, frame, 0.5);

        if(represent_manual_labels)
            draw_manual_labels(img, &camera1_manual_labels, frame);

        if(represent_lanechanges)
            draw_lane_change_events(img, &camera1_lane_changes, frame);
        
        
        cv::imshow("BGR undistorted", img);
        printf("Frame %d -- pitch/roll[rad] %+08.5f/%+08.5f \n", frame, pitch, roll);
        q = cv::waitKey(0);
        

        // enalbe/disable visual options
        if(q == '1')
            represent_pointclud = !represent_pointclud;
        if(q == '2')
            represent_radar = !represent_radar;
        if(q == '3')
            represent_detections = !represent_detections;
        if(q == '4')
            represent_lanechanges = !represent_lanechanges;
        if(q == '5')
            represent_manual_labels = !represent_manual_labels;

        if (q == 'd')
            frame += 1;
        if (q == 'a')
            frame -= 1;
        if (q == 'w')
            frame += 50;
        if (q == 's')
            frame -= 50;
        if (q == 'q')
            frame = n_frames; // exit with equal number of frames

    }
}

