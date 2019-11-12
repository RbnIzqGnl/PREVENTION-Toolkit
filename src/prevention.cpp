/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "prevention.h"

FILE* open_raw_video_file(std::string file_name) {
    FILE *fd = fopen((char*) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error, file %s does not exist\n", (char*) file_name.c_str());
        exit(-1);
    }

    return fd;
}

int get_number_of_frames(FILE *fd, camParams_t camera_parameters) {

    if (fd == NULL)
    {
        printf("Error, fd does not exist\n");
        exit(-1);
    }

    fseek(fd, 0, SEEK_END);
    uint64_t bytes = ftell(fd);
    int frames = bytes / (camera_parameters.rows * camera_parameters.cols);

    return frames;
}

cv::Mat read_image_from_video(FILE *fd, int n_frame, camParams_t camera_parameters) {
    if (fd == NULL)
    {
        printf("Error, fd does not exist\n");
        exit(-1);
    }

    cv::Mat img_raw(camera_parameters.rows, camera_parameters.cols, CV_8UC1);

    unsigned image_size = sizeof (unsigned char) * camera_parameters.cols * camera_parameters.rows;
    unsigned long int offset = (unsigned long int) image_size * (unsigned long int) n_frame;
    if (fseek(fd, offset, SEEK_SET) != 0)
    {
        printf("Error while moving to frame %d in camera video\n", n_frame);
        exit(-1);
    }

    if (fread(img_raw.data, image_size, 1, fd) != 1)
    {
        printf("Error reading frame  %d in video_camera\n", n_frame);
        exit(-1);
    }

    cv::Mat img_bgr;
    cv::cvtColor(img_raw, img_bgr, CV_BayerBG2BGR);

    cv::Mat img_und;
    cv::remap(img_bgr, img_und, camera_parameters.map1, camera_parameters.map2, cv::INTER_CUBIC);

    return img_und;
}

camParams_t read_intrinsic_camera_calibration_file(std::string file_name) {
    FILE *fd = fopen((char*) file_name.c_str(), "r");

    if (fd == NULL)
    {
        printf("Error, file %s does not exist\n", (char*) file_name.c_str());
        exit(-1);
    }

    int ret_fs = 0;
    camParams_t ret;

    // Reads image size.
    ret_fs = fscanf(fd, "ImageSize %d %d\n", &(ret.cols), &(ret.rows));
    if (ret_fs != 2)
    {
        printf("\nERROR in read_intrinsic_camera_calibration_file: size of image hasn't been read %d\n", ret_fs);
        exit(-1);
    }
    printf("Image size %d, %d\n", ret.cols, ret.rows);

    // Read the intrinsic parameters.
    ret_fs = fscanf(fd, "IntrinsicParams %lf %lf %lf %lf %lf\n", &(ret.fx), &(ret.sk), &(ret.cx), &(ret.fy), &(ret.cy));
    if (ret_fs != 5)
    {
        printf("\nERROR in read_intrinsic_camera_calibration_file: the matrix of intrinsic parameters hasn't been read %d\n", ret_fs);
        exit(-1);
    }
    printf("Intrinsic matrix %lf, %lf, %lf, %lf, %lf\n", ret.fx, ret.sk, ret.cx, ret.fy, ret.cy);


    // Read distortion parameters.
    ret_fs = fscanf(fd, "DistortionCoefficients %lf %lf %lf %lf %lf\n", &(ret.k1), &(ret.k2), &(ret.k3), &(ret.p1), &(ret.p2));
    if (ret_fs != 5)
    {
        printf("\nERROR in read_intrinsic_camera_calibration_file: distortion parameters haven't been read\n");
        exit(-1);
    }
    printf("Distortion parameters (k1 k2 k3 p1 p2) %lf, %lf, %lf, %lf, %lf\n", ret.k1, ret.k2, ret.k3, ret.p1, ret.p2);

    // init undistortion matrix
    ret.k = (cv::Mat_<double>(3, 3) << ret.fx, ret.sk, ret.cx, 0, ret.fy, ret.cy, 0, 0, 1);
    ret.d = (cv::Mat_<double>(1, 5) << ret.k1, ret.k2, ret.p1, ret.p2, ret.k3);
    cv::initUndistortRectifyMap(ret.k, ret.d, cv::Mat(), ret.k, cv::Size(ret.cols, ret.rows), CV_32FC1, ret.map1, ret.map2);

    // Closes the file.
    fclose(fd);

    return ret;
}

cv::Mat read_extrinsic_camera_calibration_file(std::string file_name) {
    FILE* fd = fopen(file_name.c_str(), "r");

    // Open the file with the information about the stereo and correlation parameters.
    if (fd == NULL)
    {
        printf("File has not been opened\n");
        exit(-1);
    }

    int ret_fs = 0;
    cv::Mat RT = cv::Mat::eye(4, 4, CV_64FC1);


    // Reads R.
    ret_fs = fscanf(fd, "R %lf %lf %lf %lf  %lf %lf %lf %lf %lf\n",
            &(RT.at<double>(0, 0)), &(RT.at<double>(0, 1)), &(RT.at<double>(0, 2)),
            &(RT.at<double>(1, 0)), &(RT.at<double>(1, 1)), &(RT.at<double>(1, 2)),
            &(RT.at<double>(2, 0)), &(RT.at<double>(2, 1)), &(RT.at<double>(2, 2)));
    //RT.at<double>(3, 3) = 1.0;

    if (ret_fs != 9)
    {
        printf("\nERROR in read_extrinsic_camera_calibration_file: R matrix hasn't been read %d\n", ret_fs);
        exit(-1);
    }

    // Reads the translation matrix.
    ret_fs = fscanf(fd, "t %lf %lf %lf\n", &(RT.at<double>(0, 3)), &(RT.at<double>(1, 3)), &(RT.at<double>(2, 3)));
    if (ret_fs != 3)
    {
        printf("\nERROR in read_extrinsic_camera_calibration_file: translation matrix hasn't been read %d\n", ret_fs);
        exit(-1);
    }

    std::cout << "Camera extrinsic calibration" << std::endl << RT << std::endl;

    // Closes the file.
    fclose(fd);

    return RT;
}

cv::Mat read_extrinsic_calibration_file(std::string file_name) {
    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error opening file %s\n", file_name.c_str());
        exit(-1);
    }

    cv::Mat RT(4, 4, CV_64FC1);

    double val;
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
        {
            if (fscanf(fd, "%lf", &val) != 1)
            {
                printf("Error reading calibration parameters in %s\n", (char*) file_name.c_str());
                exit(-1);
            }
            RT.at<double>(r, c) = val;
        }
    // Closes the file
    fclose(fd);

    std::cout << "Extrinsic calibration" << std::endl << RT << std::endl;

    return RT;
}

CameraLog read_camera_log(std::string file_name) {
    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error opening file %s\n", file_name.c_str());
        exit(-1);
    }


    CameraLog camera_log;
    cam_entry_t entry;

    while (feof(fd) == 0)
    {
        if (fscanf(fd, "%d %d %lu %lu\n", &(entry.cloud), &(entry.frame), &(entry.t_0), &(entry.t_1)) != 4)
        {
            printf("Error in function read_manual_labels, reading file %s\n", (char *) file_name.c_str());
            exit(-1);
        }
        camera_log.push_back(entry);
    }

    fclose(fd);
    printf("Manual labels in file %s have been correctly read\n", (char*) file_name.c_str());

    return camera_log;
}

CNN_detections read_cnn_detections(std::string file_name) {

    FILE *f = fopen((char *) file_name.c_str(), "r");
    if (f == NULL)
    {
        printf("Error while reading %s\n", (char *) file_name.c_str());
        exit(-1);
    }

    int fault = 0;


    CNN_detections detections;
    detections.clear();

    detection_t aux;
    int nPoints = 0;

    while (feof(f) == 0 && fault == 0)
    {
        if (fscanf(f, "%d %d %d %f %f %f %f %f %d", &aux.frame, &aux.id, &aux.category, &aux.x0, &aux.y0, &aux.xf, &aux.yf, &aux.conf, &nPoints) != 9)
        {
            printf("Error while reading frame, id, category, x0, y0, xf, yf, or confidence\n");
            exit(-1);
        }

        aux.contour.clear();
        cv::Point paux;

        for (int j = 0; j < nPoints; j++)
        {
            if (fscanf(f, " %d %d", &paux.x, &paux.y) != 2)
            {
                printf("Error importing contour points from %s at frame %d\n", (char*) file_name.c_str(), aux.frame);
                exit(-1);
            }
            aux.contour.push_back(paux);
        }

        int n = fscanf(f, "\n");
        if (n != 0)
        {
            printf("Return not read from file %s\n", (char*) file_name.c_str());
            fault = 1;
        }
        detections.push_back(aux);

    }

    printf("Detections in file %s have been correctly read\n", (char*) file_name.c_str());

    return detections;
}

void draw_cnn_detections(cv::Mat img, CNN_detections *detections, int frame, int min_conf) {

    cv::Scalar color(0, 0, 255);
    std::vector < std::vector < cv::Point2i> > contours;

    for (uint i = 0; i < detections->size(); i++)
        if (detections->at(i).frame == frame)
            if (detections->at(i).conf >= min_conf)
                if (detections->at(i).id > 0)
                {
                    contours.clear();
                    contours.push_back(detections->at(i).contour);
                    cv::drawContours(img, contours, 0, cv::Scalar(0, 255, 0), 1);

                    cv::Point pt1(detections->at(i).x0, detections->at(i).y0);
                    cv::Point pt2(detections->at(i).xf, detections->at(i).yf);
                    cv::rectangle(img, pt1, pt2, color);

                    char id[20];
                    sprintf(id, "%d", detections->at(i).id);
                    cv::putText(img, id, pt1, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                }
}

Labels read_manual_labels(std::string file_name) {

    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error while reading %s\n", (char *) file_name.c_str());
        exit(-1);
    }

    Labels detections;
    entry_t aux;
    while (feof(fd) == 0)
    {
        if (fscanf(fd, "%e %e %e %e %e %e %e %e\n", &(aux.frame), &(aux.id), &(aux.x), &(aux.y), &(aux.w), &(aux.h), &(aux.xc), &(aux.yc)) != 8)
        {
            printf("Error in function read_manual_labels, reading file %s\n", (char *) file_name.c_str());
            exit(-1);
        }
        detections.push_back(aux);
    }

    fclose(fd);
    printf("Manual labels in file %s have been correctly read\n", (char*) file_name.c_str());

    return detections;
}

void draw_manual_labels(cv::Mat img, Labels *detections, int frame) {
    cv::Scalar color(255, 0, 0);

    for (uint i = 0; i < detections->size(); i++)
        if (detections->at(i).frame == frame)
        {
            char id[10];
            snprintf(id, 10, "%d", (int) detections->at(i).id);
            cv::Point pt((int) detections->at(i).x, (int) detections->at(i).y);
            cv::Rect rect((int) detections->at(i).x, (int) detections->at(i).y, (int) detections->at(i).w, (int) detections->at(i).h);

            cv::putText(img, id, pt, cv::FONT_HERSHEY_COMPLEX, 1, color, 1);
            cv::rectangle(img, rect, color, 1, 1);
        }
}

LaneChangeEvents read_lane_change_events(std::string file_name) {
    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error while reading %s\n", (char *) file_name.c_str());
        exit(-1);
    }

    LaneChangeEvents lane_change_events;
    event_t event;
    while (feof(fd) == 0)
    {
        if (fscanf(fd, "%d %d %d %d %d %d %d\n",  &(event.id_l), &(event.id_d), &(event.type), &(event.f0), &(event.fe), &(event.ff), &(event.blk)) != 7)
        {
            printf("Error reading file %s\n", (char *) file_name.c_str());
            exit(-1);
        }
        lane_change_events.push_back(event);
    }

    fclose(fd);
    printf("Events log file %s have been correctly read\n", (char*) file_name.c_str());

    return lane_change_events;
}

void draw_lane_change_events(cv::Mat img, LaneChangeEvents *lane_change_events, int frame) {
    cv::Scalar color(255, 255, 0);
    cv::Point pt(1500, 580);

    for (uint i = 0; i < lane_change_events->size(); i++)
        if (lane_change_events->at(i).f0 != -1)
            if (lane_change_events->at(i).f0 <= frame)
                if( lane_change_events->at(i).ff >= frame)
                {
                    char txt[50];
                    switch (lane_change_events->at(i).type)
                    {
                        case 1:
                            sprintf(txt, "cut-in by id %d", lane_change_events->at(i).id_d);
                            break;
                        case 2: sprintf(txt, "cut-out by id %d", lane_change_events->at(i).id_d);
                            break;
                        case 3: sprintf(txt, "left-ll by id %d", lane_change_events->at(i).id_d);
                            break;
                        case 4: sprintf(txt, "right-ll by id %d", lane_change_events->at(i).id_d);
                            break;
                        case 5: sprintf(txt, "hazardous by id %d", lane_change_events->at(i).id_d);
                            break;
                        case 6: sprintf(txt, "pedestrian by id %d", lane_change_events->at(i).id_d);
                            break;
                        default: sprintf(txt, "unknown by id %d", lane_change_events->at(i).id_d);
                            break;
                    }
                    cv::putText(img, txt, pt, cv::FONT_HERSHEY_COMPLEX, 1, color, 1);
                }
}

GroundCoefficients read_ground_coefficients(std::string file_name) {
    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error while reading %s\n", (char *) file_name.c_str());
        exit(-1);
    }

    GroundCoefficients ground_coefficients;
    coeffs_t coeffs;
    while (feof(fd) == 0)
    {
        float a, b, c, d;
        if (fscanf(fd, "%d %f %f %f %f\n", &(coeffs.frame), &a, &b, &c, &d) != 5)
        {
            printf("Error reading file %s\n", (char *) file_name.c_str());
            exit(-1);
        }
        coeffs.coeffs = cv::Vec4f(a, b, c, d);
        ground_coefficients.push_back(coeffs);
    }

    fclose(fd);
    printf("Ground coefficients file %s have been correctly read\n", (char*) file_name.c_str());

    return ground_coefficients;
}

float compute_vehicle_pitch(GroundCoefficients *ground_coefficients, int frame) {
    float pitch = 0;
    for (uint i = 0; i < ground_coefficients->size(); i++)
        if (ground_coefficients->at(i).frame == frame)
        {
            pitch = atan2(ground_coefficients->at(i).coeffs.val[0], ground_coefficients->at(i).coeffs.val[2]);
        }
    return pitch;
}

float compute_vehicle_roll(GroundCoefficients *ground_coefficients, int frame) {
    float roll = 0;
    for (uint i = 0; i < ground_coefficients->size(); i++)
        if (ground_coefficients->at(i).frame == frame)
        {
            roll = atan2(ground_coefficients->at(i).coeffs.val[1], ground_coefficients->at(i).coeffs.val[2]);
        }
    return roll;
}

FILE* open_velodyne_video_file(std::string file_name) {
    FILE *fd = fopen((char*) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error, file %s does not exist\n", (char*) file_name.c_str());
        exit(-1);
    }

    return fd;
}

int get_number_of_clouds(FILE *fd) {
    if (fd == NULL)
    {
        printf("Error, fd does not exist\n");
        exit(-1);
    }

    fseek(fd, 0, SEEK_END);
    uint64_t bytes = ftell(fd);
    int clouds = bytes / sizeof (CustomHDLCloud);

    return clouds;
}

cloud_t read_cloud_from_file(FILE *fd, int n_cloud) {

    if (fd == NULL)
    {
        printf("Error fd == NULL\n");
        exit(-1);
    }

    if (n_cloud < 0)
    {
        printf("Error nCloud < 0\n");
        exit(-1);
    }

    if (fseek(fd, n_cloud * sizeof (CustomHDLCloud), SEEK_SET) != 0)
    {
        printf("Error while moving to cloud %d in velodyne video\n", n_cloud);
        exit(-1);
    }

    CustomHDLCloud custom_cloud;
    if (fread(&custom_cloud, sizeof (CustomHDLCloud), 1, fd) != 1)
    {
        printf("Error reading cloud  %d in velodyne video\n", n_cloud);
        exit(-1);
    }

    cloud_t cloud = convert_custom_cloud(&custom_cloud);

    return cloud;
}

cloud_t convert_custom_cloud(CustomHDLCloud *custom_cloud) {
    double HDL_FIRING_ELEVATION[] = {-30.67, -9.33, -29.33, -8.00,
        -28.00, -6.67, -26.67, -5.33,
        -25.33, -4.00, -24.00, -2.67,
        -22.67, -1.33, -21.33, 0.00,
        -20.00, 1.33, -18.67, 2.67,
        -17.33, 4.00, -16.00, 5.33,
        -14.67, 6.67, -13.33, 8.00,
        -12.00, 9.33, -10.67, 10.67};

    cv::Mat cosE(32, 1, CV_64FC1), sinE(32, 1, CV_64FC1);
    for (int i = 0; i < 32; i++)
    {
        cosE.at<double>(i, 0) = cos(M_PI / 180.0 * HDL_FIRING_ELEVATION[i]);
        sinE.at<double>(i, 0) = sin(M_PI / 180.0 * HDL_FIRING_ELEVATION[i]);
    }
    cv::Mat cosEM, sinEM;

    cv::repeat(cosE, 1, custom_cloud->nValid, cosEM);
    cv::repeat(sinE, 1, custom_cloud->nValid, sinEM);


    double cosaPre[36000], sinaPre[36000];
    double convFactor = M_PI * 0.01 / 180.0;
    for (int i = 0; i < 36000; i++)
    {
        cosaPre[i] = cos(convFactor * (double) i);
        sinaPre[i] = sin(convFactor * (double) i);
    }

    cv::Mat d(32, custom_cloud->nValid, CV_64FC1);
    cv::Mat cosA(1, custom_cloud->nValid, CV_64FC1), sinA(1, custom_cloud->nValid, CV_64FC1);

    for (uint i = 0; i < custom_cloud->nValid; i++)
    {
        int azimuthint = (int) custom_cloud->firings[i].azimuth;
        cosA.at<double>(0, i) = cosaPre[azimuthint];
        sinA.at<double>(0, i) = sinaPre[azimuthint];

        for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
            d.at<double>(j, i) = 0.002 * (double) custom_cloud->firings[i].laserReturns[j].distance;
    }

    cv::Mat cosAM, sinAM;
    cv::repeat(cosA, 32, 1, cosAM);
    cv::repeat(sinA, 32, 1, sinAM);


    std::vector<cv::Mat> XYZ(3);
    XYZ.at(2) = sinEM.mul(d);
    cv::Mat XY = cosEM.mul(d);
    XYZ.at(0) = sinAM.mul(XY);
    XYZ.at(1) = cosAM.mul(XY);

    cv::Mat X = XYZ.at(0);
    cv::Mat Y = XYZ.at(1);
    cv::Mat Z = XYZ.at(2);


    cloud_t cloud;
    cv::Point3f pt;

    for (int r = 0; r < X.rows; r++)
        for (int c = 0; c < X.cols; c++)
        {
            pt.x = X.at<double>(r, c);
            pt.y = Y.at<double>(r, c);
            pt.z = Z.at<double>(r, c);
            cloud.push_back(pt);
        }

    return cloud;
}

RadarDetections read_radar_detections(std::string file_name) {
    FILE *fd = fopen((char *) file_name.c_str(), "r");
    if (fd == NULL)
    {
        printf("Error while reading %s\n", (char *) file_name.c_str());
        exit(-1);
    }

    RadarDetections radar_detections;
    radar_detection_t detection;
    detection.associated_frame = -1;

    while (feof(fd) == 0)
    {
        if (fscanf(fd, "%d %d %f %f %f %f %f %d %lu\n", &(detection.obj_n), &(detection.id), &(detection.x), &(detection.y), &(detection.vx), &(detection.vy), &(detection.rcs), &(detection.lt_poe), &(detection.t)) != 9)
        {
            printf("Error reading file %s\n", (char *) file_name.c_str());
            exit(-1);
        }
        if (!(detection.x == 0 && detection.y == 0 && detection.vx == 0 && detection.vy == 0))
            radar_detections.push_back(detection);
    }

    fclose(fd);
    printf("Radar detections file %s have been correctly read\n", (char*) file_name.c_str());

    return radar_detections;
}

void associate_radar_to_frames(RadarDetections *radar_detections, CameraLog *camera_log) {

    uint64_t first_time = camera_log->at(0).t_0;
    uint current_radar_index = 0;

    // set the radar data previous to the first detection as unused (frame = -1)
    for (uint i = 0; i < radar_detections->size(); i++)
        if (radar_detections->at(i).t < first_time)
            radar_detections->at(i).associated_frame = -1;
        else
        {
            current_radar_index = i;
            break;
        }

    // assing the corresponding frame to each radar detection
    for (uint i = 0; i < camera_log->size(); i++)
    {
        int this_frame = camera_log->at(i).frame;
        uint64_t this_time = camera_log->at(i).t_0;
        for (uint j = current_radar_index; j < radar_detections->size(); j++)
            if (radar_detections->at(j).t < this_time)
                radar_detections->at(j).associated_frame = this_frame;
            else
            {
                current_radar_index = j;
                break;
            }
    }

    // purge duplicate radar detections
    RadarDetections single_detections;
    uint current_radar_i = 0;
    for (uint f = 0; f < camera_log->size(); f++)
    {
        int this_frame = camera_log->at(f).frame;

        RadarDetections this_frame_detections;
        // create a vector with the detections of this frame
        for (uint i = current_radar_i; i < radar_detections->size(); i++)
            if (radar_detections->at(i).associated_frame == this_frame)
                this_frame_detections.push_back(radar_detections->at(i));
            else
            {
                if (radar_detections->at(i).associated_frame > this_frame)
                {
                    current_radar_i = i;
                    break;
                }
            }

        // purge detections with more than one sample in the intra-frame period
        int count1, count2;
        for (uint i = 0; i <= 40; i++)
        {
            count1 = 0;
            for (uint j = 0; j < this_frame_detections.size(); j++)
                if (this_frame_detections.at(j).obj_n == i)
                    count1++;

            count2 = 0;
            for (uint k = 0; k < this_frame_detections.size(); k++)
                if (this_frame_detections.at(k).obj_n == i)
                {
                    count2++;
                    if (count1 == count2)
                        single_detections.push_back(this_frame_detections.at(k));
                }
        }
    }

    radar_detections->clear();
    *radar_detections = single_detections;
}

cloud_t get_this_frame_radar_cloud(RadarDetections *detections, int frame, int minTh) {

    cloud_t cloud;
    // create a vector with the detections of this frame
    for (uint i = 0; i < detections->size(); i++)
        if (detections->at(i).associated_frame == frame)
            if (detections->at(i).lt_poe > minTh)
                cloud.push_back(cv::Point3f(detections->at(i).x, detections->at(i).y, 0.0));

    return cloud;
}

void draw_radar_map(cloud_t this_frame_detections) {
    cv::Mat img = cv::Mat::zeros(1000, 400, CV_8UC3);
    float ppm = 10;

    for (uint i = 0; i < this_frame_detections.size(); i++)
    {
        int r = 1000 - (int) this_frame_detections.at(i).x * ppm;
        int c = 200 - (int) this_frame_detections.at(i).y * ppm;
        cv::circle(img, cv::Point(c, r), 3, cv::Scalar(0, 255, 0));
    }
    cv::imshow("Radar", img);
}

FullExtrinsicCalibration compute_full_extrinsic_calibrations(cv::Mat l_to_c, cv::Mat l_to_v, cv::Mat r1_to_v, cv::Mat r2_to_v, cv::Mat r3_to_v) {

    FullExtrinsicCalibration full_calibration;

    // direct transformations
    full_calibration.from_velodyne_to_camera = l_to_c;
    full_calibration.from_velodyne_to_vehicle = l_to_v;
    full_calibration.from_radar1_to_vehicle = r1_to_v;
    full_calibration.from_radar2_to_vehicle = r2_to_v;
    full_calibration.from_radar3_to_vehicle = r3_to_v;

    // inverse transformations
    full_calibration.from_camera_to_velodyne = l_to_c.inv();
    full_calibration.from_vehicle_to_velodyne = l_to_v.inv();
    full_calibration.from_vehicle_to_radar1 = r1_to_v.inv();
    full_calibration.from_vehicle_to_radar2 = r2_to_v.inv();
    full_calibration.from_vehicle_to_radar3 = r3_to_v.inv();

    // composite transformations
    full_calibration.from_velodyne_to_radar1 = full_calibration.from_vehicle_to_radar1 * full_calibration.from_velodyne_to_vehicle;
    full_calibration.from_velodyne_to_radar2 = full_calibration.from_vehicle_to_radar2 * full_calibration.from_velodyne_to_vehicle;
    full_calibration.from_velodyne_to_radar3 = full_calibration.from_vehicle_to_radar3 * full_calibration.from_velodyne_to_vehicle;
    full_calibration.from_camera_to_vehicle = full_calibration.from_velodyne_to_vehicle * full_calibration.from_camera_to_velodyne;
    full_calibration.from_camera_to_radar1 = full_calibration.from_vehicle_to_radar1 * full_calibration.from_camera_to_vehicle;
    full_calibration.from_camera_to_radar2 = full_calibration.from_vehicle_to_radar2 * full_calibration.from_camera_to_vehicle;
    full_calibration.from_camera_to_radar3 = full_calibration.from_vehicle_to_radar3 * full_calibration.from_camera_to_vehicle;

    // inverse of composite transformations
    full_calibration.from_radar1_to_velodyne = full_calibration.from_velodyne_to_radar1.inv();
    full_calibration.from_radar2_to_velodyne = full_calibration.from_velodyne_to_radar2.inv();
    full_calibration.from_radar3_to_velodyne = full_calibration.from_velodyne_to_radar3.inv();
    full_calibration.from_vehicle_to_camera = full_calibration.from_camera_to_vehicle.inv();
    full_calibration.from_radar1_to_camera = full_calibration.from_camera_to_radar1.inv();
    full_calibration.from_radar2_to_camera = full_calibration.from_camera_to_radar2.inv();
    full_calibration.from_radar3_to_camera = full_calibration.from_camera_to_radar3.inv();


    return full_calibration;
}

cloud_t apply_geometric_transformation(cloud_t cloud, cv::Mat RT) {

    cloud_t out_cloud;

    cv::Mat pt_i(4, 1, CV_64FC1);
    cv::Mat pt_o(4, 1, CV_64FC1);


    for (uint i = 0; i < cloud.size(); i++)
    {
        pt_i.at<double>(0, 0) = cloud.at(i).x;
        pt_i.at<double>(0, 1) = cloud.at(i).y;
        pt_i.at<double>(0, 2) = cloud.at(i).z;
        pt_i.at<double>(0, 3) = 1.0;

        pt_o = RT * pt_i;

        cv::Point3f pt(pt_o.at<double>(0, 0), pt_o.at<double>(1, 0), pt_o.at<double>(2, 0));
        out_cloud.push_back(pt);
    }

    return out_cloud;
}

std::vector<cv::Point2d> project_cloud_to_camera(cloud_t cloud, camParams_t camera_parameters) {

    std::vector<cv::Point2d> img_cloud;

    cv::Mat pt_XYZ(3, 1, CV_64FC1);
    cv::Mat pt_UVW(3, 1, CV_64FC1);

    for (uint i = 0; i < cloud.size(); i++)
    {
        pt_XYZ.at<double>(0, 0) = cloud.at(i).x;
        pt_XYZ.at<double>(0, 1) = cloud.at(i).y;
        pt_XYZ.at<double>(0, 2) = cloud.at(i).z;

        pt_UVW = camera_parameters.k * pt_XYZ;

        double w = pt_UVW.at<double>(2, 0);
        double u = pt_UVW.at<double>(0, 0) / w;
        double v = pt_UVW.at<double>(1, 0) / w;
        if (w > 0)
        {
            if (u >= 0)
                if (u < camera_parameters.cols)
                {
                    if (v >= 0)
                        if (v < camera_parameters.rows)
                        {
                            img_cloud.push_back(cv::Point2d(u, v));
                        }
                }
        }
    }
    return img_cloud;
}

void draw_img_points(cv::Mat img, std::vector<cv::Point2d> img_pts, cv::Scalar color) {
    for (uint i = 0; i < img_pts.size(); i++)
        cv::circle(img, img_pts.at(i), 1, color, CV_FILLED);
}

cv::Mat x_axis_rotation_matrix(double ang){
    cv::Mat RT = cv::Mat::eye(4,4,CV_64FC1);
    RT.at<double>(1,1) = +cos(ang);
    RT.at<double>(1,2) = -sin(ang);
    RT.at<double>(2,1) = +sin(ang);
    RT.at<double>(2,2) = +cos(ang);
    return RT;
}
cv::Mat y_axis_rotation_matrix(double ang){
    cv::Mat RT = cv::Mat::eye(4,4,CV_64FC1);
    RT.at<double>(0,0) = +cos(ang);
    RT.at<double>(0,2) = sin(ang);
    RT.at<double>(2,0) = -sin(ang);
    RT.at<double>(2,2) = +cos(ang);
    return RT;
}
cv::Mat z_axis_rotation_matrix(double ang){
    cv::Mat RT = cv::Mat::eye(4,4,CV_64FC1);
    RT.at<double>(0,0) = +cos(ang);
    RT.at<double>(0,1) = -sin(ang);
    RT.at<double>(1,0) = +sin(ang);
    RT.at<double>(1,1) = +cos(ang);
    return RT;
}
