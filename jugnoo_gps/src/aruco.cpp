#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>


class Aruco
{

    
    cv::Mat image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    float actual_marker_length = 0.197;  // this should be in meters
    std::ostringstream vector_to_marker;
    
public:
    
    Aruco()
    {
        
    }
    
    void init()
    {
        cv::FileStorage fs("/home/jugnoo/config/calibration_params.yml", cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        
        // std::cout << "camera_matrix\n" << camera_matrix << std::endl;
        // std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
        
    }
    void detect(cv::Mat image, geometry_msgs::Pose &pose)
    {
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length,
                                                 camera_matrix, dist_coeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                                    rvecs[i], tvecs[i], 0.1);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                << "x: " << std::setw(8)<<  tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);

                pose.position.x = tvecs[0](0);
                pose.position.y = tvecs[0](1);
                pose.position.z = tvecs[0](2);

                // tf2::Quaternion q;

                cv::Mat R;
                cv::Rodrigues(rvecs[0], R);

                double qq[4];

                getQuaternion(R, qq);

                std::cout << rvecs[0] << "\n";
                // std::cout << R << "\n";
                // std::cout << qq << "\n";
                // q.setRPY( rvecs[0](2), rvecs[0](1), rvecs[0](0) );  // Create this quaternion from roll/pitch/yaw (in radians)

                // pose.orientation.x = q.getX();
                // pose.orientation.y = q.getY();
                // pose.orientation.z = q.getZ();
                // pose.orientation.w = q.getW();

                pose.orientation.x = qq[0];
                pose.orientation.y = qq[1];
                pose.orientation.z = qq[2];
                pose.orientation.w = qq[3];
                
            }
        }
        
        cv::imshow("Pose estimation", image_copy);
        
    }

    void getQuaternion(cv::Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}
    
};
