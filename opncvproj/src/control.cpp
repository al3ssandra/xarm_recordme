#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <servoCommands.h>
#include <yolo.h>


// Robot joints angles go from -120 to 120 wrt. previous link. deg is in degrees, not rads.
uint16_t deg2serial(double deg) {
    return 500 + (1000/240) * deg;
}

int main(int argc, char **argv)
{
    double j3 = 45.0;   // positive clockwise wrt. reference base frame
    double j4 = 90.0;   // positive clockwise wrt. reference base frame
    double j5 = -45.0;  // positive clockwise wrt. reference base frame
    double j6 = 0.0;    // positive anti clockwise wrt. reference base frame

    SerialWrapper serialHandle("/dev/ttyACM0", B9600);
    uint8_t Num = 4;
    RobotServo servos[Num];
    servos[0].ID = 3;
    servos[0].Position = deg2serial(j3); // 500 + 350;  
    servos[1].ID = 4;
    servos[1].Position = deg2serial(-j4); // 500 + 175; // the xarm controller reads joint 4 angle oposite for some reason
    servos[2].ID = 5;
    servos[2].Position = deg2serial(j5); // 500 + 175; 
    servos[3].ID = 6;
    servos[3].Position = deg2serial(j6); // 500;       
    uint16_t Time = 1000;
    move_servos(serialHandle, Num, servos, Time);

    // ------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Setup robot arm config

    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    // Will assume lenghts are in meters, and reference frame is at base of arm 
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.0063))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0102))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0095))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0055))));

    // Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    // Assign some values to the joint positions, I'll assume position means angle in rads, with respect to previous link? it's servo angle, correct
    // I will also assume the order of joints is the order in which I added the segments in chain? yes it is
    jointpositions(0) = j6 * KDL::PI / 180.0;       // servo 6 base
    jointpositions(1) = j5 * KDL::PI / 180.0;       // servo 5
    jointpositions(2) = j4 * KDL::PI / 180.0;       //servo 4
    jointpositions(3) = j3 * KDL::PI / 180.0;       //servo 3 tip

    // Create the frame that will contain the results
    // output is going to be 4x3, the top 3x3 of the matrix is the rotation matrix, and the bottom 1x3 is the translation vector
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos << std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

//     // -----------------------------------------------------------------------------------------------------------------------------------------------------

//     std::vector<std::string> class_list = load_class_list();

//     cv::Mat frame;
//     cv::VideoCapture capture("/robotic_tripod/opncvproj/videora.mp4");
//     // cv::VideoCapture capture("url/video");
//     if (!capture.isOpened())
//     {
//         std::cerr << "Error opening video file\n";
//         return -1;
//     }

//     bool is_cuda = argc > 1 && strcmp(argv[1], "cuda") == 0;

//     cv::dnn::Net net;
//     load_net(net, is_cuda);

//     auto start = std::chrono::high_resolution_clock::now();
//     int frame_count = 0;
//     float fps = -1;
//     int total_frames = 0;

//     while (true)
//     {
//         capture.read(frame);
//         if (frame.empty())
//         {
//             std::cout << "End of stream\n";
//             break;
//         }

//         std::vector<Detection> output;
//         detect(frame, net, output, class_list);

//         frame_count++;
//         total_frames++;

//         int detections = output.size();

//         for (int i = 0; i < detections; ++i)
//         {

//             auto detection = output[i];
//             auto box = detection.box;
//             auto classId = detection.class_id;
//             const auto color = colors[classId % colors.size()];
//             cv::rectangle(frame, box, color, 3);

//             cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
//             cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//         }

//         if (frame_count >= 30)
//         {

//             auto end = std::chrono::high_resolution_clock::now();
//             fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

//             frame_count = 0;
//             start = std::chrono::high_resolution_clock::now();
//         }

//         if (fps > 0)
//         {

//             std::ostringstream fps_label;
//             fps_label << std::fixed << std::setprecision(2);
//             fps_label << "FPS: " << fps;
//             std::string fps_label_str = fps_label.str();

//             cv::putText(frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
//         }

//         cv::imshow("output", frame);

//         if (cv::waitKey(1) != -1)
//         {
//             capture.release();
//             std::cout << "finished by user\n";
//             break;
//         }
//     }

//     std::cout << "Total frames: " << total_frames << "\n";

//     return 0;
}