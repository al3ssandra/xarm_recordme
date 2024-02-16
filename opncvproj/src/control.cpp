#include <math.h>
#include <signal.h> // for signal()
#include <thread>
#include <atomic>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <servoCommands.h>
#include <yolo.h>

const double shift_threshold = 50; // if bbox shift from center of image is greater than this threshold (pixels) we do stuff. this is to avoid moving when is noise
const double horizontal_fov = 57 * KDL::PI / 180; // horizontal field or angle of view (rads) of front camera when phone is vertical
const double vertical_fov = 72 * KDL::PI / 180; // vertical field or angle of view (rads) of front camera when phone is vertical
const double j3 = 45.0;   // positive clockwise wrt. reference base frame
const double j4 = 90.0;   // positive clockwise wrt. reference base frame
const double j5 = -45.0;  // positive clockwise wrt. reference base frame
const double j6 = 0.0;    // positive anti clockwise wrt. reference base frame

const uint8_t Num = 4;
const uint16_t Time = 1000; //milliseconds
const int detect_rate = 20;

std::atomic<bool> isBusy(false);

// Signal handler for SIGINT (Ctrl+C)
void signalHandler(int signal) {
    std::cout << "Ctrl+C detected. Exiting program." << std::endl;
    // You can perform cleanup operations here if needed
    exit(signal); // Terminate the program with the received signal
}

// Robot joints angles go from -120 to 120 wrt. previous link. deg is in degrees, not rads.
uint16_t deg2serial(double deg) {
        return 500 + (1000/240) * deg;
}

int getNextJoints(KDL::ChainIkSolverVel_wdls& iksolverv, KDL::JntArray& jointpositions, const KDL::Twist& twist, KDL::JntArray& delta_joints, const unsigned int nj){
    Eigen::MatrixXd Mq = Eigen::MatrixXd::Identity(nj, nj);
    std::cout << Mq << std::endl;
    KDL::JntArray jointpositions_tmp(nj);
    int ret, i;
    bool flag;
    double small_w = 0.000001;

    while(true) {
        flag = true;
        ret = iksolverv.CartToJnt(jointpositions,twist,delta_joints);
        std::cout << "delta_joints: " << delta_joints(0) * 180 / KDL::PI << " " << delta_joints(1) * 180 / KDL::PI << " " << delta_joints(2) * 180 / KDL::PI << " " << delta_joints(3) * 180 / KDL::PI << std::endl;
        std::cout << "jointpositions: " << jointpositions(0) << " " << jointpositions(1) << " " << jointpositions(2) << " " << jointpositions(3) << std::endl;
        if (ret >= 0){
            KDL::Add(jointpositions,delta_joints,jointpositions_tmp);
            std::cout << "jointpositions_tmp: " << jointpositions_tmp(0) << " " << jointpositions_tmp(1) << " " << jointpositions_tmp(2) << " " << jointpositions_tmp(3) << std::endl;
            for(i=0; i<nj; i++){
                if((jointpositions_tmp(i) * 180/KDL::PI > 90) || (jointpositions_tmp(i) * 180/KDL::PI < -90)) {
                    std::cout << "joint " << i << " is out of bound" << std::endl;
                    if (Mq(i,i) == small_w) {
                        std::cerr << "It's trying to violate joint limit for joint " << i << "\n";
                        return -20;
                    }
                    else{
                        std::cout << "setting Mq(" << i << ") to " << small_w << std::endl;
                        Mq(i,i) = small_w;
                        std::cout << Mq << std::endl;
                        iksolverv.setWeightJS(Mq);
                        flag = false;
                        break;
                    }
                }
            }
            if (flag) {
                std::cout << "leaving loop" << std::endl;
                jointpositions = jointpositions_tmp;
                return ret;
            }
        }
        else{
            std::cerr << "Couldn't find inverse kinematics solution\n";
            return ret;
        }
    }
}

// void moveRobot(KDL::ChainFkSolverPos_recursive& fksolver, KDL::ChainIkSolverVel_wdls& iksolverv, const cv::Rect& box, const cv::Mat& frame, const unsigned int nj, KDL::JntArray& jointpositions){
void moveRobot(KDL::ChainFkSolverPos_recursive& fksolver, KDL::ChainIkSolverVel_pinv& iksolverv, const cv::Rect& box, const cv::Mat& frame, const unsigned int nj, KDL::JntArray& jointpositions, SerialWrapper& serialHandle){
    // Calculate forward position kinematics
    bool kinematics_status;
    KDL::Frame config_current;
    KDL::JntArray delta_joints(nj);
    KDL::JntArray joint_limits(nj);
    joint_limits(0) = 120.0 * KDL::PI / 180.0;
    joint_limits(1) = 90.0 * KDL::PI / 180.0;
    joint_limits(2) = 120.0 * KDL::PI / 180.0;
    joint_limits(3) = 120.0 * KDL::PI / 180.0;
    double max_joint_speed = 1.5;
    RobotServo servos[Num];
    int i;

    kinematics_status = fksolver.JntToCart(jointpositions,config_current);
    double x_current = config_current.p[0];
    double y_current = config_current.p[1];
    double z_current = config_current.p[2];

    // Get bbox center
    double x_bbox_center = box.x + box.width/2;
    double y_bbox_center = box.y + box.height/2;

    // Get image center
    double x_img_center = frame.cols/2;
    double y_img_center = frame.rows/2;

    // Get shift
    double x_shift = x_bbox_center - x_img_center;
    double y_shift = y_bbox_center - y_img_center;
    double module_shift = std::sqrt(pow(x_shift,2) + pow(y_shift,2));

    // If shift is bigger than threshold (to avoid noise)
    if (module_shift > shift_threshold) {
        // Calculate shift horizontal and vertical angles
        double delta_angle_x = x_shift * horizontal_fov / frame.cols; // horizontal shift angle in rads 
        double delta_angle_y = y_shift * vertical_fov / frame.rows; // vertical shift angle in rads 
        std::cout << "delta_angle_x: " << delta_angle_x * 180 / KDL::PI << std::endl;
        std::cout << "delta_angle_y: " << delta_angle_y * 180 / KDL::PI << std::endl;

        // Calculate next end effector frame origin x_next,y_next,z_next coordinates
        double d = std::sqrt(pow(x_current,2) + pow(y_current,2));
        double current_xy_angle = std::atan2(y_current, x_current); //xy is the plane at the base of robot, and this is end effector frame origin angle on that plane
        double next_xy_angle = current_xy_angle - delta_angle_x; // next angle is the current end effector origin fx angle + the horizontal shift angle
        double x_next = d * std::cos(next_xy_angle); // get next x coord
        double y_next = d * std::sin(next_xy_angle); // get next y coord
        double current_elevation_angle = std::atan(z_current/d); // the current elevation angle of the end effector origin over the xy plane
        double z_next = d * std::tan(current_elevation_angle - delta_angle_y); // next angle is the current end effector elevation + the vertical shift angle
        double dz = z_next - z_current;

        std::cout << "dz: " << dz << std::endl;
        double d_cm = std::sqrt(pow(x_next - x_current,2) + pow(y_next - y_current,2) + pow(z_next - z_current,2));
        std::cout << "delta centimeters: " << d_cm << std::endl;
       
        // Get next end effector config matrix by first rotating over z axis an angle delta horizontal, and then rotating up or down
        KDL::Rotation R = KDL::Rotation::RotZ(-delta_angle_x);
        // KDL::Rotation R = KDL::Rotation::RotZ(0.0);
        // KDL::Vector p(0.0, 0.0, dz);
        KDL::Vector p(0.0, 0.0, 0.0);
        KDL::Frame T(R, p);
        // Premultiply current frame by T to represent a transformation with respect to base frame
        KDL::Frame config_next = T * config_current;
        // // Postmultiply current frame by T roty to represent a transformation with respect to body frame
        KDL::Rotation R2 = KDL::Rotation::RotY(delta_angle_y);
        KDL::Frame T2(R2, p);
        config_next = config_next * T2;

        // Get joints angles from frame config using inverse kinematics
        KDL::Twist twist = KDL::diff(config_current, config_next);
        // int ret = getNextJoints(iksolverv, jointpositions, twist, delta_joints, nj);
        int ret = iksolverv.CartToJnt(jointpositions,twist,delta_joints);
        // If ret < 0 something went wrong
        std::cout << "ret: " << ret << std::endl;
        if(ret >= 0){
            for(i=0; i<nj; i++){
                if (std::abs(delta_joints(i)) > max_joint_speed){
                    auto s = delta_joints(i);
                    delta_joints(i) = s * max_joint_speed / std::abs(s);
                }
            }
            KDL::Add(jointpositions,delta_joints,jointpositions);
            // jointpositions(3) = jointpositions(1) + jointpositions(2); // make sure camera always stays vertical
            for(i=0; i<nj; i++){
                if (std::abs(jointpositions(i)) > joint_limits(i)){
                    auto a = jointpositions(i);
                    jointpositions(i) = a * joint_limits(i) / std::abs(a);
                }
            }                
            std::cout << "joint 6 (base): " << jointpositions(0) * 180 / KDL::PI << std::endl;
            std::cout << "joint 5 : " << jointpositions(1) * 180 / KDL::PI << std::endl;
            std::cout << "joint 4 : " << jointpositions(2) * 180 / KDL::PI << std::endl;
            std::cout << "joint 3 (tip): " << jointpositions(3) * 180 / KDL::PI << std::endl;

            // Move robot servos
            servos[0].ID = 3;
            servos[0].Position = deg2serial(jointpositions(3) * 180 / KDL::PI); // 500 + 350;  
            servos[1].ID = 4;
            servos[1].Position = deg2serial(-jointpositions(2) * 180 / KDL::PI); // 500 + 175; // the xarm controller reads joint 4 angle oposite for some reason
            servos[2].ID = 5;
            servos[2].Position = deg2serial(jointpositions(1) * 180 / KDL::PI); // 500 + 175; 
            servos[3].ID = 6;
            servos[3].Position = deg2serial(jointpositions(0) * 180 / KDL::PI); // 500;       
            std::cout << "servo 6 (base): " << servos[3].Position << std::endl;
            std::cout << "servo 5 : " << servos[2].Position << std::endl;
            std::cout << "servo 4 : " << servos[1].Position << std::endl;
            std::cout << "servo 3 (tip): " << servos[0].Position << std::endl;

            move_servos(serialHandle, Num, servos, Time);
            // std::this_thread::sleep_for(std::chrono::milliseconds(Time));
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(Time));
    isBusy.store(false);
}

int main(int argc, char **argv)
{
    // Register signal handler for SIGINT
    signal(SIGINT, signalHandler);
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Initial setup

    // Move robot servos to initial config
    SerialWrapper serialHandle("/dev/ttyACM0", B9600);
    RobotServo servos[Num];
    servos[0].ID = 3;
    servos[0].Position = deg2serial(j3); // 500 + 350;  
    servos[1].ID = 4;
    servos[1].Position = deg2serial(-j4); // 500 + 175; // the xarm controller reads joint 4 angle oposite for some reason
    servos[2].ID = 5;
    servos[2].Position = deg2serial(j5); // 500 + 175; 
    servos[3].ID = 6;
    servos[3].Position = deg2serial(j6); // 500;       
    move_servos(serialHandle, Num, servos, Time);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::ChainIkSolverVel_pinv iksolverv(chain, 0.001);//Inverse velocity solver
    // KDL::ChainIkSolverVel_wdls iksolverv(chain, 0.001);//Inverse velocity solver

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
    KDL::Frame config_current;
    KDL::JntArray delta_joints(nj);    

    // -----------------------------------------------------------------------------------------------------------------------------------------------------
    // Videocapture setup

    cv::Mat frame;
    // cv::VideoCapture capture("/robotic_tripod/opncvproj/videora.mp4");
    cv::VideoCapture capture("https://192.168.0.248:8080/video");
    if (!capture.isOpened())
    {
        std::cerr << "Error opening video file\n";
        return -1;
    }

    //  -------------------------------------------------------------------------------------------------------------------------------------------------------
    // DNN setup

    std::vector<std::string> class_list = load_class_list();

    bool is_cuda = argc > 1 && strcmp(argv[1], "cuda") == 0;

    cv::dnn::Net net;
    load_net(net, is_cuda);

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float fps = -1;
    int total_frames = 0;

    // ------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Loop

    while (true)
    {
        // Get detections
        capture.read(frame);
        if (frame.empty())
        {
            std::cout << "End of stream\n";
            break;
        }
        frame_count++;
        total_frames++;

        if ((!isBusy.load()) && (frame_count >= 30)){
            std::vector<Detection> output;
            detect(frame, net, output, class_list);

            // Only do stuff if we have detections
            int detections = output.size();
            if (detections > 0) {
                // Sort detections by bbox size
                // auto sortDetectionsLambda = [] (Detection const& a, Detection const& b) {
                //     return (a.box.height * a.box.width) > (b.box.height * b.box.width);
                // };
                // std::sort(output.begin(), output.end(), sortDetectionsLambda);

                // Get detection with biggest bbox
                auto detection = output[0];

                // Setup bbox for display
                auto box = detection.box;
                auto classId = detection.class_id;
                const auto color = colors[classId % colors.size()];
                cv::rectangle(frame, box, color, 3);

                cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
                cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

                if (frame_count >= 30)
                {

                    auto end = std::chrono::high_resolution_clock::now();
                    fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                    frame_count = 0;
                    start = std::chrono::high_resolution_clock::now();
                }

                if (fps > 0)
                {

                    std::ostringstream fps_label;
                    fps_label << std::fixed << std::setprecision(2);
                    fps_label << "FPS: " << fps;
                    std::string fps_label_str = fps_label.str();

                    cv::putText(frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                }

                // if (!isBusy.load()) {
                // Start a new thread for long calculations
                std::thread calculationThread(moveRobot, std::ref(fksolver), std::ref(iksolverv), std::ref(box), std::ref(frame), nj, std::ref(jointpositions), std::ref(serialHandle));
                calculationThread.detach(); // Detach the thread to run independently
                isBusy.store(true);
            }
        }
        cv::imshow("output", frame);
        // std::cout << "can press esc" << std::endl;
        if (cv::waitKey(1) != -1)
        {
            capture.release();
            cv::destroyAllWindows();
            std::cout << "finished by user\n";
            break;
        }
    }

    std::cout << "Total frames: " << total_frames << "\n";

    return 0;
}