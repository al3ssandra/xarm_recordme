#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>
#include <rotational_interpolation_sa.hpp>
#include <memory>

int main() {
    // Create path
    KDL::Frame F_base_start(KDL::Vector(0.0,0.0,0.0315));
    // Will assume the vectors in rotation part are the columns of the rotation matrix
    KDL::Frame F_base_end(KDL::Rotation(KDL::Vector(1.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 1.0), KDL::Vector(0.0, -1.0, 0.0)), KDL::Vector(0.0, -0.0127125, 0.0230125));
    double eqradius = 0.001;
    // If I don't use new Rot... then I get segmentation fault (core dumped) so it probably is used so that it remains after some function ends or something
    // the destructor of Path_Line takes care of deleting that new allocation for RotationalInterpolation
    KDL::Path_Line* path = new KDL::Path_Line(F_base_start, F_base_end, new KDL::RotationalInterpolation_SingleAxis(), eqradius);

    // Create velocity profile
    double max_vel = 0.5;
    double max_acc = 0.1;
    KDL::VelocityProfile* velprof = new KDL::VelocityProfile_Trap(max_vel, max_acc);
    velprof->SetProfile(0, path->PathLength());  

    // Create trajectory
    // the heap memory allocated for path and velprof will be deleted in the destructor of Trajectory_Segment 
    KDL::Trajectory_Segment traject(path, velprof);

    // Use trajectory
    double dt = 0.01;
    for (double t=0.0; t <= traject.Duration(); t+= dt) {
			KDL::Frame current_pose;
			current_pose = traject.Pos(t);
            std::cout << current_pose << std::endl;
			// for (int i=0;i<4;++i)
			// 	for (int j=0;j<4;++j)
			// 		std::cout << current_pose(i,j) << std::endl;
			// also velocities and accelerations are available !
			//traject.Vel(t);
			//traject.Acc(t);
	}

    return 0;
}