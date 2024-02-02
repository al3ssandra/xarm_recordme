#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <stdio.h>
#include <iostream>


int main() {
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
    jointpositions(0) = 0.0;
    jointpositions(1) = 0.785398;
    jointpositions(2) = -0.785398;
    jointpositions(3) = 1.570796;

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

    // Inverse: find the joint angle velocities
    KDL::ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
    KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    // Initial joints angles
    KDL::JntArray q_init(chain.getNrOfJoints());
    q_init(0) = 0.0;
    q_init(1) = 0.0;
    q_init(2) = 0.0;
    q_init(3) = 0.0;

    //Set destination frame
    KDL::Frame F_dest = cartpos;

    // Calculate joint angles using inverse kinematics
    KDL::JntArray q_out(chain.getNrOfJoints());
    int ret = iksolver.CartToJnt(q_init,F_dest,q_out);
    std::cout << "IK Return: " << ret << std::endl;

    for (int i = 0; i < nj; i++) 
        std::cout << q_out(i) * 180 / KDL::PI << " degrees" << std::endl;
    printf("%s \n","Succes, thanks KDL!");

    return 0;
}