//#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include "mymath.hpp"

#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
class Robot_Kinematics_Annalytical {
public:
    Robot_Kinematics_Annalytical();
    ~Robot_Kinematics_Annalytical();
    void FKine(KDL::JntArray jnt_in, KDL::Frame & frame_out);
    void IK_analytical(KDL::JntArray jnt_init, KDL::Frame frame, KDL::JntArray &jnt_out);

    KDL::Frame baseFrame;
    KDL::Frame toolFrame;
private:

    int IKSelection(double *q, int qnum, KDL::JntArray jnt_init, KDL::JntArray &jnt_out);
};

sensor_msgs::JointState InitJointState(KDL::JntArray initjnt);
