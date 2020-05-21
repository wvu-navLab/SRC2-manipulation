  void getInverseKinematics();
  void getJacobian();
  void invertJacobian();
  
/*--------------------------------------------------------------------
 * --------------------- KINEMATICS TOOLS ----------------------------
 *------------------------------------------------------------------*/

geometry_msgs::Twist MoveArm::getForwardKinematics(double q1, double q2, double q3, double q4)
{
  int N = theta_DH.size();
  theta_DH << PI, q1_goal, q2_goal+th2_star, q3_goal+th3_star, q4_goal+th4_star;
  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  for (size_t i = 0; i < N; i++)
  {
      Ai = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
      if (i == 0)
      {
          T0Ti = Ai;
      }
      else
      {
          T0Ti = T0Ti*Ai;
      }
  }
  T0Tn = T0Ti;
  
  pos_goal = T0Tn.block(0,3,3,1);
  geometry_msgs::Twist pose = 

}

void MoveArm::inverseKinematics()
{
  double r = sqrt(x_goal*x_goal + y_goal*y_goal);
  double r_E = r - a4*cos(phi_goal);
  double z_E = z_goal - h0 - d1 - a4*sin(phi_goal);
  double D = sqrt(z_E*z_E + r_E*r_E);

  double gamma = atan2(-z_E/D, -r_E/D) ;

  q1_goal = atan2(y_goal,x_goal);
  q2_goal = gamma - acos(-(r_E*r_E + z_E*z_E + a2*a2 - a3*a3)/(2*a2*D));
  q3_goal = atan2((z_E-l2*sin(q2_goal))/a3,(r_E-a2*cos(q2_goal))/a3);
  q4_goal = phi_goal- (q2_goal + q3_goal);

  printf ("The goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  constrainAngle(q1_goal);
  constrainAngle(q2_goal);
  constrainAngle(q3_goal);
  constrainAngle(q4_goal);

  printf ("The constrained goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  limitJoint(q1_goal, JOINT1_MAX, JOINT1_MIN);
  limitJoint(q2_goal, JOINT2_MAX, JOINT2_MIN);
  limitJoint(q3_goal, JOINT3_MAX, JOINT3_MIN);
  limitJoint(q4_goal, JOINT4_MAX, JOINT4_MIN);

  printf ("The constrained goal joint angles with limits are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);
}

void MoveArm::getJacobian()
{
  int N = theta_DH.size();

  Eigen::Vector3d on(3);
  on = T0Tn.block(0,3,3,1);

  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  Eigen::Vector3d oi(3);
  Eigen::Vector3d zi(3);
  for (size_t i = 0; i < N; i++)
  {
    Ai = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
    if (i == 0)
    {
        T0Ti = Ai;
        zi = T0Ti.block(0,2,3,1);
        oi = T0Ti.block(0,3,3,1);
    }
    else
    {
        // Calculate Jacobian column with previous origin and z-axis (i-1)
        J.block(0, i-1, 3, 1) = zi.cross(on - oi);
        J.block(3, i-1, 3, 1) = zi;
        // Update origin and z-axis (i)
        T0Ti = T0Ti * Ai;
        zi = T0Ti.block(0,2,3,1);
        oi = T0Ti.block(0,3,3,1);
    }
  }
}

void MoveArm::invertJacobian(){
  pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
}

/*--------------------------------------------------------------------
 * -------------------------- CALLBACKS ------------------------------
 *------------------------------------------------------------------*/

void MoveArm::controlJoints(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (armControlEnabled)
  {
    // Find current angles and position
    double q1_current = msg->position[15];
    double q2_current = msg->position[0];
    double q3_current = msg->position[8];
    double q4_current = msg->position[7];

    printf ("Current joint angles.\n");
    theta_DH << PI, q1_current, q2_current, q3_current, q4_current;
    std::cout << theta_DH << std::endl;

    // Update current joint angles position
    theta_DH << PI, q1_current, q2_current+th2_star, q3_current+th3_star, q4_current+th4_star;

    forwardKinematics();
    Eigen::VectorXd pos_current = T0Tn.block(0,3,3,1);
    printf("Current Position Vector:\n");
    std::cout << pos_current << std::endl;
    Eigen::VectorXd e = pos_goal - pos_current;

    printf("Position Error Vector:\n");
    std::cout << e << std::endl;

    double pos_error = e.norm(); 

    if (pos_error > max_pos_error) 
    { 
      // Calculate end-effector velocity cmd
      Eigen::VectorXd v(3);
      Eigen::VectorXd qdot(4);
      v = K_pos * e / e.norm();
      // Find qdot using error and Jacobian
      getJacobian();
      printf("Jacobian:\n");
      std::cout << J << std::endl;
      invertJacobian();
      printf("Inverse Jacobian:\n");
      std::cout << pinvJ << std::endl;
      qdot = pinvJ.block(0,0,4,3) * v;
      printf("Commanded qdot:\n");
      std::cout << qdot << std::endl;
      printf ("Joint 1 ti (%2.6f).\n", q1_current);
      printf ("Joint 1 ti+1 (%2.6f).\n", q1_current + dt*qdot(0));
      // Publish new desired joint angles
      q.q1 = q1_current + dt*qdot(0);
      q.q2 = q2_current + dt*qdot(1);
      q.q3 = q3_current + dt*qdot(2);
      q.q4 = q4_current + dt*qdot(3);
      printf ("Published joint angles.\n");
      std::cout << q << std::endl;
      pubJointAngles.publish(q);
    }
  }
}



  // if(mass_collected>98)
  // {
  //   move_excavator::ExcavationStatus msg;
  //   msg.isFinished = true;
  //   msg.collectedMass = 100;
  //   pubFinished.publish(msg);
  //   mass_collected = 0;
  // }
    
  // if(!isBucketFull)
  // {
  //   double mass = msg->mass_in_bucket;
  //   mass_collected = mass_collected + mass;
  //   if (mass != 0)
  //   {
  //     isBucketFull = true;
  //     ROS_INFO("A Pokémon is on the hook!");
  //     ROS_INFO_STREAM("Mass extracted so far:" << mass_collected);
  //     ROS_INFO_STREAM("Mass extracted this time:" << mass);
  //     ROS_INFO("Homing the manipulator..");
  //     move_excavator::HomeArm srv;
  //     bool success = clientHomeArm.call(srv);
  //     ros::Duration(10).sleep();
  //     isArmHome = true;
  //   }
  //   else
  //   {
  //     ROS_INFO("Nothing on the bucket.");
  //     isBucketFull = false;
  //   }
  // }
  // else
  // {
  //   if(isArmHome)
  //   {
  //     if(isAligned)
  //     {
  //       ROS_INFO("Extending Arm.");
  //       callExtendArm();
  //       ROS_INFO("Dropping Volatile.");
  //       callDropVolatile();
  //       enableAlign = false;
  //       isAligned = false;
  //     }
  //     else
  //     {
  //       ROS_INFO("Enabling Align.");
  //       enableAlign = true;
  //     }
  //   }
  // }


    // if(enableAlign)
  // {
  //   if(!isRoverInRange && !isRoverInView)
  //   {
  //     ROS_INFO("Rotating In Place.");
  //     driving_tools::RotateInPlace srv;
  //     srv.request.throttle = 0.5;
  //     bool success = clientRotateInPlace.call(srv);
  //   }
  //   else if(isRoverInView && !isRoverInRange)
  //   {
  //     ROS_INFO("Coming Closer.");
  //     driving_tools::MoveForward srv;
  //     srv.request.throttle = 0.5;
  //     bool success = clientMoveForward.call(srv);
  //   }
  //   else if(isRoverInView && isRoverInRange)
  //   {
  //     ROS_INFO("Stopping.");
  //     isAligned = true;
  //     enableAlign = false;
  //     driving_tools::Stop srv;
  //     srv.request.enableStop = true;
  //     bool success = clientStop.call(srv);
  //   }
  //   else
  //   {
  //     ROS_INFO("Something in front of me, but not the hauler.");
  //     driving_tools::Stop srv;
  //     srv.request.enableStop = true;
  //     bool success = clientStop.call(srv);
  //   }     
  // }