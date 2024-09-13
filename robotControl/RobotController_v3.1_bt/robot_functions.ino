/********************** Variables etc for Robot Control *****************************/


int VRobot = 0;         // Current linear velocity along the heading
int ThetaDot = 0;       // Can range from -MaxThetaDot to MaxThetaDot  (Difference between Vr and Vl may not exceed MaxThetaDot)
/*************************************************************************************/


void set_robot_speed(int Speed) // function to set robot linear velocity
{
    VRobot = Speed;
    update_robot_velocity();
}

int get_robot_speed(void)
{
  return(VRobot);
}

void ramp_robot_speed(int Speed)
{
  while(abs(VRobot-Speed)>=10 )
  {
    if(VRobot < Speed)
    VRobot += 10;

    if(VRobot > Speed)
    VRobot -= 10;

    update_robot_velocity();
    delay(2);
  }
  
}

void set_robot_omega(int omega)
{
     ThetaDot = omega;
     update_robot_velocity();
}
int get_robot_omega(void)
{
  return(ThetaDot);
}

void stop_robot(void)
{
  stop_motors();
}

void ramp_robot_omega(int omega)
{
  while(abs(ThetaDot-omega)>=10 )
  {
    if(ThetaDot < omega)
    ThetaDot += 10;

    if(ThetaDot > omega)
    ThetaDot -= 10;

    update_robot_velocity();
    delay(1);
  }
  
}

void update_robot_velocity()
{
  //int Vr = (VRobot - (int)(limit_factor * abs(ThetaDot))) - ThetaDot;  // Duty Cycle of the Right Motors  (0-1023)
  //int Vl = (VRobot - (int)(limit_factor * abs(ThetaDot))) + ThetaDot;  // Duty Cycle of the Left Motors  (0-1023)

  int Vr = VRobot - ThetaDot;  // Duty Cycle of the Right Motors  (0-1023)
  int Vl = VRobot + ThetaDot;  // Duty Cycle of the Left Motors  (0-1023)
  
  set_motor_speed(RIGHT_MOTOR, Vr);
  set_motor_speed(LEFT_MOTOR, Vl);
}

/*void follow_wall(uint8_t side, float dist )
{
  
}*/
