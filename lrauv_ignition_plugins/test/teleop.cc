/**
 * Keyboard teleop controller
 */ 
#include <unistd.h>
#include <termios.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "lrauv_command.pb.h"


char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    perror ("tcsetattr ~ICANON");
  return (buf);
}

int main(int argc, char** argv)
{
  ignition::transport::Node node;
  auto commandTopic = "command_topic";
  auto commandPub = 
    node.Advertise<lrauv_ignition_plugins::msgs::LRAUVCommand>(commandTopic);

  double rudder_angle = 0;
  double elevator_angle = 0;
  double thrust = 0;
  while(true)
  {

    std::cout << "\033[2J";
    std::cout<< "Keyboard teleop for lrauv" << std::endl;
    std::cout << "  w  <-- Control elevator to point up" <<std::endl;
    std::cout << "a   d  <-- Control Rudder left/right" <<std::endl;
    std::cout << "  s  <-- Point Elevator down" <<std::endl;

    std::cout << "Throttle control:" <<std::endl;
    std::cout << "\tj - increase thrust "<< std::endl;
    std::cout << "\tk - decrease thrust "<< std::endl;

    std::cout << "Current state:" << std::endl;
    std::cout << "\tThrust (rpm): " << thrust << std::endl;
    std::cout << "\tRudder angle (radians): " << rudder_angle << std::endl;
    std::cout << "\tElevator angle (radians): " << elevator_angle << std::endl;

    int res = getch();
    if (res < 0)
      break;

    switch(res)
    {
      case 'j':
        thrust += 10;
        break;

      case 'k':
        thrust -= 10;
        break;

      case 'w':
        elevator_angle += 0.01;
        break;
        
      case 's':
        elevator_angle -= 0.01;
        break;
      
      case 'a':
        rudder_angle -= 0.01;
        break;

      case 'd':
        rudder_angle += 0.01;
        break;
    }

    lrauv_ignition_plugins::msgs::LRAUVCommand cmd;
    cmd.set_propomegaaction_(thrust);
    cmd.set_elevatorangleaction_(elevator_angle);
    cmd.set_rudderangleaction_(rudder_angle);
    commandPub.Publish(cmd);
  }

}
