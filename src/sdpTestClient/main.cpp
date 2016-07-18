//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    main.cpp
  \brief   The ssl-vision application entry point.
  \author  Stefan Zickler, (C) 2008
*/
//========================================================================

//#include <QApplication>
//#include <QCleanlooksStyle>
//#include <QPlastiqueStyle>
//#include "mainwindow.h"

#include <stdio.h>
#include <QThread>
#include "robocup_ssl_client.h"
#include "timer.h"
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <cmath>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#define R_FORWARD   87
#define R_TURNLEFT  65
#define R_TURNRIGHT 68
#define R_STRAFELEFT    67
#define R_STRAFERIGHT   86
#define R_STOP      32
#define R_GRABBERSOPEN  90
#define R_GRABBERSCLOSE 88
#define R_KICK      81

// serial stuff
// taken from http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

void printRobotInfo(const SSL_DetectionRobot & robot) {
    printf("CONF=%4.2f ", robot.confidence());
    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(),robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }
    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());
}


int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    // init serial comms to RF stick
    char *portname = "/dev/ttyACM0";
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
            return 0;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking


    enum rstates {R_CLOSEDNOBALL, R_OPENNOBALL, R_CLOSEDWITHBALL, R_OPENWITHBALL, R_JUSTKICKED, R_CLOSINGGRABBERS};
    bool robotnearball = false;
    rstates rstate = R_CLOSEDNOBALL;

    double t_lastInstructionSent = 0;
    double t_lastTurnInstruction = 0; 
    while(true) {
        if (client.receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();

                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %7.3fms\n",(detection.t_sent()-detection.t_capture())*1000.0);
                printf("Network Latency (assuming synched system clock) %7.3fms\n",(t_now-detection.t_sent())*1000.0);
                printf("Total Latency   (assuming synched system clock) %7.3fms\n",(t_now-detection.t_capture())*1000.0);
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();

                //Ball info:
                for (int i = 0; i < balls_n; i++) {
                    SSL_DetectionBall ball = detection.balls(i);
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ball.x(),ball.y());
                    if (ball.has_z()) {
                        printf("Z=%7.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());
                }

                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot);
                }

                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);
                }

                // horribly hacky demo code
                // assumes group 14 robot is yellow

                float ballAngle;
                float ballDist;
                if ((robotnearball || balls_n > 0) && robots_yellow_n > 0){
                    printf("robopos: %2.2f, %2.2f, %2.2f\n", detection.robots_yellow(0).x(), detection.robots_yellow(0).y(), detection.robots_yellow(0).orientation());
                    if (balls_n > 0){
                    printf("ballpos: %2.2f, %2.2f\n", detection.balls(0).x(), detection.balls(0).y());
                        float distx = detection.balls(0).x() - detection.robots_yellow(0).x();
                        float disty = detection.balls(0).y() - detection.robots_yellow(0).y();
                        ballAngle = atan2(disty, distx) - detection.robots_yellow(0).orientation();
                        ballDist = sqrt(distx*distx + disty*disty);
                        if (ballDist > 500){
                            robotnearball = false;
                            //if (rstate == R_CLOSEDWITHBALL) rstate = R_CLOSEDNOBALL;
                            //if (rstate == R_OPENWITHBALL) rstate = R_OPENNOBALL;
                        }
                        printf("ballToRobot: %2.2f, %2.2f, %2.2f, %2.2f\n", distx, disty, ballAngle, ballDist);
                    }
                    if (t_now - t_lastInstructionSent > 0.2){
                        switch (rstate){
                            case R_CLOSEDNOBALL:
                            case R_OPENNOBALL:
                                if (ballAngle > 0.2 && ballAngle < 3.14){
                                    if (t_now - t_lastTurnInstruction > 2.0){
                                        uint8_t cmd[2] = {R_TURNLEFT, (uint8_t)(ballAngle*28.7)};
                                        write (fd, cmd, 2); 
                                        t_lastTurnInstruction = t_now;
                                        t_lastInstructionSent = t_now;
                                    }
                                } else if (ballAngle < -0.2){
                                    if (t_now - t_lastTurnInstruction > 2.0){
                                        uint8_t cmd[2] = {R_TURNRIGHT, (uint8_t)((0-ballAngle)*28.7)};
                                        write (fd, cmd, 2); 
                                        t_lastTurnInstruction = t_now;
                                        t_lastInstructionSent = t_now;
                                    }
                                } else if (ballDist > 1000){
                                    uint8_t cmd[2] = {R_FORWARD, 150};
                                    write (fd, cmd, 2);
                                    t_lastInstructionSent = t_now; 
                                } else if (ballDist > 300){
                                    if (ballDist < 500) robotnearball = true;
                                    uint8_t cmd[2] = {R_GRABBERSOPEN, 0};
                                    write (fd, cmd, 2);
                                    rstate = R_OPENNOBALL;
                                    cmd[0] = R_FORWARD;
                                    cmd[1] = 100;
                                    write (fd, cmd, 2);
                                    t_lastInstructionSent = t_now;
                                } else {
                                    uint8_t cmd[2] = {R_STOP, 100};
                                    write (fd, cmd, 2);
                                    cmd[0] = R_GRABBERSCLOSE;
                                    write (fd, cmd, 2);
                                    rstate = R_CLOSEDWITHBALL;
                                    robotnearball = true;
                                    t_lastInstructionSent = t_now; 
                                }
                                break;
                            case R_CLOSEDWITHBALL:
                                printf("CLOSED WITH BALL. angle: %2.2f\n", detection.robots_yellow(0).orientation());
                                if (detection.robots_yellow(0).orientation() > 0.2 || detection.robots_yellow(0).orientation() < -0.2){
                                    if (t_now - t_lastInstructionSent > 1 && t_now - t_lastTurnInstruction > 2){
                                        if (detection.robots_yellow(0).orientation() < 0){
                                            uint8_t cmd[2] = {R_TURNLEFT, detection.robots_yellow(0).orientation()*-27.8};
                                            write(fd, cmd, 2);
                                        } else {
                                            uint8_t cmd[2] = {R_TURNRIGHT, detection.robots_yellow(0).orientation()*27.8};
                                            write(fd, cmd, 2);
                                        }
                                        
                                        t_lastInstructionSent = t_now;
                                        t_lastTurnInstruction = t_now;
                                    }
                                } else if (t_now - t_lastInstructionSent > 1){
                                    /*if (abs(detection.robots_yellow(0).x()) > 500){
                                        if (detection.robots_yellow(0).x() < 0){
                                            uint8_t cmd[2] = {R_STRAFERIGHT, 180};
                                            write(fd, cmd, 2);
                                        } else {
                                            uint8_t cmd[2] = {R_STRAFELEFT, 180};
                                            write(fd, cmd, 2);
                                        }
                                    } else {*/
                                        uint8_t cmd[2] = {R_GRABBERSOPEN, 0};
                                        write(fd, cmd, 2);
                                        rstate = R_OPENWITHBALL;
                                    //}
                                    t_lastInstructionSent = t_now;
                                }    
                                break;
                            case R_OPENWITHBALL:
                                if (t_now - t_lastInstructionSent > 1){
                                    uint8_t cmd[2] = {R_KICK, 100};
                                    write(fd, cmd, 2);
                                    rstate = R_JUSTKICKED;
                                    t_lastInstructionSent = t_now;
                                }
                                break;
                            case R_JUSTKICKED:
                                if (t_now - t_lastInstructionSent > 1){
                                    uint8_t cmd[2] = {R_GRABBERSCLOSE, 100};
                                    write(fd, cmd, 2);
                                    rstate = R_CLOSEDNOBALL;
                                    t_lastInstructionSent = t_now;
                                }
                                break;
                            case R_CLOSINGGRABBERS:
                                if (t_now - t_lastInstructionSent > 1){
                                    rstate = R_CLOSEDNOBALL;
                                    t_lastInstructionSent = t_now;
                                }
                                break;
                        }
                    }
                
                    

                } else if (t_now - t_lastInstructionSent > 2) {
                    uint8_t cmd[2] = {R_STOP, 100};
                    write (fd, cmd, 2);                    
                }


            }

            //see if packet contains geometry data:
            if (packet.has_geometry()) {
                const SSL_GeometryData & geom = packet.geometry();
                printf("-[Geometry Data]-------\n");

                const SSL_GeometryFieldSize & field = geom.field();
                printf("Field Dimensions:\n");
                printf("  -line_width=%d (mm)\n",field.line_width());
                printf("  -field_length=%d (mm)\n",field.field_length());
                printf("  -field_width=%d (mm)\n",field.field_width());
                printf("  -boundary_width=%d (mm)\n",field.boundary_width());
                printf("  -referee_width=%d (mm)\n",field.referee_width());
                printf("  -goal_width=%d (mm)\n",field.goal_width());
                printf("  -goal_depth=%d (mm)\n",field.goal_depth());
                printf("  -goal_wall_width=%d (mm)\n",field.goal_wall_width());
                printf("  -center_circle_radius=%d (mm)\n",field.center_circle_radius());
                printf("  -defense_radius=%d (mm)\n",field.defense_radius());
                printf("  -defense_stretch=%d (mm)\n",field.defense_stretch());
                printf("  -free_kick_from_defense_dist=%d (mm)\n",field.free_kick_from_defense_dist());
                printf("  -penalty_spot_from_field_line_dist=%d (mm)\n",field.penalty_spot_from_field_line_dist());
                printf("  -penalty_line_from_spot_dist=%d (mm)\n",field.penalty_line_from_spot_dist());

                int calib_n = geom.calib_size();
                for (int i=0; i< calib_n; i++) {
                    const SSL_GeometryCameraCalibration & calib = geom.calib(i);
                    printf("Camera Geometry for Camera ID %d:\n", calib.camera_id());
                    printf("  -focal_length=%.2f\n",calib.focal_length());
                    printf("  -principal_point_x=%.2f\n",calib.principal_point_x());
                    printf("  -principal_point_y=%.2f\n",calib.principal_point_y());
                    printf("  -distortion=%.2f\n",calib.distortion());
                    printf("  -q0=%.2f\n",calib.q0());
                    printf("  -q1=%.2f\n",calib.q1());
                    printf("  -q2=%.2f\n",calib.q2());
                    printf("  -q3=%.2f\n",calib.q3());
                    printf("  -tx=%.2f\n",calib.tx());
                    printf("  -ty=%.2f\n",calib.ty());
                    printf("  -tz=%.2f\n",calib.tz());

                    if (calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() && calib.has_derived_camera_world_tz()) {
                      printf("  -derived_camera_world_tx=%.f\n",calib.derived_camera_world_tx());
                      printf("  -derived_camera_world_ty=%.f\n",calib.derived_camera_world_ty());
                      printf("  -derived_camera_world_tz=%.f\n",calib.derived_camera_world_tz());
                    }

                }
            }
        }
    }

    return 0;
}
