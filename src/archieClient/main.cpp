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
#include <netinet/in.h>
#include <netdb.h>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#define POLARITY    1
#define KICKOFFSET  50
#define OPPONENTCLEARANCE 120
#define KICKTHRESH   10
#define POSTHRESH    30
#define ANGLETHRESH 1.0
#define PITCHLENGTH 550     // actually have the pitch length

#define STEPTIME    12      // time, in deciseconds, to take for a step
#define TURNTIME    10      // time, in deciseconds, to take for a step of a turn on the spot

#define PORT    1569

#define CMD_HELLO   0
#define CMD_MOVEKNEE    1
#define CMD_MOVEHIP     2
#define CMD_LEANSAGITTAL    3
#define CMD_LEANSIDEWAYS    4
#define CMD_STEP    5
#define CMD_WALK    6
#define CMD_EYES    7
#define CMD_KICK    8
#define CMD_MOVEJOINTS  9
#define CMD_LIFTLEG 10
#define CMD_LOWERLEG 11
#define CMD_CELEBRATE 12
#define CMD_HIPTOBESQUARE 13

#define CMD_LEFT    0
#define CMD_RIGHT   1

#define CMD_FORWARD     0
#define CMD_BACKWARD    1

#define CMD_POSITIVE    0
#define CMD_NEGATIVE    1

// socket stuff: see http://www.linuxhowtos.org/data/6/client.c
int openSocket(int port, char hostname[]){
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        printf("ERROR opening socket");
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(port);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        printf("ERROR connecting");
    return sockfd;
}

int sendCmd(char cmdString[], int cmdLength){
    int n;
    int sockfd = openSocket(PORT, "archie2.local");
    n = write(sockfd,cmdString,cmdLength);
    if (n < 0) 
         printf("ERROR writing to socket");
    close(sockfd);
    return 1;
}

int cmdWalk(uint8_t numsteps, int steplength, int turn, uint8_t steptime){
    uint8_t walkDir = CMD_FORWARD;
    if (steplength < 0){
        walkDir = CMD_BACKWARD;
        steplength *= -1;
    }
    if (steplength > 100)
        steplength = 100;
    uint8_t turnDir = CMD_RIGHT;
    if (turn < 0){
        turnDir = CMD_LEFT;
        turn *= -1;
    }
    if (turn > 40)
        turn = 40;
    char cmd[7];
    cmd[0] = CMD_WALK;
    cmd[1] = numsteps;
    cmd[2] = walkDir;
    cmd[3] = steplength;
    cmd[4] = turnDir;
    cmd[5] = turn;
    cmd[6] = steptime;

    sendCmd(cmd, 7);
}

int cmdKick(uint8_t foot){
    if (foot != CMD_LEFT)
        foot = CMD_RIGHT;
    char cmd[2];
    cmd[0] = CMD_KICK;
    cmd[1] = foot;
    sendCmd(cmd,2);
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


int calculateTarget(SSL_DetectionFrame &detection, float &targetX, float &targetY, float &targetAngle){
    int robots_yellow_n =  detection.robots_yellow_size();

    float distx = detection.balls(0).x() - detection.robots_blue(0).x();
    float disty = detection.balls(0).y() - detection.robots_blue(0).y();
    float ballAngle = atan2(disty, distx) - detection.robots_blue(0).orientation();
    float ballDist = sqrt(distx*distx + disty*disty);

    float oppBallDist;
    if (robots_yellow_n > 0){
        float oppBallDistX = detection.balls(0).x() - detection.robots_yellow(0).x();
        float oppBallDistY = detection.balls(0).y() - detection.robots_yellow(0).y();
        oppBallDist = sqrt(oppBallDistX*oppBallDistX + oppBallDistY*oppBallDistY);
    }
    if (ballDist < oppBallDist || !robots_yellow_n){
        printf("ball is closer to me\t");
        // ball is closer to me, or there is no opponent, so go to ball
        if ((detection.balls(0).x()-detection.robots_blue(0).x())*POLARITY > 0){
            // ball is towards opponent's goal
            targetX = detection.balls(0).x() - KICKOFFSET*POLARITY;
            targetY = detection.balls(0).y();
            targetAngle = 1.57 -1.57 * POLARITY;
            printf("ball is towards opponent's goal\n");
        } else {
            targetX = detection.balls(0).x();
            targetY = detection.balls(0).y() - KICKOFFSET*POLARITY;
            targetAngle = 1.57*POLARITY;
            printf("ball is towards my goal\n");
        }
    } else {
        // opponent is closer to ball. try to defend or whatever
        printf("opponent is closer to ball\t");
        if ((detection.balls(0).x()-detection.robots_blue(0).x())*POLARITY > 0){
            // ball is towards opponent's goal
            printf("ball is towards opponent's goal\n");
            targetX = min(detection.robots_blue(0).x()*POLARITY, (detection.robots_yellow(0).x()*POLARITY)-OPPONENTCLEARANCE)*POLARITY;
            targetY = detection.balls(0).y()*(0-(targetX+PITCHLENGTH*POLARITY)/(detection.balls(0).x()+PITCHLENGTH*POLARITY));
            targetAngle = POLARITY*1.57;
        } else {
            // ball is towards our goal. oh dear.
            printf("ball is towards my goal\t");
            if (abs(detection.robots_blue(0).y()-detection.robots_yellow(0).y()) > OPPONENTCLEARANCE){
                // robots are not aligned on the long axis, so try to get past the opponent
                printf("moving backwards\n");
                targetX = detection.robots_yellow(0).x()-OPPONENTCLEARANCE*POLARITY;
                targetY = detection.robots_blue(0).y();
                targetAngle = -1.57 * POLARITY;
            } else {
                // robots are close on the long axis, so move to the side
                printf("moving sideways\n");
                targetX = detection.robots_blue(0).x();
                targetY = abs(detection.robots_blue(0).y())+OPPONENTCLEARANCE;
                if (detection.robots_blue(0).y() < 0)
                    targetY *= -1;
                targetAngle = -1.57*POLARITY;           // this shouldn't really matter, as we should replan before we try to achieve this angle
            }
        }
    }
    return 1;
}

float walkToTarget(float robotX, float robotY, float robotAngle, float targetX, float targetY, float targetAngle, float ballDist, float ballAngle){
    float distx = targetX - robotX;
    float disty = targetY - robotY;
    float angleToPos = atan2(disty, distx) - robotAngle;
    float targetDist = sqrt(distx*distx + disty*disty);
    printf("robot angle: %.2f\t target angle: %.2f\n", robotAngle, targetAngle);
    if (angleToPos > 3.14){
        angleToPos -= 6.28;
    } else if (angleToPos < -3.14){
        angleToPos += 6.28;
    }
    if (ballDist < KICKTHRESH && abs(ballAngle) < ANGLETHRESH){
        // SHOULD PROBABLY RETHINK THIS. MIGHT RESULT IN OWN GOALS
        if (ballAngle > 0){
            cmdKick(CMD_LEFT);
            return 2.5;
        } else {
            cmdKick(CMD_RIGHT);
            return 2.5;
        }
    } 
    if (targetDist > POSTHRESH){
        // need to walk to target
        if (abs(angleToPos) > 0.5){
            float turn = 40;
            if (angleToPos < 0) turn *= -1;
            cmdWalk(2,0,turn,TURNTIME);
            return TURNTIME*0.2;
        } 
        cmdWalk(1,50,angleToPos*10, STEPTIME);
        return STEPTIME/10;
    }
    float angleDiff = targetAngle - robotAngle;
    if (angleDiff > 3.14) angleDiff -= 6.28;
    if (angleDiff < -3.14) angleDiff += 6.28;
    if (abs(angleDiff) > ANGLETHRESH){
        float turn = 40;
        if (angleDiff < 0) turn *= -1;
        cmdWalk(2,0,turn, TURNTIME);
        return TURNTIME*0.2;
    }
    // NOTHING TO DO! AWAIT INSTRUCTIONS
    return 0;
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    cmdWalk(1, 40, 0, 15);


    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    enum rstates {R_CLOSEDNOBALL, R_OPENNOBALL, R_CLOSEDWITHBALL, R_OPENWITHBALL, R_JUSTKICKED, R_CLOSINGGRABBERS};
    bool robotnearball = false;
    rstates rstate = R_CLOSEDNOBALL;

    double t_lastInstructionSent = 0;
    double t_lastTurnInstruction = 0; 
    double t_next = 0, t_instr = 0;
    while(true) {
        if (client.receive(packet)) {
            //printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();
/*
                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %7.3fms\n",(detection.t_sent()-detection.t_capture())*1000.0);
                printf("Network Latency (assuming synched system clock) %7.3fms\n",(t_now-detection.t_sent())*1000.0);
               printf("Total Latency   (assuming synched system clock) %7.3fms\n",(t_now-detection.t_capture())*1000.0);
               */
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
/*
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
*/
                float ballAngle;
                float ballDist;
                if ((robotnearball || balls_n > 0) && robots_blue_n > 0){
                    //printf("robopos: %2.2f, %2.2f, %2.2f\n", detection.robots_blue(0).x(), detection.robots_blue(0).y(), detection.robots_blue(0).orientation());
                    if (balls_n > 0){
                    //printf("ballpos: %2.2f, %2.2f\n", detection.balls(0).x(), detection.balls(0).y());
                        float distx = detection.balls(0).x() - detection.robots_blue(0).x();
                        float disty = detection.balls(0).y() - detection.robots_blue(0).y();
                        ballAngle = atan2(disty, distx) - detection.robots_blue(0).orientation();
                        ballDist = sqrt(distx*distx + disty*disty);
                        if (ballAngle > 3.14){
                            ballAngle -= 6.28;
                        } else if (ballAngle < -3.14){
                            ballAngle += 6.28;
                        }

                        if (ballDist > 500){
                            robotnearball = false;
                            //if (rstate == R_CLOSEDWITHBALL) rstate = R_CLOSEDNOBALL;
                            //if (rstate == R_OPENWITHBALL) rstate = R_OPENNOBALL;
                        }
                        //printf("ballToRobot: %2.2f, %2.2f, %2.2f, %2.2f\n", distx, disty, ballAngle, ballDist);
                    }
                    if (t_now - t_next > 0){
                        // time for a new instruction
                        float targetX, targetY, targetAngle;
                        calculateTarget(detection, targetX, targetY, targetAngle);
                        t_instr = walkToTarget(detection.robots_blue(0).x(), detection.robots_blue(0).y(), detection.robots_blue(0).orientation(), targetX, targetY, targetAngle, ballDist, ballAngle);
                        t_next = t_now + t_instr + 0.1;
/*
                                if (ballAngle > 0.5 && ballAngle < 3.14){
                                    if (t_now - t_lastTurnInstruction > 2.0){
                                        cmdWalk(2,0,30,10);
                                        t_lastTurnInstruction = t_now;
                                        t_lastInstructionSent = t_now;
                                        t_next = t_now + 2.1;
                                    }
                                } else if (ballAngle < -0.5){
                                    if (t_now - t_lastTurnInstruction > 2.0){
                                        cmdWalk(2,0,-30,10);
                                        t_lastTurnInstruction = t_now;
                                        t_lastInstructionSent = t_now;
                                        t_next = t_now + 2.1;
                                    }
                                } else if (ballDist > 10){
                                    cmdWalk(1,50,ballAngle*10,12);
                                    t_lastInstructionSent = t_now;
                                    t_next = t_now + 1.2;
                                } else {
                                    if (ballAngle < 0){
                                        cmdKick(CMD_LEFT);
                                    } else {
                                        cmdKick(CMD_RIGHT);
                                    }
                                    t_lastInstructionSent = t_now;
                                    t_next = t_now + 2.5;
                                }
*/

                    }
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
