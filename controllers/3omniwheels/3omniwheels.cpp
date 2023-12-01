#include "webots/Motor.hpp"
#include "webots/Robot.hpp"
#include "webots/PositionSensor.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/GPS.hpp"
#include "iostream"
#include "Kinematic.h"
#include "webots/Supervisor.hpp"
#include "Motion.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace webots;
const int maxNumberCord = 10000;
const int refresh_factor = 2.0;

Motor *motor[4];
PositionSensor *enc[4];
InertialUnit *imu_device;

void setVel(motion_data &motor);

Motion *mot = new Motion();
Point2D nowPos(0, 0, 0);
int count = 0;

Supervisor *robot = new Supervisor();
Node *my_root = robot->getRoot();

static void create_line();

void drawArrow(cv::Mat &image, const cv::Point &pStart, const cv::Point &pEnd, const cv::Scalar &color, int thickness, int line_type, int shift, double tipLength)
{
  const double tipSize = cv::norm(pStart - pEnd) * tipLength; // Factor to normalize the arrow size

  cv::line(image, pStart, pEnd, color, thickness, line_type, shift);

  const double angle = std::atan2((double)pStart.y - pEnd.y, (double)pStart.x - pEnd.x);
  cv::Point p1, p2;

  p1.x = static_cast<int>(pEnd.x + tipSize * std::cos(angle + CV_PI / 4));
  p1.y = static_cast<int>(pEnd.y + tipSize * std::sin(angle + CV_PI / 4));
  cv::line(image, pEnd, p1, color, thickness, line_type, shift);

  p2.x = static_cast<int>(pEnd.x + tipSize * std::cos(angle - CV_PI / 4));
  p2.y = static_cast<int>(pEnd.y + tipSize * std::sin(angle - CV_PI / 4));
  cv::line(image, pEnd, p2, color, thickness, line_type, shift);
}

// Function to draw a V shape representing the heading on an OpenCV image
void drawHeadingV(cv::Mat &image, const cv::Point &center, double headingAngle, const cv::Scalar &color, int thickness, int line_type, int shift, double tipLength)
{
  const double tipSize = tipLength;

  const double angle1 = headingAngle + CV_PI / 6;
  const double angle2 = headingAngle - CV_PI / 6;

  cv::Point p1, p2;

  p1.x = static_cast<int>(center.x + tipSize * std::cos(angle1));
  p1.y = static_cast<int>(center.y + tipSize * std::sin(angle1));
  cv::line(image, center, p1, color, thickness, line_type, shift);

  p2.x = static_cast<int>(center.x + tipSize * std::cos(angle2));
  p2.y = static_cast<int>(center.y + tipSize * std::sin(angle2));
  cv::line(image, center, p2, color, thickness, line_type, shift);
}

void get_Trajectory(std::vector<Point2D> &path, Point2D &outputPID, Point2D &nowPos, motion_data &outInvers, double yaw)
{
  cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  // nowPos = RobotKinematic::getInstance()->getPos();

  float centerX = image.cols / 2.0;
  float centerY = image.rows / 2.0;
  // IC(circVec[count].x, circVec[count].y);
  cv::Point robotPose((nowPos.x * 100) + centerX, -(nowPos.y * 100) + centerY);

  for (auto &it : path)
  { //  cv::line(image, robotPose, cv::Point(it.x*100 + centerX , -(it.y*100) + centerY), cv::Scalar(0, 255, 0), 2);

    cv::circle(image, cv::Point2d(it.x * 100 + centerX, -(it.y * 100) + centerY), 5, cv::Scalar(255, 255, 0), -1);
  } // Draw the robot base (you can replace this with your robot's actual base)
  cv::circle(image, robotPose, 20, cv::Scalar(0, 0, 255), -1);
  cv::line(image, robotPose, cv::Point(path[count].x * 100 + centerX, -(path[count].y * 100) + centerY), cv::Scalar(0, 255, 0), 2);

  // drawHeadingV(image, robotPose, -nowPos.theta, cv::Scalar(255, 0, 0), 5, 8, 0, 20);
  drawHeadingV(image, robotPose, -yaw, cv::Scalar(255, 0, 0), 5, 8, 0, 20);

  double errorX = path[count].x - nowPos.x;
  double errorY = path[count].y - nowPos.y;

  double dist = sqrt(errorX * errorX + errorY * errorY);
  double errTheta = path[count].theta - (nowPos.theta * 180 / M_PI);

  if (errTheta > 180)
    errTheta -= 360;
  if (errTheta < -180)
    errTheta += 360;

  // IF using IMU as orientation
  mot->PositionAngularControl(errorX, errorY, errTheta, yaw, outputPID);

  // If using Odometry orientation
  // mot->PositionAngularControl(errorX, errorY, errTheta, nowPos.theta, outputPID);
  IC(outputPID.x,outputPID.y,outputPID.theta);
  Kinematic::getIstance()->InversKinematic(outputPID.x, outputPID.y, outputPID.theta, outInvers);
  IC(outInvers.w1, outInvers.w2 , outInvers.w3);
  if (dist < 0.07 && fabs(errTheta) < 3)
  {
    //OFF if we use circle Traj
    // mot->pid_position->reset();
    // mot->angular->reset();
    count++;
    IC("count");
  }
  if (count > path.size() - 1)
  {
    count = 0;
  }
  cv::imshow("Robot 3 omni visual", image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  motion_data outInvers;
  double dt = 0.0;
  Point2D outputPID(0, 0, 0);
  int stepTime = robot->getBasicTimeStep();
  stepTime *= refresh_factor;

  imu_device = robot->getInertialUnit("IMU");
  imu_device->enable(stepTime);

  GPS *gps_dev = robot->getGPS("gps");
  gps_dev->enable(stepTime);
  // Motor Initial
  char motorNames[4][8] = {"wheel1", "wheel2", "wheel3"};

  Node *target_line = robot->getFromDef("OMNI_WHEELS");
  Node *target_robot = robot->getFromDef("field");

  // SET the first Position our Robot
  Kinematic::getIstance()->setInitialPositon(-3.0, 0, 0); // Set Position x = 0, y = 0, theta = 0

  // Create Line
  create_line();

  Node *trail_line_set = robot->getFromDef("TRACK_3_LINE_SET");
  Field *coord_field = trail_line_set->getField("coord");

  Node *coordinate_node = coord_field->getSFNode();
  Field *pointField = coordinate_node->getField("point");
  Field *coord_index_field = trail_line_set->getField("coordIndex");

  int index = 0;
  bool first_step = true;
  // Track line end-----------

  // set Motor to webots
  for (int i = 0; i < 3; i++)
  {
    motor[i] = robot->getMotor(motorNames[i]);
    motor[i]->setPosition(INFINITY);
    motor[i]->setVelocity(0);
  }

  // Encoder Initial
  char encNames[4][8] = {"pw1", "pw2", "pw3"};

  for (int i = 0; i < 3; i++)
  {
    enc[i] = robot->getPositionSensor(encNames[i]);
    enc[i]->enable(stepTime);
  }

  std::vector<Point2D> cirTraj;
  for (int i = 0; i < 360; i += 10)
  {
    float x_ = 1 * cos((float)i * M_PI / 180)-3;
    float y_ = 1 * sin((float)i * M_PI / 180);
    cirTraj.push_back(Point2D(x_, y_, 0));
  }
  std::vector<Point2D> targetPos = {Point2D(1.5, 0, 0), Point2D(2, -1, 0), Point2D(0, -2, 0), Point2D(0, 2, 0)};
  Point2D gps_pos{0, 0, 0};

  while (robot->step(stepTime) != -1)
  {

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    // Get Current Translation
    double rbtime = robot->getTime();

    const double *robotTranslations = target_line->getPosition();

    // // add New Position
    pointField->setMFVec3f(index, robotTranslations);

    // update line track
    if (index > 0)
    {
      coord_index_field->setMFInt32(3 * (index - 1), index - 1);
      coord_index_field->setMFInt32(3 * (index - 1) + 1, index);
    }
    else if (index == 0 && first_step == false)
    {
      coord_index_field->setMFInt32(3 * (maxNumberCord - 1), 0);
      coord_index_field->setMFInt32(3 * (maxNumberCord - 1) + 1, (maxNumberCord - 1));
    }

    const double *orientation = imu_device->getRollPitchYaw();
    const double yaw = orientation[2];

    // GPS for testing
    const double *gps_raw = gps_dev->getValues();
    gps_pos.x = gps_raw[0];
    gps_pos.y = gps_raw[1];
    gps_pos.theta = yaw;

    nowPos = Kinematic::getIstance()->getPosition();
    IC(nowPos.x, nowPos.y, nowPos.theta);

    get_Trajectory(cirTraj, outputPID, nowPos, outInvers, yaw);

    setVel(outInvers);

    // Assign Enc val
    for (int i = 0; i < 3; i++)
    {
      Kinematic::getIstance()->encData[i] = enc[i]->getValue();
    }
    Kinematic::getIstance()->calcOdometry(yaw);

    // unset next indices
    coord_index_field->setMFInt32(3 * index, index);
    coord_index_field->setMFInt32(3 * index + 1, index);

    if (robot->step(robot->getBasicTimeStep()) == -1)
      break;
    first_step = false;
    index++;
    index = index % maxNumberCord;
  }

  robot->simulationQuit(EXIT_SUCCESS);
  delete robot;

  return 0;
}

void setVel(motion_data &outMotor)
{
  motor[0]->setVelocity(outMotor.w1);
  motor[1]->setVelocity(outMotor.w2);
  motor[2]->setVelocity(outMotor.w3);
}

static void create_line()
{
  Node *existing_line = robot->getFromDef("TRACK_3");
  // Node *myRobot = super->getFromDef("OMNI_WHEELS");
  // Field *translation_field = myRobot->getField("translation");

  if (existing_line)
    existing_line->remove();

  int i;
  std::string track_string = ""; // Initialize a big string which will contain the TRAIL node.
  // Create the TRAIL Shape.
  track_string += "DEF TRACK_3 Shape {\n";
  track_string += "  appearance Appearance {\n";
  track_string += "    material Material {\n";
  track_string += "      diffuseColor 0 0 0\n";
  track_string += "      emissiveColor 0 0 0\n";
  track_string += "    }\n";
  track_string += "  }\n";
  track_string += "  geometry DEF TRACK_3_LINE_SET IndexedLineSet {\n";
  track_string += "    coord Coordinate {\n";
  track_string += "      point [\n";
  for (i = 0; i < maxNumberCord; ++i)
    track_string += "      0 0 0\n";
  track_string += "      ]\n";
  track_string += "    }\n";
  track_string += "    coordIndex [\n";
  for (i = 0; i < maxNumberCord; ++i)
    track_string += "      0 0 -1\n";
  track_string += "    ]\n";
  track_string += "  }\n";
  track_string += "}\n";

  // Import TRAIL and append it as the world root nodes.

  Field *field_children = my_root->getField("children");

  field_children->importMFNodeFromString(-1, track_string);
}
