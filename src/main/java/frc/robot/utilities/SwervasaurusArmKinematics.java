// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

public class SwervasaurusArmKinematics {

  double armLengthCollapsed = 25.25;
  double gripperLength = 10.5;

  /*
    SwervasaurKinematics Class

    ## Constants
    kLengthRetract - Length of arm when fully retracted (From Shoulder pivot to wrist pivot)
    kLengthGripper - Length from wrist pivot to grip point on claw
  */

  /** Creates a new SwervasaurusArmKinematics. */
  public SwervasaurusArmKinematics() {}

  /*
    [x_boys,x_girls,theta_shoulder,theta_wrist_pitch] = toJoints(x_end,y_end,theta_end)

      L = sqrt((x_end-kLengthGripper*sin(theta_end))^2+(y_end-kLengthGripper*cos(theta_end))^2);
      theta_shoulder = atan2(x_end-kLengthGripper*sin(theta_end),y_end-kLengthGripper*cos(theta_end));
      theta_wrist_pitch = theta_shoulder-theta_end;
      x_boys = L/2;
      x_girls = L/2;
   */
  public double[] toJoints(double x_end, double y_end, double theta_end) {
    double sinTheta = gripperLength * Math.sin(Math.toRadians(theta_end));
    double cosTheta = gripperLength * Math.cos(Math.toRadians(theta_end));
    double length = Math.sqrt(Math.pow(x_end - sinTheta, 2) + Math.pow(y_end - cosTheta, 2));
    double theta_shoulder = Math.atan2(x_end - sinTheta, y_end - cosTheta);
    double theta_wrist = theta_shoulder - theta_end;
    double x_boys = length/2;
    double x_girls = length/2;
    return new double[]{x_boys, x_girls, theta_shoulder, theta_wrist};
  }

  /*
    [x_end,y_end,theta_end] = toPosition(x_boys,x_girls,theta_shoulder,theta_wrist_pitch)

      L = x_boys + x_girls
      x_end = L*sin(theta_shoulder)+kLengthGripper*sin(theta_shoulder-theta_wrist_pitch);
      y_end = L*cos(theta_shoulder)+kLengthGripper*cos(theta_shoulder-theta_wrist_pitch);
      theta_end = theta_shoulder-theta_wrist_pitch;
  */
  public double[] toPosition(double x_boys, double x_girls, double theta_shoulder, double theta_wrist) {
    double length = x_boys + x_girls;
    double x_end = (length * Math.sin(Math.toDegrees(theta_shoulder))) + (gripperLength * Math.sin(Math.toDegrees(theta_shoulder - theta_wrist)));
    double y_end = (length * Math.cos(Math.toDegrees(theta_shoulder))) + (gripperLength * Math.cos(Math.toDegrees(theta_shoulder - theta_wrist)));
    double theta_end = theta_shoulder - theta_wrist;
    return new double[]{x_end, y_end, theta_end};
  }
}
