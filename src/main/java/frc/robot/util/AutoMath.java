// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PathfindToStart;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoMath {
  public AutoMath() {}

  public static Rotation2d getRobotAngleToHub(Pose2d robotPose) {
    robotPose = PathfindToStart.flipRed(robotPose);
    double a = robotPose.getX() - FieldConstants.HUB_POSITION.getX();
    double b = robotPose.getY() - FieldConstants.HUB_POSITION.getY();

    return new Rotation2d(Math.atan(b / a) + Math.PI);
  }

  public static double getDistanceToHub(Pose2d robotPose) {
    robotPose = PathfindToStart.flipRed(robotPose);
    double a = robotPose.getX() - FieldConstants.HUB_POSITION.getX();
    double b = robotPose.getY() - FieldConstants.HUB_POSITION.getY();
    double c = Math.sqrt((a * a) + (b * b));

    Logger.recordOutput("Targeting/distToHub", c);
    return c;
  }

  public static double getShooterSpeedFromDistance(double dist) {
    double a =
        ShooterConstants.TIP_TO_RPM
            * Math.sqrt(
                (ShooterConstants.GRAVITATIONAL_CONSTANT * (dist * dist))
                    / (ShooterConstants.LAUNCH_ANGLE_COS
                        * (ShooterConstants.INITIAL_HEIGHT
                            + Math.tan(ShooterConstants.LAUNCH_ANGLE) * dist
                            - 5.9)));
    return a;
  }
}
