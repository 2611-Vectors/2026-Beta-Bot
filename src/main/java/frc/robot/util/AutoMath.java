// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.PathfindToStart;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class AutoMath {
  public AutoMath() {}

  @AutoLogOutput(key = "Targeting/")
  public static double getDistanceToHub(Pose2d robotPose) {
    robotPose = PathfindToStart.flipRed(robotPose);
    double a = robotPose.getX() - FieldConstants.HUB_POSITION.getX();
    double b = robotPose.getY() - FieldConstants.HUB_POSITION.getY();
    return Math.sqrt((a * a) + (b * b));
  }
}
