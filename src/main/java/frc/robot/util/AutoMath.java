// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.ShooterConstants.GRAVITATIONAL_CONSTANT;
import static frc.robot.Constants.ShooterConstants.INITIAL_HEIGHT;
import static frc.robot.Constants.ShooterConstants.LAUNCH_ANGLE;
import static frc.robot.Constants.ShooterConstants.LAUNCH_ANGLE_COS;
import static frc.robot.Constants.ShooterConstants.TIP_TO_RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.PathfindToStart;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoMath {
    public AutoMath() {}

    public static Rotation2d getRobotAngleToTarget(Pose2d robotPose, Pose2d target) {
        robotPose = PathfindToStart.flipRed(robotPose);
        double a = robotPose.getX() - target.getX();
        double b = robotPose.getY() - target.getY();

        return new Rotation2d(Math.atan(b / a) + Math.PI);
    }

    public static double getDistanceToTarget(Pose2d robotPose, Pose2d target) {
        robotPose = PathfindToStart.flipRed(robotPose);
        double a = robotPose.getX() - target.getX();
        double b = robotPose.getY() - target.getY();
        double c = Math.sqrt((a * a) + (b * b));

        Logger.recordOutput("Targeting/distToHub", c);
        return c;
    }

    public static Pose2d translateTargetByChassisSpeeds(Pose2d robotPose, Pose2d target, ChassisSpeeds speeds) {
        // TODO: Get a variable for time (fuel in air), and multiply all velos by that variable
        double dist = getDistanceToTarget(robotPose, target);
        Pose2d rotVector = new Pose2d(dist, 0, new Rotation2d());
        rotVector.rotateAround(new Translation2d(0, 0), new Rotation2d(speeds.omegaRadiansPerSecond));

        Pose2d out = new Pose2d(
                target.getX() + Math.abs(speeds.vxMetersPerSecond) + rotVector.getX(),
                target.getY() + Math.abs(speeds.vyMetersPerSecond) + rotVector.getY(),
                target.getRotation());

        Logger.recordOutput("Targeting/Target", target);
        Logger.recordOutput("Targeting/Target New", out);
        return out;
    }

    public static double getShooterSpeedFromDistance(double dist) {
        double a = TIP_TO_RPM
                * Math.sqrt((GRAVITATIONAL_CONSTANT * (dist * dist))
                        / (LAUNCH_ANGLE_COS * (INITIAL_HEIGHT + Math.tan(LAUNCH_ANGLE) * dist - 5.9)));
        return a;
    }
}
