// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.HUB_POSITION;
import static frc.robot.Constants.RobotConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoMath;

public class CrossBumpToShoot extends SequentialCommandGroup {
    PathConstraints constraints =
            new PathConstraints(PATHFIND_VEL, PATHFIND_ACCEL, PATHFIND_ANGULAR_VEL, PATHFIND_ANGULAR_ACCEL);

    public CrossBumpToShoot(Drive m_Drive) {
        Rotation2d bumpRot;
        if (AutoMath.onLeftSide(m_Drive.getPose()))
            bumpRot = new Rotation2d(Units.degreesToRadians(AutoMath.flipAngle(BUMP_ROTATION)));
        else bumpRot = new Rotation2d(Units.degreesToRadians(BUMP_ROTATION));
        Pose2d bumpPoint = AutoMath.flipLR(m_Drive.getPose(), new Pose2d(RIGHT_BUMP_POS, bumpRot));
        Pose2d afterBumpPoint = AutoMath.flipLR(m_Drive.getPose(), new Pose2d(RIGHT_AFTER_BUMP_POS, bumpRot));

        Rotation2d targetAngle = AutoMath.getRobotAngleToTarget(
                new Pose2d(RIGHT_SHOOT_POS, new Rotation2d()), HUB_POSITION.toPose2d());
        Pose2d shootPoint = AutoMath.flipLR(m_Drive.getPose(), new Pose2d(RIGHT_SHOOT_POS, targetAngle));

        addCommands(
                AutoBuilder.pathfindToPose(bumpPoint, constraints, VEL_OVER_BUMP),
                AutoBuilder.pathfindToPose(afterBumpPoint, constraints, VEL_OVER_BUMP),
                AutoBuilder.pathfindToPose(shootPoint, constraints));
    }
}
