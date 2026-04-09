// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoMath;

public class BumpCheckpoint extends SequentialCommandGroup {
    PathConstraints constraints =
            new PathConstraints(PATHFIND_VEL, PATHFIND_ACCEL, PATHFIND_ANGULAR_VEL, PATHFIND_ANGULAR_ACCEL);

    public BumpCheckpoint(Drive m_Drive) {
        Rotation2d bumpRot;
        if (AutoMath.onLeftSide(m_Drive.getPose())) 
            bumpRot = new Rotation2d(Units.degreesToRadians(AutoMath.flipAngle(45)));
        else bumpRot = new Rotation2d(Units.degreesToRadians(45));

        Pose2d bumpPoint = AutoMath.flipLR(m_Drive.getPose(), new Pose2d(RIGHT_BUMP_POS, bumpRot));

        addCommands(AutoBuilder.pathfindToPose(bumpPoint, constraints, VEL_OVER_BUMP));
    }
}
