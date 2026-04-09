// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.*;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class CrossBumpToShoot extends SequentialCommandGroup {
    PathConstraints constraints =
            new PathConstraints(PATHFIND_VEL, PATHFIND_ACCEL, PATHFIND_ANGULAR_VEL, PATHFIND_ANGULAR_ACCEL);

    public CrossBumpToShoot(Drive m_Drive) {
        addCommands(new BumpCheckpoint(m_Drive));
    }
}
