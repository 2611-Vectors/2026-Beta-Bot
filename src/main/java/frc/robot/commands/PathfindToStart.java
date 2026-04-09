// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.AutoMath;

public class PathfindToStart extends SequentialCommandGroup {
    public PathfindToStart(PathPlannerAuto selectedAuto) {
        Pose2d startPose = AutoMath.flipRed(selectedAuto.getStartingPose());
        PathConstraints constraints = new PathConstraints(3.0, 2.5, 90.0, 720.0);

        addCommands(AutoBuilder.pathfindToPose(startPose, constraints), selectedAuto);
    }
}
