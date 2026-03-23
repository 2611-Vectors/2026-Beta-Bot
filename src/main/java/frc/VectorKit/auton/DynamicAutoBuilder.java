// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

/** Add your docs here. */
public class DynamicAutoBuilder {
    public static SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        for (String autoName : autoNames) {
            chooser.addOption(autoName, new PathfindToStart(new PathPlannerAuto(autoName)));
        }

        return chooser;
    }
}
