// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.VectorKit.hardware.LoggedPDH;

/** Add your docs here. */
public class PDH extends LoggedPDH {
    public PDH() {
        super(1, ModuleType.kRev);
        addMech("Drivetrain", 0, 1, 3, 4, 15, 16, 17, 18, 19);
        addMech("Intake", 7, 13);
        addMech("Transition", 8);
        addMech("Shooter", 5, 6, 10, 11);
        addMech("FullSend", 14);
        addMech("Misc", 20, 21, 22);
    }
}
