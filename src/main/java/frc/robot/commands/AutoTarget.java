// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoMath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTarget extends ParallelCommandGroup {
  /** Creates a new AutoShooterDistance. */
  public AutoTarget(Drive m_Drive, Shooter m_Shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        m_Shooter.setShooterRPM(
            () ->
                AutoMath.getShooterSpeedFromDistance(AutoMath.getDistanceToHub(m_Drive.getPose()))),
        DriveCommands.joystickDriveAtAngle(
            m_Drive, () -> 0.0, () -> 0.0, () -> AutoMath.getRobotAngleToHub(m_Drive.getPose())));
  }
}
