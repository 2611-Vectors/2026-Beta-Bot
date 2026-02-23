// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.pidTuner;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);

  KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

  // TODO: Make Tunable
  PIDController pivotController = new PIDController(0, 0, 0);

  // TODO: Tune and set defaults
  pidTuner intakePidTuner = new pidTuner("/Intake/", 0.0, 0.0, 0.0, 0.0, 0.0);

  // TODO: Add encoder things later

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakePidTuner.updated()) intakeMotor.updateFromTuner(intakePidTuner);
  }
}
