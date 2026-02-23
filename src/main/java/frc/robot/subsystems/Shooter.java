// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.hardware.WCP_0408;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  KrakenX60 leftMotor = new KrakenX60(ShooterConstants.LEFT_MOTOR_ID);
  KrakenX60 rightMotor = new KrakenX60(ShooterConstants.RIGHT_MOTOR_ID);

  WCP_0408 leftLinearActuator = new WCP_0408(ShooterConstants.LEFT_LINEAR_ACTUATOR_ID);
  WCP_0408 rightLinearActuator = new WCP_0408(ShooterConstants.RIGHT_LINEAR_ACTUATOR_ID);

  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
