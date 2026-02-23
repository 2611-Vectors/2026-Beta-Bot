// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FullSendConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.pidTuner;

public class FullSend extends SubsystemBase {
  /** Creates a new FullSend. */

  KrakenX60 fullSendMotor = new KrakenX60(FullSendConstants.MAIN_MOTOR_ID);

  // TODO: Tune and set defaults
  pidTuner fullSendPidTuner = new pidTuner("/FullSend/", 0.0, 0.0, 0.0, 0.0, 0.0);

  public FullSend() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (fullSendPidTuner.updated()) fullSendMotor.updateFromTuner(fullSendPidTuner);
  }
}
