// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitionConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;

public class Transition extends SubsystemBase {
  /** Creates a new Transition. */

  private final KrakenX60 lowerMotor = new KrakenX60(TransitionConstants.LOWER_MOTOR_ID);
  private final KrakenX60 upperLeftMotor = new KrakenX60(TransitionConstants.UPPER_LEFT_MOTOR_ID);
  private final KrakenX60 upperRightMotor = new KrakenX60(TransitionConstants.UPPER_RIGHT_MOTOR_ID);

  // TODO: Tune and set defaults
  private final PidTuner lowerTransitionpPidTuner = new PidTuner("/Transition/Lower/", 0.0, 0.0, 0.0, 0.0, 0.0);
  private final PidTuner upperTransitionPidTuner = new PidTuner("/Transition/Upper/", 0.0, 0.0, 0.0, 0.0, 0.0);

  public Transition() {
    upperLeftMotor.setFollower(upperRightMotor, MotorAlignmentValue.Opposed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (lowerTransitionpPidTuner.updated()) lowerMotor.updateFromTuner(lowerTransitionpPidTuner);
    if (upperTransitionPidTuner.updated()) upperLeftMotor.updateFromTuner(lowerTransitionpPidTuner);
  }
}
