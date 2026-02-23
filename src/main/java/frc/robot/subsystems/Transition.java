// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitionConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.pidTuner;

public class Transition extends SubsystemBase {
  /** Creates a new Transition. */
  KrakenX60 lowerMotor = new KrakenX60(TransitionConstants.LOWER_MOTOR_ID);

  KrakenX60 upperLeftMotor = new KrakenX60(TransitionConstants.UPPER_LEFT_MOTOR_ID);
  KrakenX60 upperRightMotor = new KrakenX60(TransitionConstants.UPPER_RIGHT_MOTOR_ID);

  // TODO: Tune and set defaults
  pidTuner lowerTransitionpPidTuner = new pidTuner("/Transition/Lower/", 0.0, 0.0, 0.0, 0.0, 0.0);
  pidTuner upperTransitionPidTuner = new pidTuner("/Transition/Upper/", 0.0, 0.0, 0.0, 0.0, 0.0);

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
