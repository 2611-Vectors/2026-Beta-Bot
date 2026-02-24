// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import frc.robot.VectorKit.tuners.TunablePidController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);
  private final KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER_ID);

  // TODO: Tune and set defaults
  private final PidTuner intakePidTuner = new PidTuner("/Intake/", 0.0, 0.0, 0.0, 0.0, 0.0);
  private final TunablePidController pivotController = new TunablePidController("/Intake/Pivot/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public Intake() {}

  public Command setPivotVoltage(Supplier<Double> voltage) {
    return runOnce(() -> {
      pivotMotor.setVoltage(voltage.get());
    });
  }

  public Command setPivotPosition(Supplier<Double> position) {
    return run(() -> {
      pivotMotor.setVoltage(pivotController.calculate(pivotEncoder.get(), position.get()));
    });
  }

  public Command setIntakeVoltage(Supplier<Double> voltage) {
    return runOnce(() -> {
      intakeMotor.setVoltage(voltage.get());
    });
  }

  public Command setIntakeRPM(Supplier<Double> rpm) {
    return run(() -> {
      intakeMotor.setVelocity(rpm.get(), RPM);
    });
  }

  public Command manualIntakeRPM() {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Intake/Target RPM", 0.0);
    return setIntakeRPM(() -> rpm.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakePidTuner.updated()) intakeMotor.updateFromTuner(intakePidTuner);
    pivotController.update();
  }
}
