// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.hardware.WCP_0408;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final KrakenX60 leftMotor = new KrakenX60(ShooterConstants.LEFT_MOTOR_ID);

  private final KrakenX60 rightMotor = new KrakenX60(ShooterConstants.RIGHT_MOTOR_ID);

  private final WCP_0408 leftLinearActuator =
      new WCP_0408(ShooterConstants.LEFT_LINEAR_ACTUATOR_ID);
  private final WCP_0408 rightLinearActuator =
      new WCP_0408(ShooterConstants.RIGHT_LINEAR_ACTUATOR_ID);

  // TODO: Tune and set defaults
  PidTuner shooterPidTuner = new PidTuner("/Shooter/", 0.0, 0.0, 0.0, 0.0, 0.0);

  // Max RPM / Seconds to max RPM
  SlewRateLimiter RPMSlew = new SlewRateLimiter(6000.0 / 3.0);

  public Shooter() {
    leftMotor.setFollower(rightMotor, MotorAlignmentValue.Opposed);
    leftMotor.setInverted(InvertedValue.CounterClockwise_Positive);
  }

  public Command setShooterRPM(Supplier<Double> rpm) {
    return run(
        () -> {
          leftMotor.setVelocity(rpm.get(), RPM);
        });
  }

  public Command manualShooterRPM() {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Shooter/Target RPM", 0.0);
    return new SequentialCommandGroup(
            runOnce(
                () -> {
                  RPMSlew.reset(leftMotor.getVelocity().getValueAsDouble() * 60);
                }),
            run(
                () -> {
                  double filtered = RPMSlew.calculate(rpm.get());
                  leftMotor.setVelocity(filtered, RPM);
                }))
        .handleInterrupt(
            () -> {
              leftMotor.set(0.0);
            });
  }

  public Command setHoodPos(Supplier<Double> pos) {
    return run(
        () -> {
          leftLinearActuator.set(pos.get());
          rightLinearActuator.set(pos.get());
        });
  }

  public Command manualHoodPos() {
    LoggedNetworkNumber pos = new LoggedNetworkNumber("/Shooter/Hood Position", 0.0);
    return setHoodPos(() -> pos.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shooterPidTuner.updated()) leftMotor.updateFromTuner(shooterPidTuner);
  }
}
