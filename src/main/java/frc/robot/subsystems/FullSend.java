// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FullSendConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FullSend extends SubsystemBase {
  /** Creates a new FullSend. */
  private final KrakenX60 fullSendMotor = new KrakenX60(FullSendConstants.MAIN_MOTOR_ID);

  // TODO: Tune and set defaults
  private final PidTuner fullSendPidTuner = new PidTuner("/FullSend/", 0.0, 0.0, 0.0, 0.0, 0.0);

  public FullSend() {}

  public Command setFullSendVoltage(Supplier<Double> voltage) {
    return run(
        () -> {
          fullSendMotor.setVoltage(voltage.get());
          ;
        });
  }

  public Command setFullSendRPM(Supplier<Double> rpm) {
    return run(
        () -> {
          fullSendMotor.setVelocity(rpm.get(), RPM);
        });
  }

  public Command manualFullSendRPM(Supplier<Boolean> reverse) {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/FullSend/Target RPM", 0.0);
    return setFullSendRPM(() -> (reverse.get() ? rpm.get() : -rpm.get()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (fullSendPidTuner.updated()) fullSendMotor.updateFromTuner(fullSendPidTuner);
  }
}
