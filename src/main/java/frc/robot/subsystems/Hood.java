// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.WCP_0408;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private final WCP_0408 leftLinearActuator =
      new WCP_0408(ShooterConstants.LEFT_LINEAR_ACTUATOR_ID);

  private final WCP_0408 rightLinearActuator =
      new WCP_0408(ShooterConstants.RIGHT_LINEAR_ACTUATOR_ID);

  public Hood() {}

  public Command setHoodPos(Supplier<Double> pos) {
    return run(
        () -> {
          double posActual = pos.get(); // 0.65 is our current minimum (2/26)
          if (posActual < ShooterConstants.LINEAR_ACTUATOR_MINIMUM)
            posActual = ShooterConstants.LINEAR_ACTUATOR_MINIMUM;
          if (posActual > ShooterConstants.LINEAR_ACTUATOR_MAXIMUM)
            posActual = ShooterConstants.LINEAR_ACTUATOR_MAXIMUM;

          leftLinearActuator.set(posActual);
          rightLinearActuator.set(posActual);
        });
  }

  public Command manualHoodPos() {
    LoggedNetworkNumber pos = new LoggedNetworkNumber("/Shooter/Hood Position", 0.65);
    return setHoodPos(() -> pos.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
