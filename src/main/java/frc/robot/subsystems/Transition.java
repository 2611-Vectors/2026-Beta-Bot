// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorKit.hardware.KrakenX60;
import frc.VectorKit.hardware.KrakenX60.logType;
import frc.VectorKit.tuners.PidFfTuner;
import frc.robot.Constants.TransitionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Transition extends SubsystemBase {
    private final KrakenX60 transitionMotor = new KrakenX60(TransitionConstants.MOTOR_ID);

    private final PidFfTuner lowerTransitionPidTuner = new PidFfTuner("Transition", 0.1, 0.02, 0.0, 0.0, 0.14);

    private final LoggedNetworkNumber manualRpm = new LoggedNetworkNumber("Transition/Target RPM", 2000.0);

    /** Creates a new Transition. */
    public Transition() {
        transitionMotor.setInverted(InvertedValue.Clockwise_Positive);
        transitionMotor.setStatorCurrentLimit(60);

        transitionMotor.attachTuner(lowerTransitionPidTuner);
        transitionMotor.setEnabledLoggers("Transition", logType.CURRENT, logType.VELOCITY);
    }

    public Command setTransitionVoltage(Supplier<Double> voltage) {
        return transitionMotor.setVoltage(voltage);
    }

    public Command setTransitionRPM(Supplier<Double> rpm) {
        return transitionMotor.setVelocity(() -> (rpm.get() / TransitionConstants.GEAR_RATIO), RPM);
    }

    public Command manualTransitionRPM(Supplier<Boolean> reverse) {
        return setTransitionRPM(() -> (reverse.get() ? -manualRpm.get() : manualRpm.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
