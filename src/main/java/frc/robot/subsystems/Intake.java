// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    private final KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);

    // TODO: Tune and set defaults
    private final PidTuner intakePidTuner = new PidTuner("/Intake/", 0.1, 0.02, 0.0, 0.0, 0.13);

    public Intake() {
        intakeMotor.setInverted(InvertedValue.Clockwise_Positive);
        intakeMotor.setStatorCurrentLimit(80);
    }

    public Command setIntakeVoltage(Supplier<Double> voltage) {
        return runOnce(() -> {
                    intakeMotor.setVoltage(voltage.get());
                })
                .handleInterrupt(() -> {
                    intakeMotor.setVoltage(0.0);
                });
    }

    public Command setIntakeRPM(Supplier<Double> rpm) {
        return run(() -> {
                    intakeMotor.setVelocity(rpm.get() / IntakeConstants.INTAKE_GEAR_RATIO, RPM);
                })
                .handleInterrupt(() -> {
                    intakeMotor.setVoltage(0.0);
                });
    }

    public Command manualIntakeRPM(Supplier<Boolean> reverse) {
        LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Intake/Target RPM", 3000.0);
        LoggedNetworkNumber revrpm = new LoggedNetworkNumber("/Intake/Target Reverse RPM", 500.0);
        return setIntakeRPM(() -> (reverse.get() ? -revrpm.get() : rpm.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (intakePidTuner.updated()) intakeMotor.updateFromTuner(intakePidTuner);

        Logger.recordOutput("Intake/Current RPM (Motor)", intakeMotor.getRPM());
        Logger.recordOutput("Intake/Current RPM (Output)", intakeMotor.getRPM() * IntakeConstants.INTAKE_GEAR_RATIO);

        intakeMotor.logCurrents("Intake");
    }
}
