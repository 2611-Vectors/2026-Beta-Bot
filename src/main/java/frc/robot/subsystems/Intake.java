// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorKit.hardware.KrakenX60;
import frc.VectorKit.hardware.KrakenX60.logType;
import frc.VectorKit.hardware.RevThroughboreEncoder;
import frc.VectorKit.hardware.RevThroughboreEncoder.EncoderMode;
import frc.VectorKit.tuners.PidFfTuner;
import frc.robot.Constants.IntakeConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
    private final KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);
    private final KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

    private final RevThroughboreEncoder pivotEncoder =
            new RevThroughboreEncoder(EncoderMode.ABSOLUTE, IntakeConstants.PIVOT_ENCODER_ID);

    private final PidFfTuner intakePidTuner = new PidFfTuner("Intake", 0.1, 0.02, 0.0, 0.0, 0.13);

    private final LoggedNetworkNumber manualRpm = new LoggedNetworkNumber("Intake/Target RPM", 3000.0);
    private final LoggedNetworkNumber manualRevRpm = new LoggedNetworkNumber("Intake/Target Reverse RPM", 500.0);

    /** Creates a new Intake. */
    public Intake() {
        intakeMotor.setEnabledLoggers("Intake", logType.CURRENT, logType.VELOCITY);
        intakeMotor.setInverted(InvertedValue.Clockwise_Positive);
        intakeMotor.setStatorCurrentLimit(80);
        intakeMotor.attachTuner(intakePidTuner);

        pivotMotor.setEnabledLoggers("Intake", logType.CURRENT, logType.VELOCITY);
        pivotMotor.setInverted(InvertedValue.Clockwise_Positive);
        pivotMotor.setBrakeMode(NeutralModeValue.Coast);

        pivotEncoder.setInverted(true);
    }

    public Command setIntakeVoltage(Supplier<Double> voltage) {
        return intakeMotor.setVoltage(voltage);
    }

    public Command setIntakeRPM(Supplier<Double> rpm) {
        return intakeMotor.setVelocity(() -> (rpm.get() / IntakeConstants.INTAKE_GEAR_RATIO), RPM);
    }

    public Command setIntakeRPMSafe(Supplier<Double> rpm) {
        return intakeMotor
                .setVelocity(() -> (rpm.get() / IntakeConstants.INTAKE_GEAR_RATIO), RPM)
                .onlyWhile(() -> pivotEncoder.get() >= IntakeConstants.PIVOT_RUN_ANGLE);
    }

    public Command manualIntakeRPM(Supplier<Boolean> reverse) {
        return setIntakeRPM(() -> (reverse.get() ? -manualRevRpm.get() : manualRpm.get()));
    }

    public Command manualPivotVoltage(Supplier<Double> volts) {
        return pivotMotor.setVoltage(volts);
    }

    public Command dumbIntakeOut() {
        return pivotMotor.setVoltage(() -> 10.0).until(() -> pivotEncoder.get() >= IntakeConstants.PIVOT_OUT_ANGLE);
    }

    public Command holdIntakeOut() {
        return pivotMotor.setVoltage(() -> 5.0).onlyWhile(() -> pivotEncoder.get() >= IntakeConstants.PIVOT_OUT_ANGLE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Pivot/Current Angle", pivotEncoder.get());
        Logger.recordOutput("Intake/Pivot/New Offset", (pivotEncoder.getRaw() * 360.0) - 5.0);
    }
}
