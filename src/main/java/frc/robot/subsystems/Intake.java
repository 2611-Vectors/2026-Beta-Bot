// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.AbsoluteEncoder;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import frc.robot.VectorKit.tuners.TunablePidController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    private final KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);

    private final KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

    private final AbsoluteEncoder pivotEncoder =
            new AbsoluteEncoder(IntakeConstants.PIVOT_ENCODER_ID, IntakeConstants.PIVOT_ENCODER_OFFSET);

    // TODO: Tune and set defaults
    private final PidTuner intakePidTuner = new PidTuner("/Intake/", 0.1, 0.02, 0.0, 0.0, 0.13);
    private final TunablePidController pivotController =
            new TunablePidController("/Intake/Pivot/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public Intake() {
        pivotEncoder.setInverted(true);
        pivotMotor.setBrakeMode(NeutralModeValue.Coast);
        pivotMotor.setInverted(InvertedValue.Clockwise_Positive);
        intakeMotor.setInverted(InvertedValue.Clockwise_Positive);
    }

    public Command setPivotVoltage(Supplier<Double> voltage) {
        return runOnce(() -> {
            pivotMotor.setVoltage(voltage.get());
        });
    }

    public Command setPivotPosition(Supplier<Double> position) {
        return setPivotVoltage(() -> pivotController.calculate(pivotEncoder.get(), position.get()));
    }

    public Command manualPivotVoltage() {
        LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Intake/Pivot/Voltage", 0.0);
        return setPivotVoltage(() -> voltage.get());
    }

    public Command manualPivotPosition() {
        LoggedNetworkNumber pos = new LoggedNetworkNumber("/Intake/Pivot/Target Position", 0.0);
        return setPivotPosition(() -> pos.get());
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

    public Command dumbIntakeOut() {
        return run(() -> {
                    pivotMotor.setVoltage(6.0);
                })
                .until(() -> pivotEncoder.get() >= IntakeConstants.PIVOT_OUT_ANGLE)
                .andThen(() -> {
                    pivotMotor.setVoltage(0.0);
                })
                .handleInterrupt(() -> {
                    pivotMotor.setVoltage(0.0);
                });
    }

    public Command manualIntakeRPM(Supplier<Boolean> reverse) {
        LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Intake/Target RPM", 2500.0);
        LoggedNetworkNumber revrpm = new LoggedNetworkNumber("/Intake/Target Reverse RPM", 500.0);
        return setIntakeRPM(() -> (reverse.get() ? -revrpm.get() : rpm.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Pivot/Current Angle", pivotEncoder.get());
        Logger.recordOutput("Intake/Pivot/New Offset", (pivotEncoder.getRaw() * 360.0) - 0.5);
        if (intakePidTuner.updated()) intakeMotor.updateFromTuner(intakePidTuner);
        pivotController.update();

        Logger.recordOutput("Intake/Pivot/Current RPM (Motor)", pivotMotor.getRPM());
        Logger.recordOutput(
                "Intake/Pivot/Current RPM (Output)", pivotMotor.getRPM() / IntakeConstants.PIVOT_GEAR_RATIO);
        Logger.recordOutput("Intake/Current RPM (Motor)", intakeMotor.getRPM());
        Logger.recordOutput("Intake/Current RPM (Output)", intakeMotor.getRPM() * IntakeConstants.INTAKE_GEAR_RATIO);

        intakeMotor.logCurrents("Intake");
        pivotMotor.logCurrents("Intake/Pivot");
    }
}
