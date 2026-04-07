// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.AbsoluteEncoder;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.TunablePidController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Pivot extends SubsystemBase {
    /** Creates a new Pivot. */
    private final KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

    private final AbsoluteEncoder pivotEncoder =
            new AbsoluteEncoder(IntakeConstants.PIVOT_ENCODER_ID, IntakeConstants.PIVOT_ENCODER_OFFSET);

    private final TunablePidController pivotController =
            new TunablePidController("/Intake/Pivot/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public Pivot() {
        pivotEncoder.setInverted(true);
        pivotMotor.setBrakeMode(NeutralModeValue.Brake);
        pivotMotor.setInverted(InvertedValue.Clockwise_Positive);
    }

    public Command manualPivotPosition() {
        LoggedNetworkNumber pos = new LoggedNetworkNumber("/Intake/Pivot/Target Position", 0.0);
        return setPivotPosition(() -> pos.get());
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

    public Command dumbIntakeOut() {
        return run(() -> {
                    pivotMotor.setVoltage(10.0);
                })
                .until(() -> intakeIsOut())
                .andThen(() -> {
                    pivotMotor.setVoltage(0.0);
                })
                .handleInterrupt(() -> {
                    pivotMotor.setVoltage(0.0);
                });
    }

    public boolean intakeIsOut() {
        return pivotEncoder.get() >= IntakeConstants.PIVOT_OUT_ANGLE;
    }

    public boolean intakeCanRun() {
        return pivotEncoder.get() >= IntakeConstants.PIVOT_RUN_ANGLE;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Pivot/Current Angle", pivotEncoder.get());
        Logger.recordOutput("Intake/Pivot/New Offset", (pivotEncoder.getRaw() * 360.0) - 8.0);
        pivotController.update();
        Logger.recordOutput(
                "Intake/Pivot/Current RPM (Output)", pivotMotor.getRPM() / IntakeConstants.PIVOT_GEAR_RATIO);
        Logger.recordOutput("Intake/Pivot/Current RPM (Motor)", pivotMotor.getRPM());
        pivotMotor.logCurrents("Intake/Pivot");
    }
}
