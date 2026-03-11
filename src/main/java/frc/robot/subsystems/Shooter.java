// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FullSendConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.hardware.WCP_0408;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
    /** Creates a new Shooter. */
    private final KrakenX60 leftMotor = new KrakenX60(ShooterConstants.LEFT_MOTOR_ID);

    private final KrakenX60 rightMotor = new KrakenX60(ShooterConstants.RIGHT_MOTOR_ID);
    private final KrakenX60 fullSendMotor = new KrakenX60(FullSendConstants.MAIN_MOTOR_ID);

    private final PidTuner shooterPidTuner = new PidTuner("Shooter", 0.2, 0.02, 0.0, 0.0, 0.12);
    private final PidTuner fullSendPidTuner = new PidTuner("FullSend", 0.1, 0.02, 0.0, 0.0, 0.11);

    private final WCP_0408 leftLinearActuator = new WCP_0408(
            ShooterConstants.LEFT_LINEAR_ACTUATOR_ID,
            ShooterConstants.LINEAR_ACTUATOR_MINIMUM,
            ShooterConstants.LINEAR_ACTUATOR_MAXIMUM);

    private final WCP_0408 rightLinearActuator = new WCP_0408(
            ShooterConstants.RIGHT_LINEAR_ACTUATOR_ID,
            ShooterConstants.LINEAR_ACTUATOR_MINIMUM,
            ShooterConstants.LINEAR_ACTUATOR_MAXIMUM);

    public Shooter() {
        leftMotor.setFollower(rightMotor, MotorAlignmentValue.Opposed);
        leftMotor.setInverted(InvertedValue.CounterClockwise_Positive);
        leftMotor.enableCurrentLogging("Shooter/Left");
        leftMotor.enableRPMLogging("Shooter/Left");
        leftMotor.attachTuner(shooterPidTuner);

        rightMotor.enableCurrentLogging("Shooter/Right");
        rightMotor.enableRPMLogging("Shooter/Right");

        fullSendMotor.setInverted(InvertedValue.Clockwise_Positive);
        fullSendMotor.attachTuner(fullSendPidTuner);
        fullSendMotor.enableCurrentLogging("FullSend");
        fullSendMotor.enableRPMLogging("FullSend");

        leftLinearActuator.addFollower(rightLinearActuator);
    }

    public Command setFullSendRPM(Supplier<Double> rpm) {
        return fullSendMotor.setVelocity(rpm, () -> RPM);
    }

    public Command manualFullSendRPM(Supplier<Boolean> reverse) {
        LoggedNetworkNumber rpm = new LoggedNetworkNumber("FullSend/Manual Target RPM", 5000.0);
        return setFullSendRPM(() -> (reverse.get() ? -rpm.get() : rpm.get()));
    }

    public Command setShooterRPM(Supplier<Double> rpm) {
        return leftMotor.setVelocity(rpm, () -> RPM);
    }

    public Command manualShooterRPM() {
        LoggedNetworkNumber rpm = new LoggedNetworkNumber("Shooter/Manual Target RPM", ShooterConstants.MANUAL_RPM);
        return leftMotor.setVelocity(() -> rpm.get(), () -> RPM);
    }

    public Boolean isAtSpeed() {
        return leftMotor.getVelocity(RPM) >= leftMotor.getTargetVelocity(RPM);
    }

    public Command setHoodPos(Supplier<Double> pos) {
        return leftLinearActuator.setHoodPos(pos);
    }

    public Command manualHoodPos() {
        LoggedNetworkNumber pos = new LoggedNetworkNumber("Shooter/Manual Hood Position", 0.65);
        return leftLinearActuator.setHoodPos(() -> pos.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Shooter/On", leftMotor.getTargetVelocity(RPM) > 0.0);
        Logger.recordOutput(
                "Shooter/Manual Mode", Math.abs(leftMotor.getTargetVelocity(RPM) - ShooterConstants.MANUAL_RPM) < 1.0);
    }
}
