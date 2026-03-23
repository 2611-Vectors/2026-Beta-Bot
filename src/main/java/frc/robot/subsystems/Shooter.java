// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorKit.hardware.KrakenX60;
import frc.VectorKit.hardware.KrakenX60.logType;
import frc.VectorKit.tuners.PidFfTuner;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
    private final KrakenX60 leftMotor = new KrakenX60(ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    private final KrakenX60 leftMotor2 = new KrakenX60(ShooterConstants.LEFT_SHOOTER_MOTOR2_ID);
    private final KrakenX60 rightMotor = new KrakenX60(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    private final KrakenX60 rightMotor2 = new KrakenX60(ShooterConstants.RIGHT_SHOOTER_MOTOR2_ID);

    private final KrakenX60 fullSendMotor = new KrakenX60(ShooterConstants.FULLSEND_MOTOR_ID);

    private final PidFfTuner shooterPidTuner = new PidFfTuner("Shooter", 0.6, 0.0, 0.0, 0.0, 0.12);
    private final PidFfTuner fullSendPidTuner = new PidFfTuner("FullSend", 0.6, 0.0, 0.0, 0.0, 0.13);

    private final LoggedNetworkNumber manualShooterRpm = new LoggedNetworkNumber("Shooter/Target RPM", 2900.0);
    private final LoggedNetworkNumber manualFullSendRpm = new LoggedNetworkNumber("FullSend/Target RPM", 1000.0);

    /** Creates a new Shooter. */
    public Shooter() {
        leftMotor.addFollower(rightMotor2, MotorAlignmentValue.Opposed);
        leftMotor.addFollower(leftMotor2, MotorAlignmentValue.Aligned);
        leftMotor.addFollower(rightMotor, MotorAlignmentValue.Opposed);

        leftMotor.setEnabledLoggers("Shooter", logType.CURRENT, logType.VELOCITY);
        leftMotor.setInverted(InvertedValue.CounterClockwise_Positive);
        leftMotor.attachTuner(shooterPidTuner);

        leftMotor.setStatorCurrentLimit(40);
        leftMotor2.setStatorCurrentLimit(40);
        rightMotor.setStatorCurrentLimit(40);
        rightMotor2.setStatorCurrentLimit(40);

        fullSendMotor.setEnabledLoggers("FullSend", logType.CURRENT, logType.VELOCITY);
        fullSendMotor.setInverted(InvertedValue.Clockwise_Positive);
        fullSendMotor.attachTuner(fullSendPidTuner);

        fullSendMotor.setStatorCurrentLimit(50);
    }

    public Command setShooterRPM(Supplier<Double> rpm) {
        Logger.recordOutput("Shooter/On", false);
        return Commands.parallel(leftMotor.setVelocity(rpm, RPM), Commands.runOnce(() -> {
            manualShooterRpm.set(rpm.get());
            Logger.recordOutput("Shooter/On", true);
        }));
    }

    public Command manualShooterRPM() {
        return leftMotor.setVelocity(() -> manualShooterRpm.get(), RPM);
    }

    public Boolean isAtSpeed() {
        return Math.abs(leftMotor.getVelocity(RPM) - manualShooterRpm.get()) < 100.0;
    }

    public Command setFullSendRPM(Supplier<Double> rpm) {
        return fullSendMotor.setVelocity(() -> (rpm.get() / ShooterConstants.FULLSEND_GEAR_RATIO), RPM);
    }

    public Command manualFullSendRPM(Supplier<Boolean> reverse) {
        return setFullSendRPM(() -> (reverse.get() ? -manualFullSendRpm.get() : manualFullSendRpm.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Shooter/At Speed", isAtSpeed());
        Logger.recordOutput("Shooter/Manual Mode", Math.abs(manualShooterRpm.get() - 3050.0) <= 2);
    }
}
