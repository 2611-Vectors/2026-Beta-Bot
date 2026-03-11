// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitionConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Transition extends SubsystemBase {
    /** Creates a new Transition. */
    private final KrakenX60 lowerMotor = new KrakenX60(TransitionConstants.LOWER_MOTOR_ID);

    private final KrakenX60 upperLeftMotor = new KrakenX60(TransitionConstants.UPPER_LEFT_MOTOR_ID);
    private final KrakenX60 upperRightMotor = new KrakenX60(TransitionConstants.UPPER_RIGHT_MOTOR_ID);

    private final PidTuner lowerTransitionPidTuner = new PidTuner("Transition/Lower", 0.1, 0.02, 0.0, 0.0, 0.14);
    private final PidTuner upperTransitionPidTuner = new PidTuner("Transition/Upper", 0.1, 0.02, 0.0, 0.0, 0.11);

    public Transition() {
        upperLeftMotor.setFollower(upperRightMotor, MotorAlignmentValue.Opposed);
        upperLeftMotor.setInverted(InvertedValue.CounterClockwise_Positive);
        upperLeftMotor.attachTuner(upperTransitionPidTuner);

        lowerMotor.setInverted(InvertedValue.Clockwise_Positive);
        lowerMotor.attachTuner(lowerTransitionPidTuner);

        upperLeftMotor.enableCurrentLogging("Transition/UpperLeft");
        upperLeftMotor.enableRPMLogging("Transition/UpperLeft");

        upperRightMotor.enableCurrentLogging("Transition/UpperRight");
        upperRightMotor.enableCurrentLogging("Transition/UpperRight");

        lowerMotor.enableCurrentLogging("Transition/Lower");
        lowerMotor.enableRPMLogging("Transition/Lower");
    }

    public Command setTransitionVoltage(Supplier<Double> upperVoltage, Supplier<Double> lowerVoltage) {
        return new ParallelCommandGroup(lowerMotor.setVoltage(lowerVoltage), upperLeftMotor.setVoltage(upperVoltage));
    }

    public Command setTransitionRPM(Supplier<Double> upperRPM, Supplier<Double> lowerRPM) {
        return new ParallelCommandGroup(
                lowerMotor.setVelocity(() -> (lowerRPM.get() / TransitionConstants.LOWER_GEAR_RATIO), () -> RPM),
                upperLeftMotor.setVelocity(() -> (upperRPM.get() / TransitionConstants.UPPER_GEAR_RATIO), () -> RPM));
    }

    public Command manualTransitionRPM(Supplier<Boolean> reverse) {
        LoggedNetworkNumber lowerRPM = new LoggedNetworkNumber("Transition/Lower/Target RPM", 500.0);
        LoggedNetworkNumber upperRPM = new LoggedNetworkNumber("Transition/Upper/Target RPM", 1000.0);
        return setTransitionRPM(
                () -> (reverse.get() ? -upperRPM.get() : upperRPM.get()),
                () -> (reverse.get() ? -lowerRPM.get() : lowerRPM.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
