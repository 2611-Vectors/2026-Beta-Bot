// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.hardware;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorKit.tuners.PidFfTuner;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class KrakenX60 extends SubsystemBase {
    // Generic motor values
    protected final TalonFX kMotor;
    protected final TalonFXConfigurator kConfigurator;
    protected final MotorOutputConfigs kOutputConfigs;
    protected final CurrentLimitsConfigs kCurrentConfigs;

    // Logging values
    protected PidFfTuner tuner;
    protected String logBaseDir;
    protected List<logType> enabledLogs;

    // Sim values
    protected final TalonFXSimState kSimState;
    protected final DCMotorSim kSimModel;
    protected final DCMotor kGearboxSim;
    protected double dt, lt = 0;

    // Controllers
    protected final VelocityVoltage kVelocityControl;
    protected final Slot0Configs kSlot0Configs;

    /** Creates a new KrakenX60. */
    public KrakenX60(int ID) {
        kMotor = new TalonFX(ID);
        kConfigurator = kMotor.getConfigurator();

        kSlot0Configs = new Slot0Configs();
        kConfigurator.refresh(kSlot0Configs);

        kOutputConfigs = new MotorOutputConfigs();
        kConfigurator.refresh(kOutputConfigs);

        kCurrentConfigs = new CurrentLimitsConfigs();
        kConfigurator.refresh(kCurrentConfigs);

        kVelocityControl = new VelocityVoltage(0).withSlot(0).withVelocity(0);

        logBaseDir = String.format("KrakenX60-%d", ID);
        enabledLogs = new LinkedList<>();

        if (RobotBase.isSimulation()) {
            kSimState = kMotor.getSimState();
            kSimState.Orientation = ChassisReference.Clockwise_Positive;
            kSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

            kGearboxSim = getMotorSim();
            kSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(kGearboxSim, 0.001, 1), kGearboxSim);
        } else {
            kSimState = null;
            kGearboxSim = null;
            kSimModel = null;
        }
    }

    protected DCMotor getMotorSim() {
        return DCMotor.getKrakenX60(1);
    }

    public void setEnabledLoggers(logType... enabledLoggers) {
        enabledLogs = new LinkedList<>();
        for (logType logger : enabledLoggers) {
            enabledLogs.add(logger);
        }
    }

    public void setEnabledLoggers(String basePath, logType... enabledLoggers) {
        logBaseDir = (basePath == "") ? logBaseDir : basePath;
        enabledLogs = new LinkedList<>();
        for (logType logger : enabledLoggers) {
            enabledLogs.add(logger);
        }
    }

    public void attachTuner(PidFfTuner tuner) {
        this.tuner = tuner;
    }

    public Command setPower(Supplier<Double> power) {
        return run(() -> kMotor.set(power.get())).handleInterrupt(() -> kMotor.set(0.0));
    }

    public Command setVoltage(Supplier<Double> volts) {
        return run(() -> kMotor.setVoltage(volts.get())).handleInterrupt(() -> kMotor.setVoltage(0.0));
    }

    public Command setVelocity(Supplier<Double> vel, AngularVelocityUnit unit) {
        return run(() -> kMotor.setControl(
                        kVelocityControl.withVelocity(RotationsPerSecond.convertFrom(vel.get(), unit))))
                .handleInterrupt(() -> {
                    kVelocityControl.withVelocity(0);
                    kMotor.setVoltage(0.0);
                });
    }

    public void setInverted(InvertedValue direction) {
        kOutputConfigs.withInverted(direction);
        kConfigurator.apply(kOutputConfigs);
    }

    public void setBrakeMode(NeutralModeValue mode) {
        kOutputConfigs.withNeutralMode(mode);
        kConfigurator.apply(kOutputConfigs);
    }

    public void addFollower(KrakenX60 follower, MotorAlignmentValue motorAlignment) {
        Follower m_followerRequest = new Follower(kMotor.getDeviceID(), motorAlignment);
        follower.kMotor.setControl(m_followerRequest);
    }

    public void setSupplyCurrentLimit(double maxAmps, double minAmps, double seconds) {
        kCurrentConfigs
                .withSupplyCurrentLimitEnable(minAmps > 0 && maxAmps > 0)
                .withSupplyCurrentLimit(maxAmps)
                .withSupplyCurrentLowerLimit(minAmps)
                .withSupplyCurrentLowerTime(seconds);
        kConfigurator.apply(kCurrentConfigs);
    }

    public void setStatorCurrentLimit(double amps) {
        kCurrentConfigs.withStatorCurrentLimitEnable(amps > 0).withStatorCurrentLimit(amps);
        kConfigurator.apply(kCurrentConfigs);
    }

    public double getVelocity(AngularVelocityUnit unit) {
        return unit.convertFrom(kMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    public double getTargetVelocity(AngularVelocityUnit unit) {
        return unit.convertFrom(kVelocityControl.Velocity, RotationsPerSecond);
    }

    public void setPID(double kP, double kI, double kD) {
        kSlot0Configs.withKP(kP).withKI(kI).withKD(kD);
        kConfigurator.apply(kSlot0Configs);
    }

    public void setFF(double kS, double kV) {
        kSlot0Configs.withKS(kS).withKV(kV);
        kConfigurator.apply(kSlot0Configs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (tuner != null) if (tuner.updateCtreSlot(kSlot0Configs)) kConfigurator.apply(kSlot0Configs);
        for (logType logger : enabledLogs) {
            switch (logger) {
                case CURRENT:
                    Logger.recordOutput(
                            String.format("%s/Stator Current", logBaseDir),
                            kMotor.getStatorCurrent().getValueAsDouble());
                    Logger.recordOutput(
                            String.format("%s/Supply Current", logBaseDir),
                            kMotor.getSupplyCurrent().getValueAsDouble());
                    break;
                case VELOCITY:
                    Logger.recordOutput(
                            String.format("%s/Current Velocity", logBaseDir),
                            RPM.convertFrom(kMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                    Logger.recordOutput(
                            String.format("%s/Target Velocity", logBaseDir),
                            RPM.convertFrom(kVelocityControl.Velocity, RotationsPerSecond));
                    break;
                case TEMPERATURE:
                    Logger.recordOutput(
                            String.format("%s/Temperature (C)", logBaseDir),
                            kMotor.getDeviceTemp().getValueAsDouble());
                    break;
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update Delta Time (dt)
        double ct = Timer.getFPGATimestamp();
        dt = ct - lt;
        lt = ct;

        // Update motor sim
        kSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        var MotorVoltage = kSimState.getMotorVoltageMeasure();
        kSimModel.setInputVoltage(MotorVoltage.in(Volts));
        kSimModel.update(dt < 0.001 ? 0.02 : dt);

        kSimState.setRawRotorPosition(kSimModel.getAngularPosition());
        kSimState.setRotorVelocity(kSimModel.getAngularVelocity());
    }

    public enum logType {
        CURRENT,
        VELOCITY,
        TEMPERATURE
    }
}
