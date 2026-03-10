package frc.robot.VectorKit.hardware;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class KrakenX60 extends SubsystemBase {
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    private final VelocityVoltage kVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private final CurrentLimitsConfigs talonCurrentConfigs = new CurrentLimitsConfigs();
    public final TalonFX kMotor;
    private final TalonFXConfigurator kConfig;

    private String currentLogDir, rpmLogDir = "";

    private PidTuner kPidTuner;

    public KrakenX60(int ID) {
        kMotor = new TalonFX(ID);
        kConfig = kMotor.getConfigurator();
    }

    public void enableCurrentLogging(String basePath) {
        currentLogDir = basePath;
    }

    public void enableRPMLogging(String basePath) {
        rpmLogDir = basePath;
    }

    public void attachTuner(PidTuner tuner) {
        kPidTuner = tuner;
    }

    @Override
    public void periodic() {
        if (currentLogDir != "") {
            Logger.recordOutput(
                    String.format("%s/StatorCurrent", currentLogDir),
                    kMotor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput(
                    String.format("%s/SupplyCurrent", currentLogDir),
                    kMotor.getSupplyCurrent().getValueAsDouble());
        }

        if (rpmLogDir != "") {
            Logger.recordOutput(String.format("%s/Current RPM", rpmLogDir), getVelocity(RPM));
            Logger.recordOutput(String.format("%s/Target RPM", rpmLogDir), getTargetVelocity(RPM));
        }

        if (kPidTuner != null) {
            slot0Configs.kP = kPidTuner.getP();
            slot0Configs.kI = kPidTuner.getI();
            slot0Configs.kD = kPidTuner.getD();
            slot0Configs.kV = kPidTuner.getV();
            slot0Configs.kS = kPidTuner.getS();

            if (kPidTuner.updated()) kConfig.apply(slot0Configs);
        }
    }

    public void setSupplyCurrentLimit(double maxAmps, double minAmps, double seconds) {
        talonCurrentConfigs.withSupplyCurrentLimitEnable(minAmps > 0 && maxAmps > 0);
        talonCurrentConfigs.withSupplyCurrentLimit(maxAmps);
        talonCurrentConfigs.withSupplyCurrentLowerLimit(minAmps);
        talonCurrentConfigs.withSupplyCurrentLowerTime(seconds);

        talonConfigs.withCurrentLimits(talonCurrentConfigs);
        kConfig.apply(talonConfigs);
    }

    public void setStatorCurrentLimit(double amps) {
        talonCurrentConfigs.withStatorCurrentLimitEnable(amps > 0);
        talonCurrentConfigs.withStatorCurrentLimit(amps);

        talonConfigs.withCurrentLimits(talonCurrentConfigs);
        kConfig.apply(talonConfigs);
    }

    public double getVelocity(AngularVelocityUnit unit) {
        return unit.convertFrom(kMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    public double getTargetVelocity(AngularVelocityUnit unit) {
        return unit.convertFrom(kVelocityControl.Velocity, RotationsPerSecond);
    }

    public double getPosition() {
        return kMotor.getPosition().getValueAsDouble();
    }

    private void setVelocity(double vel, AngularVelocityUnit unit) {
        kMotor.feed();
        kMotor.setControl(kVelocityControl.withVelocity(RotationsPerSecond.convertFrom(vel, unit)));
    }

    public Command setVelocity(Supplier<Double> vel, AngularVelocityUnit unit) {
        return run(() -> setVelocity(vel.get(), unit)).handleInterrupt(() -> kMotor.setVoltage(0.0));
    }

    public Command setVoltage(Supplier<Double> volts) {
        return run(() -> kMotor.setVoltage(volts.get())).handleInterrupt(() -> kMotor.setVoltage(0.0));
    }

    public Command setPower(Supplier<Double> power) {
        return run(() -> kMotor.set(power.get())).handleInterrupt(() -> kMotor.setVoltage(0.0));
    }

    public void setInverted(InvertedValue direction) {
        motorOutputConfigs.Inverted = direction;
        kConfig.apply(motorOutputConfigs);
    }

    public void setBrakeMode(NeutralModeValue mode) {
        motorOutputConfigs.NeutralMode = mode;
        kConfig.apply(motorOutputConfigs);
    }

    public void setFollower(KrakenX60 follower, MotorAlignmentValue motorAlignment) {
        Follower m_followerRequest = new Follower(kMotor.getDeviceID(), motorAlignment);
        follower.kMotor.setControl(m_followerRequest);
    }

    public void setPID(double kP, double kI, double kD) {
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        kConfig.apply(slot0Configs);
    }

    public void setFF(double kS, double kV) {
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        kConfig.apply(slot0Configs);
    }
}
