// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.hardware;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Random;
import java.util.function.Supplier;

/** Add your docs here. */
public class RevThroughboreEncoder extends SubsystemBase {
    // Generic encoder values
    private final DutyCycleEncoder kAbsoluteEncoder;
    private final DutyCycleEncoderSim kAbsoluteEncoderSim;
    private final Encoder kRelativeEncoder;
    private final EncoderSim kRelativeEncoderSim;
    private final EncoderMode kMode;
    private boolean kInverted;
    private double kOffset;

    // Sim values
    private Supplier<Double> kSourcePosition;
    private double kGearRatio;
    private boolean kNoiseEnabled;
    private final Random kNoise;

    public RevThroughboreEncoder(EncoderMode mode, int... pins) {
        // Set default sim values
        kNoise = new Random();
        kNoiseEnabled = false;
        kGearRatio = 1.0;

        // Set default real values
        kInverted = false;

        // Set mode & create encoder object(s)
        kMode = mode;
        switch (kMode) {
            case RELATIVE:
                kAbsoluteEncoder = null;
                kRelativeEncoder = new Encoder(pins[0], pins[1]);
                break;
            case ABSOLUTE:
                kAbsoluteEncoder = new DutyCycleEncoder(pins[0]);
                kRelativeEncoder = null;
                break;
            default:
                kAbsoluteEncoder = null;
                kRelativeEncoder = null;
                break;
        }

        if (Utils.isSimulation()) {
            switch (kMode) {
                case RELATIVE:
                    kAbsoluteEncoderSim = null;
                    kRelativeEncoderSim = new EncoderSim(kRelativeEncoder);
                    break;
                case ABSOLUTE:
                    kRelativeEncoderSim = null;
                    kAbsoluteEncoderSim = new DutyCycleEncoderSim(kAbsoluteEncoder);
                    break;
                default:
                    kAbsoluteEncoderSim = null;
                    kRelativeEncoderSim = null;
                    break;
            }
        } else {
            kAbsoluteEncoderSim = null;
            kRelativeEncoderSim = null;
        }
    }

    public RevThroughboreEncoder(EncoderMode mode, Rotation2d offset, int... pins) {
        this(mode, pins);
        kOffset = offset.getDegrees() / 360.0;
    }

    /**
     * @param mode ABSOLUTE or RELATIVE
     * @param offset in Degrees
     * @param pins
     */
    public RevThroughboreEncoder(EncoderMode mode, double offset, int... pins) {
        this(mode, pins);
        kOffset = offset / 360.0;
    }

    public void attachSource(Supplier<Double> source, int sourceGearTeeth, int encoderGearTeeth) {
        kSourcePosition = source;
        kGearRatio = (double) sourceGearTeeth / (double) encoderGearTeeth;
    }

    public void attachSource(Supplier<Double> source, double sourceGearTeeth, double encoderGearTeeth) {
        kSourcePosition = source;
        kGearRatio = sourceGearTeeth / encoderGearTeeth;
    }

    public void setInverted(boolean inverted) {
        kInverted = inverted;
    }

    public double getRaw() {
        return kMode == EncoderMode.ABSOLUTE ? kAbsoluteEncoder.get() : kRelativeEncoder.getRaw();
    }

    public double get() {
        switch (kMode) {
            case ABSOLUTE:
                double degrees = getRaw() - kOffset;
                if (degrees < 0.0) degrees += 1.0;
                if (degrees > 1.0) degrees -= 1.0;
                if (kInverted) degrees = 1.0 - degrees;
                return degrees * 360.0;
            case RELATIVE:
                return kRelativeEncoder.getDistance();
            default:
                return 0.0;
        }
    }

    public double get(double min, double max) {
        switch (kMode) {
            case ABSOLUTE:
                double degrees = getRaw() - kOffset;
                if (degrees < 0.0) degrees += 1.0;
                if (degrees > 1.0) degrees -= 1.0;
                if (kInverted) degrees = 1.0 - degrees;
                return (degrees * max + min);
            case RELATIVE:
                double pos = kRelativeEncoder.getDistance();
                if (pos < min) pos = min;
                else if (pos > max) pos = max;
                return pos;
            default:
                return 0.0;
        }
    }

    public double getSource() {
        if (kSourcePosition != null) return kSourcePosition.get();
        else
            switch (kMode) {
                case ABSOLUTE:
                    return kAbsoluteEncoder.get() / kGearRatio;
                case RELATIVE:
                    return kRelativeEncoder.getDistance() / kGearRatio;
                default:
                    return 0.0;
            }
    }

    @Override
    public void simulationPeriodic() {
        switch (kMode) {
            case ABSOLUTE:
                double out = (kSourcePosition.get() * kGearRatio) % 1.0;
                if (kNoiseEnabled) out += kNoise.nextDouble() * (kNoise.nextBoolean() ? 1e-3 : -1e-3);
                kAbsoluteEncoderSim.set(out);
                break;
            case RELATIVE:
                kRelativeEncoderSim.setCount((int) (kSourcePosition.get() * kGearRatio));
                break;
        }
    }

    public enum EncoderMode {
        ABSOLUTE,
        RELATIVE
    }
}
