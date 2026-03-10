package frc.robot.VectorKit.hardware;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Random;

public class AbsoluteEncoder extends DutyCycleEncoder implements Subsystem {
    private final DutyCycleEncoderSim m_sim;

    private SourceFeeder kSourceFeeder;
    private double kSimState, kOffset, kGearRatio, kMin, kMax;

    private boolean kNoiseEnabled = false;
    private Random kNoise = new Random();

    private boolean kIsReversed = false;

    public AbsoluteEncoder(int channel, double offset) {
        super(channel);

        kMin = 0.0;
        kMax = 360.0;

        kOffset = (offset + kMin) / kMax;

        if (Utils.isSimulation()) {
            m_sim = new DutyCycleEncoderSim(channel);
            m_sim.set(0.0);
            kNoiseEnabled = true;
        } else {
            m_sim = null;
        }
    }

    public void attachSource(SourceFeeder source, int sourceGearTeeth, int encoderGearTeeth) {
        kSourceFeeder = source;
        kGearRatio = (double) sourceGearTeeth / (double) encoderGearTeeth;
    }

    public void attachSource(SourceFeeder source, double gearRatio) {
        kSourceFeeder = source;
        kGearRatio = gearRatio;
    }

    public void setRange(double min, double max) {
        kOffset = (((kOffset * kMax) - kMin) + min) / max;
        kMin = min;
        kMax = max;
    }

    public void setReversed(boolean reversed) {
        kIsReversed = reversed;
    }

    public double getRaw() {
        if (Utils.isSimulation()) return getSimRaw();
        return super.get();
    }

    // TODO: Use simulated state as a sanity check
    @Override
    public double get() {
        if (Utils.isSimulation()) return getSim();
        double degrees = getRaw() - kOffset;
        if (degrees < 0.0) degrees += 1.0;
        if (degrees > 1.0) degrees -= 1.0;
        if (kIsReversed) degrees = 1.0 - degrees;
        return (degrees * kMax + kMin);
    }

    public double get(double offset) {
        if (Utils.isSimulation()) return getSim(offset);
        double degrees = getRaw() - offset;
        if (degrees < 0.0) degrees += 1.0;
        if (degrees > 1.0) degrees -= 1.0;
        if (kIsReversed) degrees = 1.0 - degrees;
        return (degrees * kMax + kMin);
    }

    private double getSimRaw() {
        return kSimState;
    }

    private double getSim() {
        double degrees = getSimRaw() - kOffset;
        if (degrees < 0.0) degrees += 1.0;
        if (degrees > 1.0) degrees -= 1.0;
        if (kIsReversed) degrees = 1.0 - degrees;
        return (degrees * kMax + kMin);
    }

    private double getSim(double offset) {
        double degrees = getSimRaw() - offset;
        if (degrees < 0.0) degrees += 1.0;
        if (degrees > 1.0) degrees -= 1.0;
        if (kIsReversed) degrees = 1.0 - degrees;
        return (degrees * kMax + kMin);
    }

    @Override
    public void periodic() {
        if (kSourceFeeder != null) {
            double out = (kSourceFeeder.get() * kGearRatio) % 1.0;
            if (kNoiseEnabled) out += kNoise.nextDouble() * (kNoise.nextBoolean() ? -1e-3 : 1e-3);

            if (Utils.isSimulation()) m_sim.set(out);
            kSimState = out;
        }
    }

    @FunctionalInterface
    public static interface SourceFeeder {
        public double get();
    }
}
