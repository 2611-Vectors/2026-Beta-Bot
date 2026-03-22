// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.VectorKit.hardware;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class LinearActuator {
    private final Servo kActuator;
    private double kMin = 0.0, kMax = 1.0;

    public LinearActuator(ActuatorType type, int channel) {
        kActuator = new Servo(channel);
        switch (type) {
            case WCP_0408:
                kActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
                break;
        }
    }

    public LinearActuator(ActuatorType type, int channel, double min, double max) {
        this(type, channel);
        kMin = min;
        kMax = max;
    }

    public void set(double pos) {
        if (pos > kMax) pos = kMax;
        else if (pos < kMin) pos = kMin;
        kActuator.set(pos);
    }

    public enum ActuatorType {
        WCP_0408
    }
}
