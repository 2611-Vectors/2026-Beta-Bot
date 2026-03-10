// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.hardware;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class WCP_0408 extends SubsystemBase {
    // https://wcproducts.com/products/wcp-0408

    private final double kMin, kMax;
    private final Servo kServo;
    private final List<WCP_0408> linkedActuators = new LinkedList<>();

    public WCP_0408(int channel) {
        kServo = new Servo(channel);
        kServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        kMin = 0;
        kMax = 1;

        linkedActuators.add(this);
    }

    public WCP_0408(int channel, double min, double max) {
        kServo = new Servo(channel);
        kServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        kMin = min;
        kMax = max;

        linkedActuators.add(this);
    }

    public void addFollower(WCP_0408 follower) {
        linkedActuators.add(follower);
    }

    public void set(double pos) {
        kServo.set(pos);
    }

    public Command setHoodPos(Supplier<Double> pos) {
        return runOnce(() -> {
            double posActual = pos.get();
            if (posActual < kMin) posActual = kMin;
            if (posActual > kMax) posActual = kMax;

            for (WCP_0408 actuator : linkedActuators) {
                actuator.set(posActual);
            }
        });
    }
}
