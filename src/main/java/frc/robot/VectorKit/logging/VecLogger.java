// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.logging;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class VecLogger {
    private static final NetworkTableInstance nt4 = NetworkTableInstance.getDefault();
    private static final NetworkTable VecTable = nt4.getTable("VectorKit");
    private static final Map<String, GenericPublisher> publishers = new HashMap<>();

    public static void logData(String key, double value) {
        //     if
        // (!VecTable.getTopic(key).getType().getValueStr().contentEquals(NetworkTableType.getStringFromObject(value)))
        //         return;
        //     if (!publishers.containsKey(key))
        //         publishers.put(key, VecTable.getDoubleTopic(key).publish());
        //     publishers.get(key).
    }
}
