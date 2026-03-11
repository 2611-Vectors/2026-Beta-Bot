// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.logging;

// import static frc.robot.VectorKit.logging.VecLogger.VecDataLog;
// import static frc.robot.VectorKit.logging.VecLogger.VecTable;

/** Add your docs here. */
public class Loggable {
    // private final String name;
    // private final String type;
    // private final Topic topic;
    // private final GenericPublisher pub;
    // private final GenericSubscriber sub;
    // private final DataLogEntry logEntry;
    // private Object lastValue;

    // public Loggable(String name, Object value) {
    //     this.name = name;
    //     type = NetworkTableType.getStringFromObject(value);
    //     topic = VecTable.getTopic(name);
    //     pub = topic.genericPublish(type);
    //     sub = topic.genericSubscribe(type);

    //     // if (NetworkTableType.getFromString(type).equals(NetworkTableType.kDouble))
    //     //     logEntry = new DoubleLogEntry(VecDataLog, name, type);
    //     // else if (NetworkTableType.getFromString(type).equals(NetworkTableType.kBoolean))
    //     //     logEntry = new BooleanLogEntry(VecDataLog, name, type);
    //     // else if (NetworkTableType.getFromString(type).equals(NetworkTableType.kInteger))
    //     //     logEntry = new IntegerLogEntry(VecDataLog, name, type);
    //     // else if (NetworkTableType.getFromString(type).equals(NetworkTableType.kString))
    //     //     logEntry = new StringLogEntry(VecDataLog, name, type);
    //     // else logEntry = null;

    //     set(value);
    // }

    // public void set(Object value) {
    //     if (NetworkTableType.getStringFromObject(value) == "")
    //         DriverStation.reportError(
    //                 String.format("[Error] VecKit: Cannot publish topic (%s) (Invalid Type: %s)", name, type), true);

    //     if (!pub.isValid())
    //         DriverStation.reportError(
    //                 String.format("[Error] VecKit: Cannot set an invalid publisher (%s)", name), true);

    //     if (topic.getType().getValueStr().equals(NetworkTableType.getStringFromObject(value)))
    //         DriverStation.reportError(
    //                 String.format(
    //                         "[Error] VecKit: Cannot set NT4 Value (%s) of %s type to value of %s type",
    //                         name, topic.getType().getValueStr(), NetworkTableType.getStringFromObject(value)),
    //                 true);

    //     Boolean setValError = false;

    //     if (value.getClass().equals(Double.class)) setValError = !pub.setDouble((Double) value);
    //     else if (value.getClass().equals(Integer.class)) setValError = !pub.setInteger((Integer) value);
    //     else if (value.getClass().equals(Boolean.class)) setValError = !pub.setBoolean((Boolean) value);
    //     else if (value.getClass().equals(String.class)) setValError = !pub.setString((String) value);

    //     if (setValError)
    //         DriverStation.reportError(String.format("[Error] VecKit: Failed to set value (%s:%s)", name, value),
    // true);

    //     // lastValue = value;
    // }

    // // public void logUSB() {
    // //     if (logEntry == null)
    // //         DriverStation.reportError(
    // //                 String.format("[Error] VecKit: Cannot USB Log topic (%s) (Invalid Type: %s)", name, type),
    // true);

    // //     if (lastValue.getClass().equals(Double.class) && logEntry instanceof DoubleLogEntry i)
    // //         i.append((Double) lastValue);
    // //     else if (lastValue.getClass().equals(Integer.class) && logEntry instanceof IntegerLogEntry i)
    // //         i.append((Integer) lastValue);
    // //     else if (lastValue.getClass().equals(Boolean.class) && logEntry instanceof BooleanLogEntry i)
    // //         i.append((Boolean) lastValue);
    // //     else if (lastValue.getClass().equals(String.class) && logEntry instanceof StringLogEntry i)
    // //         i.append((String) lastValue);
    // // }

    // public NetworkTableValue get() {
    //     if (!pub.isValid())
    //         throw new RuntimeException(String.format("[Error] VecKit: Cannot get an invalid publisher (%s)", name));
    //     return sub.get();
    // }
}
