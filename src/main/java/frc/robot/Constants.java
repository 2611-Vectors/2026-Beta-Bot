// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
  }

  public static class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 61;
    public static final int RIGHT_MOTOR_ID = 62;

    public static final int LEFT_LINEAR_ACTUATOR_ID = 0;
    public static final int RIGHT_LINEAR_ACTUATOR_ID = 1;
  }

  public static class FullSendConstants {
    public static final int MAIN_MOTOR_ID = 60;
  }

  public static class IntakeConstants {
    public static final int PIVOT_MOTOR_ID = 32;
    public static final int WHEEL_MOTOR_ID = 31;

    public static final int PIVOT_ENCODER_ID = 0;
    public static final double PIVOT_ENCODER_OFFSET = 0.0;

    public static final double PIVOT_ANGLE_TOLERANCE = 0.5;
    public static final double PIVOT_IN_ANGLE = 0.0;
    public static final double PIVOT_OUT_ANGLE = 0.0;
  }

  public static class TransitionConstants {
    public static final int UPPER_LEFT_MOTOR_ID = 41;
    public static final int UPPER_RIGHT_MOTOR_ID = 42;
    public static final int LOWER_MOTOR_ID = 52;
  }

  public static class VisionConstants {
    // Apriltag Field Layout

    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double FIELD_WIDTH = 16.541;
    public static final double FIELD_HEIGHT = 8.069;

    // Name of the PhotonVision Reef Camera
    public static String RightRearCam = "Camera4";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToRightRearCam =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.2;
    public static double maxZError = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }
}
