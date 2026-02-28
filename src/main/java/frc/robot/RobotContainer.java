// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VectorKit.vision.Vision;
import frc.robot.VectorKit.vision.VisionIOPhotonVision;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.AutoTargetDriverControl;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindToStart;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.FullSend;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_Drive;
  private final Shooter m_Shooter;
  private final Intake m_Intake;
  private final Transition m_Transition;
  private final FullSend m_FullSend;
  private final Hood m_Hood;
  private final Vision m_Vision;

  // Controller
  private final CommandXboxController m_DriverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController m_OperatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Shooter = new Shooter();
    m_Transition = new Transition();
    m_Intake = new Intake();
    m_FullSend = new FullSend();
    m_Hood = new Hood();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        m_Drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        m_Vision =
            new Vision(
                m_Drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.RightRearCam, VisionConstants.robotToRightRearCam),
                new VisionIOPhotonVision(
                    VisionConstants.LeftRearCam, VisionConstants.robotToLeftRearCam),
                new VisionIOPhotonVision(
                    VisionConstants.RightFrontCam, VisionConstants.robotToRightFrontCam),
                new VisionIOPhotonVision(
                    VisionConstants.LeftFrontCam, VisionConstants.robotToLeftFrontCam));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        m_Vision = null;

        break;

      default:
        // Replayed robot, disable IO implementations
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        m_Vision = null;

        break;
    }

    NamedCommands.registerCommand("autoTarget", new AutoTarget(m_Drive, m_Shooter));
    NamedCommands.registerCommand("runIntake", m_Intake.setIntakeRPM(() -> 2000.0));
    NamedCommands.registerCommand(
        "runTransition", m_Transition.setTransitionRPM(() -> 1000.0, () -> 3000.0));
    NamedCommands.registerCommand("runFullSend", m_FullSend.setFullSendRPM(() -> 5000.0));
    NamedCommands.registerCommand("resetHood", m_Hood.setHoodPos(() -> 0.65));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_Drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_Drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_Hood.setDefaultCommand(m_Hood.manualHoodPos());

    // Default command, normal field-relative drive
    m_Drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_Drive,
            () -> -m_DriverController.getLeftY(),
            () -> -m_DriverController.getLeftX(),
            () -> -m_DriverController.getRightX()));

    // Lock to 0° when A button is held
    m_DriverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> -m_DriverController.getLeftY(),
                () -> -m_DriverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    m_DriverController.x().onTrue(Commands.runOnce(m_Drive::stopWithX, m_Drive));

    // Reset gyro to 0° when Back button is pressed
    m_DriverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_Drive.setPose(
                            new Pose2d(m_Drive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_Drive)
                .ignoringDisable(true));

    m_OperatorController.rightTrigger().whileTrue(m_Intake.manualIntakeRPM(() -> false));

    m_OperatorController.leftTrigger().whileTrue(m_Intake.manualIntakeRPM(() -> true));

    m_DriverController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                m_FullSend.manualFullSendRPM(() -> false),
                m_Transition.manualTransitionRPM(() -> false)));

    m_DriverController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                m_FullSend.manualFullSendRPM(() -> true),
                m_Transition.manualTransitionRPM(() -> true)));

    m_DriverController
        .leftBumper()
        .toggleOnTrue(new AutoTargetDriverControl(m_Drive, m_Shooter, m_DriverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathfindToStart(new PathPlannerAuto(autoChooser.get().getName()));
  }
}
