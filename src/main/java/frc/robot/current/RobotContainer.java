// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.subsystems.Climber;
import frc.robot.current.subsystems.Hopper;
import frc.robot.current.subsystems.Intake;
import frc.robot.current.subsystems.LedOperation;
import frc.robot.current.subsystems.Outtake;
import frc.robot.current.subsystems.Pivot;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.GyroIO;
import frc.robot.current.subsystems.swerveDrive.GyroIONavX;
import frc.robot.current.subsystems.swerveDrive.ModuleIO;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSim;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSpark;
import frc.robot.lib.ObjectVision.ObjectVision;
import frc.robot.lib.ObjectVision.ObjectVisionIO;
import frc.robot.lib.ObjectVision.ObjectVisionIODetection;
import frc.robot.lib.commands.DriveCommands;
import frc.robot.lib.roboRoute.RoboRoute;
import frc.robot.lib.roboRoute.RoboRouteIO;
import frc.robot.lib.roboRoute.RoboRouteIONetworkTables;
import frc.robot.lib.util.AllianceRotationUtil;
import frc.robot.lib.vision.Vision;
import frc.robot.lib.vision.VisionIO;
import frc.robot.lib.vision.VisionIOPhotonVision;
import frc.robot.lib.vision.VisionIOPhotonVisionSim;
import static frc.robot.lib.vision.VisionConstants.*;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private Drive drive;
  private Intake intake;
  private Pivot pivot;
  @SuppressWarnings("unused")
  private Vision vision;
  private ObjectVision objectVision;
  private Outtake outtake;
  private Hopper hopper;
  private Climber climber;
  @SuppressWarnings("unused")
  private LedOperation leds;
  @SuppressWarnings("unused")
  private RoboRoute roboRoute;

  private final CommandXboxController driveXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controlXbox = new CommandXboxController(OperatorConstants.kOtherControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVision(camera1Name, robotToCamera1),
            new VisionIOPhotonVision(camera2Name, robotToCamera2),
            new VisionIOPhotonVision(camera3Name, robotToCamera3));

        objectVision = new ObjectVision(drive, new ObjectVisionIODetection());
        roboRoute = new RoboRoute(new RoboRouteIONetworkTables());
        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
            new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));
        
        objectVision = new ObjectVision(drive, new ObjectVisionIODetection());
        roboRoute = new RoboRoute(new RoboRouteIONetworkTables());

        break;
      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIO() {
            },
            new VisionIO() {
            });

        objectVision = new ObjectVision(drive, new ObjectVisionIO() {});
        roboRoute = new RoboRoute(new RoboRouteIO() {});
        break;

    }

    hopper = new Hopper();
    outtake = new Outtake(drive, hopper);
    intake = new Intake(drive);
    pivot = new Pivot();
    climber = new Climber();

    leds = new LedOperation();

    NamedCommands.registerCommand("Launch", outtake.timedLaunch(6));
    NamedCommands.registerCommand("LaunchOff", outtake.stop());
    NamedCommands.registerCommand("IntakeOn", intake.intakeSlow());
    NamedCommands.registerCommand("IntakeOff", intake.stop());
    NamedCommands.registerCommand("PivotDown", pivot.gotoCollectionPos());
    NamedCommands.registerCommand("PivotUp", pivot.gotoStoredPos());
    NamedCommands.registerCommand("ClimbUp", climber.climbMaxBoth());
    NamedCommands.registerCommand("ClimbDown", climber.climbFlatBoth());

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

    // Add sysID routines to the SendableChooser for autos
    if (Constants.isTuningMode) {
      autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("FFCharacterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // SPEED LIMITS
    private final double movementMultiplier = 0.25;
    private final double rotationMultiplier = 0.25;
    
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> movementMultiplier * -driveXbox.getLeftY(),
            () -> movementMultiplier * -driveXbox.getLeftX(),
            () -> rotationMultiplier * -0.75 * driveXbox.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driveXbox
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
                .ignoringDisable(true));

        driveXbox.leftTrigger().whileTrue(intake.intakeSlow()).onFalse(intake.stop());
        driveXbox.leftBumper().whileTrue(intake.intakeFast()).onFalse(intake.stop());

        driveXbox.rightBumper().onTrue(outtake.manualTuningLaunch()).onFalse(outtake.stop());

        driveXbox.povUp().onTrue(pivot.gotoStoredPos());
        driveXbox.povDown().onTrue(pivot.gotoCollectionPos());

        // driveXbox.y().whileTrue(climber.climbMaxBoth());
        // driveXbox.a().whileTrue(climber.climbStowedBoth());
        // driveXbox.x().whileTrue(Commands.defer(() -> climber.climbDownIndividual(), Set.of(climber)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Change to null to prevent auto from running during demos
    return null;
    // return autoChooser.get();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }
}
