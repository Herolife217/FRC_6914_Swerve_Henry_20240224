// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AccelerateShooter;
import frc.robot.commands.Feed;
import frc.robot.commands.HomeClimber;
import frc.robot.commands.IntakeNoteAutomatic;
import frc.robot.commands.RunArmClosedLoop;
import frc.robot.commands.RunIntakeOpenLoop;
import frc.robot.commands.RunShooterAtVelocity;
import frc.robot.commands.ShootNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
 // The robot's subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  private final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kSensorDIOPort);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  private final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO);
  private final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO);
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My driverController
  private final Dashboard dashboard;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Command Groups
  ParallelCommandGroup feedAndShootSubwoofer = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.kSubwooferSpeed),
    new Feed(m_intake)
  );
  ParallelCommandGroup feedAndShootPodium = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.k3mSpeed),
    new Feed(m_intake)
  );
  SequentialCommandGroup shootSubwoofer = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
    new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
    feedAndShootSubwoofer
  );
  SequentialCommandGroup shootPodium = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
    new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
    feedAndShootPodium
  );
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
    new HomeClimber(m_portClimber),
    new HomeClimber(m_starboardClimber)
  );

  //ForceFeed forceFeed = new ForceFeed(m_intake, m_shooter);

  ParallelCommandGroup forceReverse = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, -IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, -ShooterConstants.kAmpSpeed)
  );

  // halp. thrrows exception
  // SequentialCommandGroup amp = new SequentialCommandGroup(
  //   new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
  //   forceFeed
  // );

  private final SendableChooser<Command> autoChooser;

  // temp strings
  private String k1NC = "1-Note Center";
  private String k1NAS = "1-NoteAmpSide";
  private String k2NAS = "2-NoteAmpSide";
  private String k2NFC = "2-NoteFieldCenter";
  private String k2NMS = "2-NoteMidSpeaker";
  private String k2NMSA = "2-NoteMidSpeakerAmp";
  private String k3NAS = "3-NoteAmpSide";
  private String k3NMSC = "3-NoteMidSpeakerCenter";
  private String kBL = "BasicLeave";
  private String kBS = "BasicShoot";
  private String kEntropy = "Entropy";

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("shootSubwoofer", shootSubwoofer);
    NamedCommands.registerCommand("shootpodium", shootPodium);
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
