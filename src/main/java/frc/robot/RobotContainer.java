// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Pass;
import frc.robot.commands.Amp.AmpShoot;
import frc.robot.commands.Amp.ampController;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Drivetrain.DriveToNote;
import frc.robot.commands.Drivetrain.DriveToNoteDistance;
import frc.robot.commands.Intake.intakePIDF;
import frc.robot.commands.Primer.PrimerBeamBreakerBroken;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Scoring.AutoSpeakerShoot;
import frc.robot.commands.Scoring.BackupShoot;
import frc.robot.commands.Scoring.Shoot;
import frc.robot.commands.Shooter.shooterPIDF;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.FOCSwitch;

import java.awt.geom.Path2D;
import java.util.Optional;

public class RobotContainer {
  public static double MaxSpeed = 6; // 6 meters per second desired top speed
  public static double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final CommandXboxController coJoystick = new CommandXboxController(1);
  public static VisionSubsystem vision;
  public final static PrimerSubsystem m_primerSubsystem = new PrimerSubsystem();
  public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final AmpSubsystem m_ampSubsystem = new AmpSubsystem();
  public static SendableChooser<Command> autoChooser;

  public static Trigger primerBeamTrigger = new Trigger(() -> m_primerSubsystem.getPrimerBeamBreaker());

  private static Path2D blueSpeaker = new Path2D.Float();
  private static Path2D redSpeaker = new Path2D.Float();
  
  public static final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxSpeed * 0.05)
    .withCenterOfRotation(new Translation2d(0, 0))
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withSteerRequestType(SteerRequestType.MotionMagic);

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop                         
                                                               
  public static final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.04).withRotationalDeadband(MaxAngularRate * 0.04) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  public static final FOCSwitch focDrive = new FOCSwitch().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withSwitchSpeed(4.0);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    if (Constants.VisionConstants.kVisionEnabled) {
      vision = new VisionSubsystem();
    }
    m_shooterSubsystem.setDefaultCommand(new shooterPIDF(-1500));
    m_intakeSubsystem.setDefaultCommand(new intakeController(0));
    
    // Configure driver controller
    configureDriverController();

    // Configure co-driver controller
    configureCoDriverController();

    // Configure named autonomous commands
    configureAutoCommands();

    // Test commands (DELETE AFTER TESTED)
    coJoystick.povUp().whileTrue(new DriveToNoteDistance(-3).alongWith(new IntakeNote()));
    coJoystick.povLeft().whileTrue(new ampController(0.1));
    coJoystick.povRight().whileTrue(new ampController(-0.1));
    coJoystick.povDown().onTrue(new AmpShoot());

    // Log when we have a note
    primerBeamTrigger.whileTrue(new PrimerBeamBreakerBroken());

    // Configure heading controller in the field centric facing angle
    headingDrive.HeadingController.setPID(10, 0, 0);
    headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    // Create auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set motor ramp rates
    drivetrain.getModule(0).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(2).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(3).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("Auto Intake", new ParallelRaceGroup(new DriveToNote(-3).withTimeout(1.5), new IntakeNote()));
    NamedCommands.registerCommand("Auto Intake Slow", new ParallelRaceGroup(new DriveToNote(-3.5).withTimeout(2), new IntakeNote()));
    NamedCommands.registerCommand("Just Shoot", new Shoot());
    NamedCommands.registerCommand("Shoot At Speaker", new AutoSpeakerShoot());
    NamedCommands.registerCommand("Point At Speaker", drivetrain.applyRequest(() -> headingDrive.withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX())))));
    NamedCommands.registerCommand("Reset Pose to Stage", new InstantCommand(() -> drivetrain.resetPoseToStage()));
    NamedCommands.registerCommand("Just Intake", new IntakeNote());

  }

  private void configureDriverController() {
      // Driver Joystick Controls
      // Reset Pose
      joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
      // Run Everything Backwards (Spit)
      joystick.leftTrigger(0.1).whileTrue(new RetractNote(SpeedConstants.kRetract, -0.1).alongWith(new intakePIDF(-2500)));
      // Normal Intake Note
      joystick.leftBumper().whileTrue(new IntakeNote());
      // Auto Shoot when at Speaker
      joystick.rightTrigger(0.1).onTrue(new AutoSpeakerShoot());
      // Point towards Speaker
      joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> headingDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)                                                                       
             .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
             .withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX())))));
      // New Shoot Command
      joystick.a().onTrue(new Shoot());
      // Old Shoot Command (backup)
      joystick.b().onTrue(new BackupShoot());
      // Backup drive command
      joystick.x().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
           .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
           .withRotationalRate(-joystick.getRightX() * MaxAngularRate
          )));})));
      // Normal (FOC) drive command
      joystick.y().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> focDrive.withVelocityX(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? joystick.getLeftY() * MaxSpeed : -joystick.getLeftY() * MaxSpeed) // Drive forward with
         .withVelocityY(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? joystick.getLeftX() * MaxSpeed : -joystick.getLeftX()) // Drive left with negative X (left)
         .withRotationalRate(-joystick.getRightX() * MaxAngularRate
       )));})));
      // X formation with swerve module
      joystick.rightStick().whileTrue(drivetrain.applyRequest(() -> brake));
      // Auto Intake
      joystick.leftStick().whileTrue(new DriveToNote(-3).alongWith(new IntakeNote()));
  }

  private void configureCoDriverController() {
    // New Shoot Command
    coJoystick.leftTrigger(0.1).whileTrue(new Shoot());
    // Pass Command
    coJoystick.leftBumper().onTrue(new Pass());
    // Auto speaker shoot
    coJoystick.rightTrigger(0.1).whileTrue(new AutoSpeakerShoot());
    // Backup Shoot Command
    coJoystick.a().onTrue(new BackupShoot());
    //Y --- Intake note with intake and primers
    coJoystick.y().whileTrue(new IntakeNote());
  }

  public RobotContainer() {
    configureBindings();

    // MAKE SURE TO TUNE THESE VALUES ON A REAL ROBOT (these are kind of annoying to tune on a simulation)
    redSpeaker.moveTo(16.541748, 1.709145);
    redSpeaker.lineTo(14.275851, 1.709145);
    redSpeaker.lineTo(14.275851, 4.113369);
    redSpeaker.lineTo(16.541748, 4.113369);
    redSpeaker.closePath();

    blueSpeaker.moveTo(0, 1.709145);
    blueSpeaker.lineTo(2.265897, 1.709145);
    blueSpeaker.lineTo(2.265897, 4.113369);
    blueSpeaker.lineTo(0, 4.113369);
    blueSpeaker.closePath();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static Boolean checkIntersection() {
    double x = drivetrain.getState().Pose.getX() - 0.381503746;
    double y = 8.220855 - drivetrain.getState().Pose.getY();
    return DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red)) ? redSpeaker.contains(x, y, 0.76300749201, 0.76300749201) : blueSpeaker.contains(x, y, 0.76300749201, 0.76300749201);
  }

  public static Rotation2d getSpeakerRotation() {
    return DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
      ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX()))
      : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX()));
  }

  public static Boolean getRobotPointedToSpeaker() {
    double rotationToSpeaker = getSpeakerRotation().getDegrees();
    
    return (drivetrain.getState().Pose.getRotation().getDegrees() - 15 < rotationToSpeaker) && (drivetrain.getState().Pose.getRotation().getDegrees() + 15 > rotationToSpeaker);
  }
}
