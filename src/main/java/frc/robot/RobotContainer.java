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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeNoteCustom;
import frc.robot.commands.Pass;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShootNoteCustom;
import frc.robot.commands.Arm.ArmHumanPlayer;
import frc.robot.commands.Arm.ArmHumanPlayerBack;
import frc.robot.commands.Arm.armAutoShoot;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Arm.scoreAmp;
import frc.robot.commands.Arm.startAmp;
import frc.robot.commands.Drivetrain.alignNoteDrive;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Intake.intakePIDF;
import frc.robot.commands.NewAuto.AutoShoot;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Primer.PrimerBeamBreakerBroken;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Shooter.shooterPIDF;
import frc.robot.commands.ShooterOnly.AutoSpeakerShootWithoutRetract;
import frc.robot.commands.ShooterOnly.AutoSpeakerShoot;
import frc.robot.commands.ShooterOnly.Shoot;
import frc.robot.commands.ShooterOnly.ShootAmp;
import frc.robot.commands.ShooterOnly.ShooterAuto;
import frc.robot.commands.ShooterOnly.StartAmp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  public double customAngle = 51;

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public static VisionSubsystem vision;
  private final CommandXboxController coJoystick = new CommandXboxController(1);
  public final static ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public final static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final static PrimerSubsystem m_primerSubsystem = new PrimerSubsystem();
  public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
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
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  public static final FOCSwitch focDrive = new FOCSwitch().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).withSwitchSpeed(4.0);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
      SmartDashboard.putNumber("Speed", 0);
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> robotDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    if (Constants.VisionConstants.kVisionEnabled) {
      vision = new VisionSubsystem();
    }
    m_ElevatorSubsystem.setDefaultCommand(new elevatorController(0));
    // m_ArmSubsystem.setDefaultCommand(new armPID(10));
    m_shooterSubsystem.setDefaultCommand(new shooterPIDF(-1500));
    m_intakeSubsystem.setDefaultCommand(new intakeController(0));


    //Driver controlls

    joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // joystick.rightTrigger(0.1).whileTrue(new armAutoShoot());
    joystick.rightTrigger(0.1).onTrue(new AutoSpeakerShootWithoutRetract());
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> headingDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)                                                                       
             .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
             .withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX())))));
    // joystick.a().whileTrue(new PrimeNote(SpeedConstants.kPrime));
    joystick.a().onTrue(new Shoot());
    joystick.leftTrigger(0.1).whileTrue(new RetractNote(SpeedConstants.kRetract, -0.1).alongWith(new intakePIDF(-2500)));
    joystick.leftBumper().whileTrue(new IntakeNote());
    joystick.x().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
           .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
           .withRotationalRate(-joystick.getRightX() * MaxAngularRate
          )));})));
    joystick.b().onTrue(new ShootNote());
    joystick.y().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> focDrive.withVelocityX(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? joystick.getLeftY() * MaxSpeed : -joystick.getLeftY() * MaxSpeed) // Drive forward with
         .withVelocityY(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? joystick.getLeftX() * MaxSpeed : -joystick.getLeftX()) // Drive left with negative X (left)
         .withRotationalRate(-joystick.getRightX() * MaxAngularRate
       )));})));
    joystick.rightStick().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.leftStick().whileTrue(new alignNoteDrive(-3).alongWith(new IntakeNote()));
    // joystick.leftStick().onTrue(new AutoShoot());
    
    
    //POV Up --- Move arm to feed note from intake into barrel
    //joystick.povUp().onTrue(new StartAmp());
    //POV Down --- Angle arm to 0
   // joystick.povDown().onTrue(new ShootAmp());

    //POV Left - sets position of elavator to bottom
   // joystick.povLeft().onTrue(new elevatorController(0));
    //POV Right --- Sets position of elevator to top
    //joystick.povRight().onTrue(new elevatorController(52.5));


    // //A --- Brake drivetrain
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // //B --- Drive with left joystick?
    // // joystick.b().whileTrue(drivetrain
    // //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // joystick.b().onTrue(drivetrain.applyRequest(() -> headingDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)                                                                       
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
    //         // .withTargetDirection(new Rotation2d(Robot.currentAlliance.equals(Optional.of(DriverStation.Alliance.Red)) ? 315 : 45))
    //         .withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.5) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX()))))); //Trig for speaker rotation

    // //Left Trigger --- Set new robot coordinates in x-direction
    // // joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(Units.inchesToMeters(15), 3, new Rotation2d(0)))));
    // joystick.leftTrigger().whileTrue(new alignNoteTranslation().alongWith(new IntakeNote()));


    // //X --- Drive field relative
    // joystick.x().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate
    //      )));})));

    // //Y --- Drive non-field relative?
    // joystick.y().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> robotDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate
    //     )));})));

    // //Left Bumper --- Set robot angle to track the speaker
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // // joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> headingDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)                                                                       
    // //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
    // //         .withTargetDirection(new Rotation2d(360 - Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.5 - Units.inchesToMeters(36.125)) - drivetrain.getState().Pose.getX())) //Trig for speaker rotation
    // //     ))); 
    // //Right Bumper --- Reset rotation angle to 0
    // // joystick.leftBumper().onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d(16.03, 5.475, new Rotation2d(0)))));
    
    // //POV Up --- Set arm to optimal angle for shooting note into speaker
    // // joystick.povUp().whileTrue(new armAutoAngle().alongWith(new shooterPIDF(m_ArmSubsystem.getDistance())));
    // joystick.rightTrigger().whileTrue(new armAutoShoot());

    // joystick.rightBumper().whileTrue(new PrimeNote(SpeedConstants.kPrime));

    // joystick.povUp().whileTrue(new armManual(0.4));
    // joystick.povDown().whileTrue(new armPID(20));
    // joystick.povLeft().whileTrue(new shooterManual(10));
    // joystick.povRight().whileTrue(new shooterManual(-10));


    

    //Co-Driver

    //POV Up --- Move arm to feed note from intake into barrel
    coJoystick.povUp().onTrue(new armPID(52));
    //POV Down --- Angle arm to 0
    coJoystick.povDown().onTrue(new armPID(0));

    //POV Left - sets position of elavator to bottom
   // coJoystick.povLeft().onTrue(new elevatorController(0));
    //POV Right --- Sets position of elevator to top
    //coJoystick.povRight().onTrue(new elevatorController(52.5));

    //A --- Automatically angle arm and shoot note
    coJoystick.a().onTrue(new ShootNote());
    // coJoystick.a().onTrue(new ShootNote(true));

    //B --- Lift arm to human player feeder. On release, bring arm back down
   // coJoystick.b().onTrue(new StartAmp());
    coJoystick.b().onTrue(new AmpShoot());

    //X --- Spin intake out
    //coJoystick.x().onTrue(new ShootAmp());
    //Y --- Intake note with intake and primers
    coJoystick.y().whileTrue(new IntakeNote());

    coJoystick.x().onTrue(new InstantCommand(() -> drivetrain.resetPoseToStage()));

    //Right Bumper --- Auto angle arm to speaker
    coJoystick.rightBumper().onTrue(new startAmp());

    //Left Bumper --- Reverse intake
    coJoystick.leftBumper().onTrue(new Pass());

    //Left Trigger --- Retract note with primers
    coJoystick.leftTrigger(0.1).whileTrue(new Shoot());
    //Right Trigger --- Push note into flywheel
    coJoystick.rightTrigger(0.1).whileTrue(new AutoSpeakerShootWithoutRetract());

    coJoystick.leftStick().whileTrue(new armPID(50).alongWith(new shooterPIDF(3500)));
    coJoystick.leftStick().toggleOnFalse(new armPID(50).alongWith(new shooterPIDF(3500).alongWith(new PrimeNote(SpeedConstants.kPrime))).withTimeout(1));

    coJoystick.back().whileTrue(new armPID(54));
    coJoystick.start().whileTrue(new IntakeNoteCustom());

    coJoystick.rightStick().onTrue(new ShootNoteCustom());

    primerBeamTrigger.whileTrue(new PrimerBeamBreakerBroken());
    primerBeamTrigger.onTrue(new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0.5)).andThen(new WaitCommand(0.5).andThen(new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0)))));
    //coJoystick.getLeftTriggerAxis().whileTrue
    // coJoystick.leftStick().whileTrue(/*new armPID(52*/new armManual(.3));


    headingDrive.HeadingController.setPID(10, 0, 0);
    headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    // NamedCommands.registerCommand("Arm Intake Position", new armPID(50).alongWith(new elevatorController(0)).withTimeout(2));
    // NamedCommands.registerCommand("Align and pick up note better version", new alignNoteDrive(-3).withTimeout(1.5));
    // NamedCommands.registerCommand("Align and pick up note better version fast", new alignNoteDrive(-2.5).withTimeout(1.5));
    // NamedCommands.registerCommand("Intake", new IntakeNote().withTimeout(2));
    // NamedCommands.registerCommand("Arm Auto Angle", new armAutoShoot().withTimeout(1));
    // NamedCommands.registerCommand("Arm Auto Shoot", new PrimeNote(SpeedConstants.kPrime).withTimeout(0.35));
    // NamedCommands.registerCommand("Auto Heading", drivetrain.applyRequest(() -> headingDrive.withVelocityX(0).withVelocityY(0)).withTimeout(3));
    // NamedCommands.registerCommand("Arm Auto Angle No Timeout", new armAutoShoot());
    // NamedCommands.registerCommand("Arm Auto Angle Quick", new armAutoShoot().withTimeout(0.75));
    NamedCommands.registerCommand("Retract", new RetractNote(-0.1, -0.1));
    // NamedCommands.registerCommand("Arm Pos", new armPID(10).withTimeout(1));
    // NamedCommands.registerCommand("Point At Speaker", drivetrain.applyRequest(() -> headingDrive.withVelocityX(0).withVelocityY(0).withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX())))).withTimeout(1));

    // New auto commands
    NamedCommands.registerCommand("New Shoot", new AutoShoot());
    NamedCommands.registerCommand("Auto Intake", new ParallelRaceGroup(new alignNoteDrive(-4.2).withTimeout(2), new IntakeNote()));
    NamedCommands.registerCommand("Auto Intake Slow", new ParallelRaceGroup(new alignNoteDrive(-3.5).withTimeout(2), new IntakeNote()));
    NamedCommands.registerCommand("Speaker", new ShooterAuto(SpeedConstants.kShooter));
    NamedCommands.registerCommand("Shooter Speedup", new shooterPIDF(SpeedConstants.kShooter));

    // New Auto Commands V2
    NamedCommands.registerCommand("Just Shoot", new Shoot());
    NamedCommands.registerCommand("Shoot At Speaker", new AutoSpeakerShootWithoutRetract());
    NamedCommands.registerCommand("Point At Speaker", drivetrain.applyRequest(() -> headingDrive.withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.541748) - drivetrain.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (0) - drivetrain.getState().Pose.getX())))));
    NamedCommands.registerCommand("Reset Pose to Stage", new InstantCommand(() -> drivetrain.resetPoseToStage()));
    NamedCommands.registerCommand("Just Intake", new IntakeNote());

    // Create auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    drivetrain.getModule(0).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(2).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(3).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
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
