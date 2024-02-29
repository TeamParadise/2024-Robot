// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShootTrig;
import frc.robot.commands.Arm.ArmHumanPlayer;
import frc.robot.commands.Arm.ArmHumanPlayerBack;
import frc.robot.commands.Arm.armAutoAngle;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Shooter.SpeedTune;
import frc.robot.commands.Shooter.shooterController;
import frc.robot.commands.Shooter.shooterPIDF;
import frc.robot.commands.Vision.PoseLogger;
import frc.robot.commands.Vision.VisionPoseEstimator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  public static double MaxSpeed = 6; // 6 meters per second desired top speed
  public static double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public static VisionSubsystem vision;
  private final CommandXboxController coJoystick = new CommandXboxController(1);
  public final static ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public final static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final static PrimerSubsystem m_primerSubsystem = new PrimerSubsystem();
  public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  
  
  private final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
    .withCenterOfRotation(new Translation2d(0, 0))
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withSteerRequestType(SteerRequestType.MotionMagic);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop                         
                                                               
  public static final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  PhoenixPIDController headingController = new PhoenixPIDController(0, 0, 0);

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
    m_ArmSubsystem.setDefaultCommand(new armPID(0));
    m_shooterSubsystem.setDefaultCommand(new shooterPIDF(0));


    //Driver controlls

    //A --- Brake drivetrain
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    //B --- Drive with left joystick?
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    //Left Bumper --- Set new robot coordinates in x-direction
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(Units.inchesToMeters(15), 3, new Rotation2d(0)))));


    //X --- Drive field relative
    joystick.x().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate
         )));})));

    //Y --- Drive non-field relative?
    joystick.y().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> robotDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate
        )));})));

    //Left Trigger --- Set robot angle to track the speaker
    joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> headingDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)                                                                       
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
            .withTargetDirection(new Rotation2d(Math.atan2(5.475 - drivetrain.getState().Pose.getY(),  (16.5 - Units.inchesToMeters(36.125)) - drivetrain.getState().Pose.getX())) //Trig for speaker rotation
        )));

    //Right Bumper --- Reset rotation angle to 0
    joystick.rightBumper().onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d(16.03, 5.475, new Rotation2d(0)))));
    
    //POV Up --- Set arm to optimal angle for shooting note into speaker
    joystick.povUp().whileTrue(new armAutoAngle());

    joystick.povDown().onTrue(new armPID(60));

    //Co-Driver

    //POV Up --- Move arm to feed note from intake into barrel
    coJoystick.povUp().onTrue(new armPID(43));

    //POV Down --- Angle arm to 0
    coJoystick.povDown().onTrue(new armPID(0));

    //POV Left - sets position of elavator to bottom
    coJoystick.povLeft().onTrue(new elevatorController(0));
    //POV Right --- Sets position of elevator to top
    coJoystick.povRight().onTrue(new elevatorController(52.5));

    //A --- Automatically angle arm and shoot note
    coJoystick.a().onTrue(new ShootNote(coJoystick.leftBumper().getAsBoolean()));

    coJoystick.b().whileTrue(new ArmHumanPlayer());
    coJoystick.b().toggleOnFalse(new ArmHumanPlayerBack());

    //X --- Spin intake out
    coJoystick.x().whileTrue(new intakeController(SpeedConstants.kOutake));
    //Y --- Automatically seek and move to notes in front of robot
    coJoystick.y().whileTrue(new IntakeNote());

    //Right Bumper --- Auto angle arm to speaker
    coJoystick.rightBumper().onTrue(new armAutoAngle());

    //Left Trigger --- Retract note with primers
    coJoystick.leftTrigger(0.1).whileTrue(new RetractNote(SpeedConstants.kRetract));
    //Right Trigger --- Push note into flywheel
    coJoystick.rightTrigger(0.1).whileTrue(new PrimeNote(SpeedConstants.kPrime));

    //coJoystick.getLeftTriggerAxis().whileTrue
    // coJoystick.leftStick().whileTrue(/*new armPID(52*/new armManual(.3));

    headingDrive.HeadingController.setPID(10, 0, 0);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Amp");
  }
}
