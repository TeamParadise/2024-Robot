// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.Arm.ArmHumanPlayer;
import frc.robot.commands.Arm.ArmHumanPlayerBack;
import frc.robot.commands.Arm.armAmp;
import frc.robot.commands.Arm.armAutoAngle;
import frc.robot.commands.Arm.armAutoShoot;
import frc.robot.commands.Arm.armManual;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Arm.scoreAmp;
import frc.robot.commands.Arm.startAmp;
import frc.robot.commands.Drivetrain.alignNoteDrive;
import frc.robot.commands.Drivetrain.pickUpNote;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Primer.PrimerBeamBreakerBroken;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Primer.FlywheelBeamBreakerBroken;
import frc.robot.commands.Shooter.setSpeakerPID;
import frc.robot.commands.Shooter.shooterController;
import frc.robot.commands.Shooter.shooterManual;
import frc.robot.commands.Shooter.shooterPIDF;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.awt.geom.Path2D;

public class RobotContainer {
  public static double MaxSpeed = 6; // 6 meters per second desired top speed
  public static double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

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
  public static SendableChooser<String> mainAutoChooser = new SendableChooser<>(), rightAutoChooser = new SendableChooser<>(), leftAutoChooser = new SendableChooser<>(), centerAutoChooser = new SendableChooser<>();

  private Path2D stage = new Path2D.Float();
  public Trigger underStage = new Trigger(() -> stage.intersects(drivetrain.getState().Pose.getX() - 0.76300749201, 8.220855 - drivetrain.getState().Pose.getY(), 0.76300749201, 0.76300749201));
  
  
  public static final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
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
    m_ArmSubsystem.setDefaultCommand(new armPID(10));
    m_shooterSubsystem.setDefaultCommand(new shooterController(-0.1));


    //Driver controlls

    //A --- Brake drivetrain
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    //B --- Drive with left joystick?
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    //Left Bumper --- Set new robot coordinates in x-direction
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(Units.inchesToMeters(15), 3, new Rotation2d(0)))));


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
    joystick.leftBumper().onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d(16.03, 5.475, new Rotation2d(0)))));
    
    //POV Up --- Set arm to optimal angle for shooting note into speaker
    // joystick.povUp().whileTrue(new armAutoAngle().alongWith(new shooterPIDF(m_ArmSubsystem.getDistance())));
    joystick.rightTrigger().whileTrue(new armAutoShoot());

    joystick.rightBumper().whileTrue(new PrimeNote(SpeedConstants.kPrime));

    joystick.povUp().whileTrue(new armManual(0.4));
    joystick.povDown().whileTrue(new armManual(-0.4));
    joystick.povLeft().whileTrue(new shooterManual(10));
    joystick.povRight().whileTrue(new shooterManual(-10));


    

    //Co-Driver

    //POV Up --- Move arm to feed note from intake into barrel
    coJoystick.povUp().onTrue(new armPID(62.5));

    //POV Down --- Angle arm to 0
    coJoystick.povDown().onTrue(new armPID(0));

    //POV Left - sets position of elavator to bottom
    coJoystick.povLeft().onTrue(new elevatorController(0));
    //POV Right --- Sets position of elevator to top
    coJoystick.povRight().onTrue(new elevatorController(52.5));

    //A --- Automatically angle arm and shoot note
    coJoystick.a().onTrue(new armPID(60));
    // coJoystick.a().onTrue(new ShootNote(true));

    //B --- Lift arm to human player feeder. On release, bring arm back down
    coJoystick.b().whileTrue(new ArmHumanPlayer());
    coJoystick.b().toggleOnFalse(new ArmHumanPlayerBack());

    //X --- Spin intake out
    coJoystick.x().onTrue(new scoreAmp());
    //Y --- Intake note with intake and primers
    coJoystick.y().whileTrue(new IntakeNote());

    //Right Bumper --- Auto angle arm to speaker
    coJoystick.rightBumper().onTrue(new startAmp());

    //Left Bumper --- Reverse intake
    coJoystick.leftBumper().whileTrue(new
     intakeController(SpeedConstants.kOutake));

    //Left Trigger --- Retract note with primers
    coJoystick.leftTrigger(0.1).whileTrue(new RetractNote(SpeedConstants.kRetract, -0.1));
    //Right Trigger --- Push note into flywheel
    coJoystick.rightTrigger(0.1).whileTrue(new PrimeNote(SpeedConstants.kPrime));

    Trigger flywheelBeamTrigger = new Trigger(() -> m_primerSubsystem.getFlywheelBeamBreaker()),
            primerBeamTrigger = new Trigger(() -> m_primerSubsystem.getPrimerBeamBreaker());
    flywheelBeamTrigger.whileTrue(new FlywheelBeamBreakerBroken());
    primerBeamTrigger.whileTrue(new PrimerBeamBreakerBroken());

    //coJoystick.getLeftTriggerAxis().whileTrue
    // coJoystick.leftStick().whileTrue(/*new armPID(52*/new armManual(.3));

    headingDrive.HeadingController.setPID(10, 0, 0);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    NamedCommands.registerCommand("Align and Pick Up Note", new pickUpNote().withTimeout(5));
    NamedCommands.registerCommand("Shoot Amp", new armAmp().withTimeout(5));
    NamedCommands.registerCommand("Arm Intake Position", new armPID(50).alongWith(new elevatorController(0)).withTimeout(2));
    NamedCommands.registerCommand("Align and pick up note better version", new alignNoteDrive().withTimeout(3));
    NamedCommands.registerCommand("Shoot in Speaker", new ShootNote(false));
    NamedCommands.registerCommand("Intake", new IntakeNote().withTimeout(3));

    mainAutoChooser.setDefaultOption("Left", "Left");
    mainAutoChooser.addOption("Center", "Center");
    mainAutoChooser.addOption("Right", "Right");

    leftAutoChooser.setDefaultOption("Amp", "Amp");
    leftAutoChooser.addOption("None", "None");

    centerAutoChooser.setDefaultOption("2 Note Speaker", "2 Note Speaker");
    centerAutoChooser.addOption("1 Note Speaker", "1 Note Speaker");
    centerAutoChooser.addOption("Leave", "Leave");

    drivetrain.getModule(0).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(2).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);
    drivetrain.getModule(3).getDriveMotor().getConfigurator().refresh(Constants.MotorConstants.driveRamp);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().refresh(Constants.MotorConstants.steerRamp);

    rightAutoChooser.setDefaultOption("1 Note Speaker", "1 Note Speaker");
    rightAutoChooser.setDefaultOption("2 Note Speaker", "2 Note Speaker");

    SmartDashboard.putData("Side Auto Chooser", mainAutoChooser);

    // Configure stage coordinates
    stage.moveTo(10, 2.6211122);
    stage.lineTo(10, 6.5);
    stage.lineTo(13, 4.412154);
    stage.closePath();

    stage.moveTo(5.8, 2.6211122);
    stage.lineTo(5.8, 6.5);
    stage.lineTo(2.8, 4.412154);
    stage.closePath();
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    if (Robot.allianceCurrentlySelectedAuto == "Left") {
      if (leftAutoChooser.getSelected() == "Amp") {
        return new PathPlannerAuto("Amp");
      } else if (leftAutoChooser.getSelected() == "None") {
        return new PathPlannerAuto("None");
      } else {
        return new PathPlannerAuto("None");
      }
    } else if (Robot.allianceCurrentlySelectedAuto == "Center") {
      if (centerAutoChooser.getSelected() == "2 Note Speaker") {
        return new PathPlannerAuto("Speaker Center 2 Note");
      } else if (centerAutoChooser.getSelected() == "1 Note Speaker") {
        return new PathPlannerAuto("Speaker Center 1 Note");
      } else if (centerAutoChooser.getSelected() == "Leave") {
        return new PathPlannerAuto("Leave");
      } else {
        return new PathPlannerAuto("None");
      }
    } else if (Robot.allianceCurrentlySelectedAuto == "Right") {
      if (rightAutoChooser.getSelected() == "1 Note Speaker") {
        return new PathPlannerAuto("Speaker Right 1 Note");
      } else if (rightAutoChooser.getSelected() == "2 Note Speaker") {
        return new PathPlannerAuto("Speaker Right 2 Note");
      } else {
        return new PathPlannerAuto("None");
      }
    } else {
      return new PathPlannerAuto("None");
    }
  }

  public Command getDriveBack() {
    return drivetrain.applyRequest(() -> robotDrive.withVelocityX(-0.9)).withTimeout(2).andThen(drivetrain.applyRequest(() -> robotDrive.withVelocityX(0)));
  }
}
