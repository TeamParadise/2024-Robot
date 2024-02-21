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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.Arm.armController;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Shooter.SpeedTune;
import frc.robot.commands.Shooter.shooterController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController coJoystick = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final static ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public final static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final static PrimerSubsystem m_primerSubsystem = new PrimerSubsystem();
  public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> robotDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    m_ArmSubsystem.setDefaultCommand(new armPID(43));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

   

    joystick.x().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate
         )));})));

    joystick.y().onTrue(drivetrain.runOnce(() -> {drivetrain.removeDefaultCommand();}).andThen(new InstantCommand(() -> {drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> robotDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate
        )));})));

      


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    coJoystick.povUp().onTrue(new armPID(60));
    coJoystick.povDown().onTrue(new armPID(0));

    coJoystick.povLeft().onTrue(new elevatorController(0));
    coJoystick.povRight().onTrue(new elevatorController(52.5));

    coJoystick.a().onTrue(new ShootNote());
    coJoystick.b().whileTrue(new intakeController(SpeedConstants.kIntake));
    coJoystick.x().whileTrue(new intakeController(SpeedConstants.kOutake));
    coJoystick.y().whileTrue(new IntakeNote());

    coJoystick.rightBumper().onTrue(new SpeedTune(0.05));
    coJoystick.leftBumper().onTrue(new SpeedTune(-0.05));

    coJoystick.leftTrigger(0.1).whileTrue(new RetractNote(SpeedConstants.kRetract));
    // coJoystick.leftTrigger(0.1).whileTrue(new intakeController(coJoystick.getLeftTriggerAxis() * coJoystick.getLeftTriggerAxis()));

    coJoystick.rightTrigger(0.1).whileTrue(new PrimeNote(SpeedConstants.kPrime));

  }

  public RobotContainer() {
    
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
