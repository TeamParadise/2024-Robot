// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class alignNote extends Command {
  PhotonPipelineResult visionResult;
  PIDController turnController;
  CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
  double tx;
  public static final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.05).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  /** Creates a new allignNote. */
  public alignNote() {
    // addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(0.15, 0, 0);
    turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionResult = RobotContainer.vision.intakeCamera.getLatestResult();
    System.out.println("Running!");
    if (visionResult.hasTargets()) {
      tx = visionResult.getBestTarget().getYaw();
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("align output", turnController.calculate(tx, 0));
      drivetrain.applyRequest(() -> robotDrive.withRotationalRate(turnController.calculate(tx, 0))).schedule();;
    }
    else{
      drivetrain.applyRequest(() -> robotDrive.withRotationalRate(0)).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.applyRequest(() -> robotDrive.withRotationalRate( 0)).schedule();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
