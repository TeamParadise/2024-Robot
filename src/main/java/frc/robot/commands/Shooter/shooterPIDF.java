// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class shooterPIDF extends Command {
  private final SparkPIDController leftPIDController, rightPIDController;
  private double setpoint;

  /** Creates a new shooterPIDF. */
  public shooterPIDF(double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooterSubsystem);
    leftPIDController = RobotContainer.m_shooterSubsystem.leftShooter.getPIDController();
    rightPIDController = RobotContainer.m_shooterSubsystem.rightShooter.getPIDController();
    this.setpoint = velocity;

    leftPIDController.setP(ShooterConstants.kLeftP);
    leftPIDController.setI(ShooterConstants.kLeftI);
    leftPIDController.setD(ShooterConstants.kLeftD);
    leftPIDController.setIZone(ShooterConstants.kLeftIz);
    leftPIDController.setFF(ShooterConstants.kLeftFF);
    leftPIDController.setOutputRange(ShooterConstants.kLeftMin, ShooterConstants.kLeftMax);

    rightPIDController.setP(ShooterConstants.kRightP);
    rightPIDController.setI(ShooterConstants.kRightI);
    rightPIDController.setD(ShooterConstants.kRightD);
    rightPIDController.setIZone(ShooterConstants.kRightIz);
    rightPIDController.setFF(ShooterConstants.kRightFF);
    rightPIDController.setOutputRange(ShooterConstants.kRightMin, ShooterConstants.kRightMax);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    rightPIDController.setReference(-setpoint, CANSparkBase.ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
