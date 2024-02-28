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

public class shooterPIDF extends Command {
  private final SparkPIDController leftPIDController, rightPIDController;
  public double leftkp, rightkp, leftki, rightki, leftkd, rightkd, leftkiz, rightkiz, leftkff, rightkff, leftkmax, rightkmax, leftkmin, rightkmin;
  /** Creates a new shooterPIDF. */
  public shooterPIDF() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooterSubsystem);
    leftPIDController = RobotContainer.m_shooterSubsystem.leftShooter.getPIDController();
    rightPIDController = RobotContainer.m_shooterSubsystem.rightShooter.getPIDController();

    leftkp = 0;
    leftki = 0;
    leftkd = 0;
    leftkiz = 0;
    leftkff = 0;
    leftkmax = 1;
    leftkmin = -1;

    rightkp = 0;
    rightki = 0;
    rightkd = 0;
    rightkiz = 0;
    rightkff = 0;
    rightkmax = 1;
    rightkmin = -1;

    leftPIDController.setP(0);
    leftPIDController.setI(0);
    leftPIDController.setD(0);
    leftPIDController.setIZone(0);
    leftPIDController.setFF(0);
    leftPIDController.setOutputRange(-1, 1);

    rightPIDController.setP(0);
    rightPIDController.setI(0);
    rightPIDController.setD(0);
    rightPIDController.setIZone(0);
    rightPIDController.setFF(0);
    rightPIDController.setOutputRange(-1, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Left P Gain", 0);
    SmartDashboard.putNumber("Left I Gain", 0);
    SmartDashboard.putNumber("Left D Gain", 0);
    SmartDashboard.putNumber("Left I Zone", 0);
    SmartDashboard.putNumber("Left Feed Forward", 0);
    SmartDashboard.putNumber("Left Max Output", 1);
    SmartDashboard.putNumber("Left Min Output", -1);

    SmartDashboard.putNumber("Right P Gain", 0);
    SmartDashboard.putNumber("Right I Gain", 0);
    SmartDashboard.putNumber("Right D Gain", 0);
    SmartDashboard.putNumber("Right I Zone", 0);
    SmartDashboard.putNumber("Right Feed Forward", 0);
    SmartDashboard.putNumber("Right Max Output", 1);
    SmartDashboard.putNumber("Right Min Output", -1);

    SmartDashboard.putNumber("SetPoint", 100);
    SmartDashboard.putNumber("Left Output", RobotContainer.m_shooterSubsystem.leftShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Output", RobotContainer.m_shooterSubsystem.rightShooter.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Left Enabled", false);
    SmartDashboard.putBoolean("Right Enabled", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // read PID coefficients from SmartDashboard
     double leftp = SmartDashboard.getNumber("Left P Gain", 0);
     double lefti = SmartDashboard.getNumber("Left I Gain", 0);
     double leftd = SmartDashboard.getNumber("Left D Gain", 0);
     double leftiz = SmartDashboard.getNumber("Left I Zone", 0);
     double leftff = SmartDashboard.getNumber("Left Feed Forward", 0);
     double leftmax = SmartDashboard.getNumber("Left Max Output", 0);
     double leftmin = SmartDashboard.getNumber("Left Min Output", 0);

     double rightp = SmartDashboard.getNumber("Right P Gain", 0);
     double righti = SmartDashboard.getNumber("Right I Gain", 0);
     double rightd = SmartDashboard.getNumber("Right D Gain", 0);
     double rightiz = SmartDashboard.getNumber("Right I Zone", 0);
     double rightff = SmartDashboard.getNumber("Right Feed Forward", 0);
     double rightmax = SmartDashboard.getNumber("Right Max Output", 0);
     double rightmin = SmartDashboard.getNumber("Right Min Output", 0);

    if((leftp != leftkp)) { leftPIDController.setP(leftp); leftkp = leftp; }
    if((lefti != leftki)) { leftPIDController.setI(lefti); leftki = lefti; }
    if((leftd != leftkd)) { leftPIDController.setD(leftd); leftkd = leftd; }
    if((leftiz != leftiz)) { leftPIDController.setIZone(leftiz); leftkiz = leftiz; }
    if((leftff != leftkff)) { leftPIDController.setFF(leftff); leftkff = leftff; }
    if((leftmax != leftkmax) || (leftmin != leftkmin)) { 
      leftPIDController.setOutputRange(leftmin, leftmax); 
      leftkmin = leftmin; leftkmax = leftmax; 
    }

    if((rightp != rightkp)) { rightPIDController.setP(rightp); rightkp = rightp; }
    if((righti != rightki)) { rightPIDController.setI(righti); rightki = righti; }
    if((rightd != rightkd)) { rightPIDController.setD(rightd); rightkd = rightd; }
    if((rightiz != rightiz)) { rightPIDController.setIZone(rightiz); rightkiz = rightiz; }
    if((rightff != rightkff)) { rightPIDController.setFF(rightff); rightkff = rightff; }
    if((rightmax != rightkmax) || (rightmin != rightkmin)) { 
      rightPIDController.setOutputRange(rightmin, rightmax); 
      rightkmin = rightmin; rightkmax = rightmax; 
    }

    double setpoint = SmartDashboard.getNumber("SetPoint", 100);
    if (false) {
      leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    } else {
      RobotContainer.m_shooterSubsystem.leftShooter.set(0);
    }

    if (true) {
      rightPIDController.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    } else {
      RobotContainer.m_shooterSubsystem.leftShooter.set(0);
    }

    SmartDashboard.putNumber("Left Output", RobotContainer.m_shooterSubsystem.leftShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Output", RobotContainer.m_shooterSubsystem.rightShooter.getEncoder().getVelocity());
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
