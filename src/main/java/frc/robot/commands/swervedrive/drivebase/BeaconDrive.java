// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class BeaconDrive extends Command
{

  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final SwerveController controller;
  private       double           angle    = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public BeaconDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.controller = swerve.getSwerveController();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
     double xVelocity   = Math.pow(vX.getAsDouble(), 3);
     double yVelocity   = Math.pow(vY.getAsDouble(), 3);
      angle = Math.atan2(4 - swerve.getPose().getY(),  16 - swerve.getPose().getX());
      
      ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVelocity, yVelocity, angle, swerve.getHeading().getRadians(), swerve.maximumSpeed);
      swerve.drive(SwerveController.getTranslation2d(correctedChassisSpeeds),correctedChassisSpeeds.omegaRadiansPerSecond * controller.config.maxAngularVelocity,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
