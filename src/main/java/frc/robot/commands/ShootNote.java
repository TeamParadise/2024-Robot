// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Primer.RetractNote;
import frc.robot.commands.Shooter.shooterController;
import frc.robot.commands.Shooter.shooterSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new armPID(55).withTimeout(0.25), 
      new armPID(55).alongWith(
        new RetractNote(SpeedConstants.kRetract).withTimeout(0.25).alongWith(
        new shooterController(SpeedConstants.kShooter).withTimeout(1)).andThen(
        new PrimeNote(SpeedConstants.kPrime).withTimeout(0.45))).withTimeout(1.7),
      new shooterController(0).withTimeout(0.1)
     );
  }
}