// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Shooter.shooterPIDF;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pass extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public Pass() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new armPID(54).alongWith(
          new PrimeNote(SpeedConstants.kRetract).withTimeout(0.15).andThen(
          new shooterPIDF(6000).withTimeout(0.85)).andThen(
          new PrimeNote(SpeedConstants.kPrime).withTimeout(0.35))).withTimeout(2),
        new shooterPIDF(0).withTimeout(0.1)
      );
    }
  }
