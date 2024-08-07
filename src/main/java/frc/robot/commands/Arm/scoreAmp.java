// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Shooter.shooterController;
import frc.robot.commands.Shooter.shooterPIDF;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreAmp extends SequentialCommandGroup {
  /** Creates a new ArmHumanPlayer. */
  public scoreAmp() {
    // Add your commands in the addCommands() call, e.g.

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new armPID(35).alongWith(
      new elevatorController(40).withTimeout(2).alongWith(
      new shooterController(0).withTimeout(0).andThen(
      new shooterPIDF(550)).withTimeout(1).andThen(
      new PrimeNote(0.25).withTimeout(1))).andThen(
      new elevatorController(0).alongWith(new WaitCommand(2)))));
  }
}
