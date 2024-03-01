// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.elevatorController;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Shooter.shooterPIDF;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armAmp extends SequentialCommandGroup {
  /** Creates a new ArmHumanPlayer. */
  public armAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new armPID(40.17).withTimeout(1),
      new armPID(40.17).alongWith(
      new shooterPIDF(2000).alongWith((
      new elevatorController(43.75).withTimeout(2))).andThen(
      new shooterPIDF(2000)).withTimeout(1).alongWith(
      new PrimeNote(0.4).withTimeout(1))));
  }
}
