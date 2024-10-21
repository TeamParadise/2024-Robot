// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShootTest extends SequentialCommandGroup {
  /** Creates a new AmpShootTest. */
  public AmpShootTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ampPIDF(-2.9).alongWith(new WaitCommand(0.5).andThen(new AmpShoot())), new WaitCommand(0.5), new ampPIDF(-3.5).withTimeout(0.75), new ampPIDF(-1.5).withTimeout(0.75));
  }
}
