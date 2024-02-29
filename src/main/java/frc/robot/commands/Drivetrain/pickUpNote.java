// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pickUpNote extends SequentialCommandGroup {
  /** Creates a new test. */
  public pickUpNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new alignNote().repeatedly()
      .andThen(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withVelocityX(-1)).repeatedly().withTimeout(1))
      .andThen(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withVelocityX(0))));
  }
}
