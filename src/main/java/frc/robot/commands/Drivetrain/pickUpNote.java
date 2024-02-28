// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pickUpNote extends ParallelDeadlineGroup {
  /** Creates a new pickUpNote. */
  public pickUpNote() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeNote().repeatedly().withTimeout(5));
    addCommands(new alignNote().withTimeout(3)
      .andThen(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withVelocityX(-1)).repeatedly().withTimeout(1))
      .andThen(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withVelocityX(0))));
  }
}
