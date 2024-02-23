// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllianceAnimation extends SequentialCommandGroup {
  /** Creates a new SetIndividual. */
  public AllianceAnimation() {
    addRequirements(RobotContainer.ledSubsystem);

    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(1, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(2, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(3, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(4, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(5, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(6, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(7, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(8, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(9, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(10, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(11, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(12, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(13, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(14, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(15, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(16, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(17, Color.kRed)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(18, Color.kWhite)));
    addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(19, Color.kRed)));
    // addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(20, Color.kRed)));
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
