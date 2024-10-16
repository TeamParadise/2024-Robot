package frc.robot.commands.Led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;

public class ledStop extends Command{
    public final static LedSubsystem m_ledSubsystem = LedSubsystem.getInstance();
    /** Creates a new ledsubsytem. */
  public ledStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeSubsystem);
    SmartDashboard.putBoolean("Should Stop?", true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sets my variable in my led subsystems to true meaning my periodic loop wont run anything inside it. 
    m_ledSubsystem.setStop(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_ledSubsystem.setStop(true);
    return false;
  }
}
