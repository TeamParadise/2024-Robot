// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class MotorConstants
  {
    public static final int leftIntakeMotorID = 37;
    public static final int rightIntakeMotorID = 22;

    public static final int leftElevatorMotorID = 32;
    public static final int rightElevatorMotorID = 42;

    public static final int leftClimberMotorID = 25;
    public static final int rightClimberMotorID = 26;

    public static final int leftPrimerMotorID = 27;
    public static final int rightPrimerMotorID = 28;

    public static final int leftFlywheelMotorID = 29;
    public static final int rightFlywheelMotorID = 20;

    public static final int leftArmMotorID = 21;
    public static final int rightArmMotorID = 23;
  }

  public static class SpeedConstants{
    public static final double intakeSpeed = 0.5;
    public static final double outtakeSpeed = 0.5;

    public static final double primeSpeed = 0.5;
    public static final double feedSpeed = 0.5;

    public static final double speakerSpeed = 4000; //RPM
    public static final double ampSpeed = 1000; //RPM
  }

  public static class OperatorConstants {
  

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;
  }
}
