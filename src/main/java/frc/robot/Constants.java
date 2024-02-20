// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class MotorConstants {
    public static final int leftArmMotorID = 13; //1
    public static final int rightArmMotorID = 14; //2
    public static final int leftElevatorMotorID = 15; //3
    public static final int rightElevatorMotorID = 16; //4

    public static final int leftPrimerMotorID = 17; //5
    public static final int rightPrimerMotorID = 18; //6
    public static final int leftShooterMotorID = 19; //7
    public static final int rightShooterMotorID = 20; //8

    public static final int intakeMotorID = 7; //9
  }

  public static class SpeedConstants {
    public static final double kIntake = 0.6;
    public static final double kPrime = 0.5;
    public static final double kRetract = -0.1;
    public static final double kShooter = 0.885;
    public static final double kOutake = -kIntake;
  }
}
