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
    public static final double loopPeriodSec = 0.02;
  }
  public static class TurningConstants{
    public static final double turningGearRatio = 12.8;

    public static final double turningkP = 4;
    public static final double turningkD = 0.02;

    public static final int[] turningMotorIds = {4, 6, 8, 10};
    public static final int[] turningEncoderIds = {11, 12, 13, 14};
    public static final double[] turningEncoderOffsets = {26.191, 137.594, 71.455, 186.943};
  }
  public static class DrivingConstants{
    public static final double driveGearRatio = 6.75;

    public static final int driveWheelRadius = 2;

    public static final double drivingkV = 0.1;
    public static final double maxV = 15;

    public static final int[] driveMotorIds = {3, 5, 7, 9};
  }
}
