// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int ARM_LIFT_LEFT_MOTOR_PORT = 15;
  public static final int ARM_LIFT_RIGHT_MOTOR_PORT = 16;

  public static final int LIMIT_SWITCH_PORT_ONE = 1;
  public static final int LIMIT_SWITCH_PORT_TWO = 2;

  public static final int BOTTOM_ROW = 0;

  public static final double GEAR_RATIO = 153.6;
  public static final double COUNT_PER_DEGREES = 2048 * GEAR_RATIO / 360;

  public static final double TOP_SOFT_LIMIT_MOVEPOS = 135 * COUNT_PER_DEGREES;
  public static final double TOP_HARD_LIMIT_MOVEPOS = 150 * COUNT_PER_DEGREES;
  public static final double BOTTOM_SOFT_LIMIT_MOVEPOS = -135 * COUNT_PER_DEGREES;
  public static final double BOTTOM_HARD_LIMIT_MOVEPOS = -150 * COUNT_PER_DEGREES;

  //6 pot switch encoder values
  public static final int ENCODER_PORT = 2;
  public static final double REST = -10000;
  public static final double PICKUP = 10000;
  public static final double CONE = 15000;
  public static final double CUBE = 20000;
  public static final double CHARGING_STATION = 25000;
  public static final double FLOOR = 30000;


}
