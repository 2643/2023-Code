// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int PIGEON_CAN = 2;

    //FRONT LEFT MODULE
    public static final int FRONT_LEFT_CANCODER = 5;
    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_TURN_MOTOR = 3;
    public static final double FRONT_LEFT_TURN_OFFSET = -Math.toRadians(92.46);

    //FRONT RIGHT MODULE
    public static final int FRONT_RIGHT_CANCODER = 14;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;
    public static final double FRONT_RIGHT_TURN_OFFSET = -Math.toRadians(138.16);

    //REAR LEFT MODULE
    public static final int REAR_LEFT_CANCODER = 8;
    public static final int REAR_LEFT_DRIVE_MOTOR = 9;
    public static final int REAR_LEFT_TURN_MOTOR = 10;
    public static final double REAR_LEFT_TURN_OFFSET = -Math.toRadians(349.53);

    //REAR RIGHT MODULE
    public static final int REAR_RIGHT_CANCODER = 11;
    public static final int REAR_RIGHT_DRIVE_MOTOR = 12;
    public static final int REAR_RIGHT_TURN_MOTOR = 13;
    public static final double REAR_RIGHT_TURN_OFFSET = -Math.toRadians(274.82);
        
    //joystick ports
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int ROTATIONAL_AXIS = 2;

    //Chassis length/width and field/robot relative mode control
    public static final double TRANSLATION_2D_METERS = 0.625/2;
    public static final boolean FIELD_RELATIVE_MODE = true;


    //MAX SPEEDS 
    public static double MAX_METERS_PER_SECOND = 19800 / 60 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;//Change based on max speed
    // FIXME: Make sure to take other factors into account such as radius from the center of the robot
    public static double MAX_RADIANS_PER_SECOND = MAX_METERS_PER_SECOND /
    Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS); //12.773732
    
    public static final double AUTONOMOUS_VELOCITY_PER_SECOND = MAX_METERS_PER_SECOND;
    public static final double AUTONOMOUS_RADIANS_PER_SECOND = AUTONOMOUS_VELOCITY_PER_SECOND /
    Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS);

    public static final double MAX_VOLTAGE = 12.5;
    public static final int GRABBER_MOTOR_PORT = 17;
    public static final int GRABBER_LIMIT_SWITCH_PORT = 0;
    public static final int GRABBER_MAX_OPEN_POS = 0;
    public static final double TARGET_CURRENT_VALUE = 10;
    public static final double GRABBER_PERCENT_OUTPUT = 0.143; //change


}
