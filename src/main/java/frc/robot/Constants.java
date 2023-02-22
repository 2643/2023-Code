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
    public static final int FRONT_LEFT_CANCODER = 14;  //was 5
    public static final int FRONT_LEFT_DRIVE_MOTOR = 6; // was 4
    public static final int FRONT_LEFT_TURN_MOTOR = 7;  //was 3
    public static final double FRONT_LEFT_TURN_OFFSET = -Math.toRadians(47.3648);

    //FRONT RIGHT MODULE
    public static final int FRONT_RIGHT_CANCODER = 5;  //was 14
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;  //was 6
    public static final int FRONT_RIGHT_TURN_MOTOR = 3;  //was 7
    public static final double FRONT_RIGHT_TURN_OFFSET = -Math.toRadians(175.6055);  //was 138.16

    //REAR LEFT MODULE
    public static final int REAR_LEFT_CANCODER = 8;
    public static final int REAR_LEFT_DRIVE_MOTOR = 9;
    public static final int REAR_LEFT_TURN_MOTOR = 10;
    public static final double REAR_LEFT_TURN_OFFSET = -Math.toRadians(345.3223);  //was 349.53

    //REAR RIGHT MODULE
    public static final int REAR_RIGHT_CANCODER = 11;
    public static final int REAR_RIGHT_DRIVE_MOTOR = 12;
    public static final int REAR_RIGHT_TURN_MOTOR = 13;
    public static final double REAR_RIGHT_TURN_OFFSET = -Math.toRadians(275.1855); //was 274.82
        
    //joystick ports
    public static final int X_AXIS_PORT = 0;
    public static final int Y_AXIS_PORT = 1;
    public static final int ROTATIONAL_AXIS_PORT = 2;

    //Chassis length/width and field/robot relative mode control

    public static final double TRANSLATION_2D_METERS = 0.625/2;
    public static final boolean FIELD_RELATIVE_MODE = true;
    public static boolean ROTATIONAL_AXIS_MODE = true;


    //MAX SPEEDS 
    public static double MAX_METERS_PER_SECOND = 19800 / 60 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;//Change based on max speed
    // FIXME: Make sure to take other factors into account such as radius from the center of the robot
    public static double MAX_RADIANS_PER_SECOND = MAX_METERS_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS); //12.773732
    
    public static final double AUTONOMOUS_VELOCITY_PER_SECOND = MAX_METERS_PER_SECOND;
    public static final double AUTONOMOUS_RADIANS_PER_SECOND = AUTONOMOUS_VELOCITY_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS);
    public static final double AUTONOMOUS_SLOW_MODE_MULTIPLIER = 0.2;

    public static final double MAX_VOLTAGE = 12.5;
    
    //public static final Drivetrain.JoystickConfiguration M_JOYSTICK = Drivetrain.JoystickConfiguration.RotationalJoystick;
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
//  * constants. This class should not be used for any other purpose. All constants should be declared
//  * globally (i.e. public static). Do not put anything functional in this class.
//  *
//  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
//  * constants are needed, to reduce verbosity.
//  */
// public final class Constants {

//     public static final int PIGEON_CAN = 2;

//     //FRONT LEFT MODULE
//     public static final int FRONT_LEFT_CANCODER = 14;  //was 5
//     public static final int FRONT_LEFT_DRIVE_MOTOR = 6; // was 4
//     public static final int FRONT_LEFT_TURN_MOTOR = 7;  //was 3
//     public static final double FRONT_LEFT_TURN_OFFSET = -Math.toRadians(226.58);
//     //public static final double FRONT_LEFT_TURN_OFFSET = -Math.toRadians(47.6355);

//     //FRONT RIGHT MODULE
//     public static final int FRONT_RIGHT_CANCODER = 5;  //was 14
//     public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;  //was 6
//     public static final int FRONT_RIGHT_TURN_MOTOR = 3;  //was 7
//     //public static final double FRONT_RIGHT_TURN_OFFSET = -Math.toRadians(179.5605);  //was 138.16
//     public static final double FRONT_RIGHT_TURN_OFFSET = -Math.toRadians(358.07);  //was 138.16


//     //REAR LEFT MODULE
//     public static final int REAR_LEFT_CANCODER = 8;
//     public static final int REAR_LEFT_DRIVE_MOTOR = 9;
//     public static final int REAR_LEFT_TURN_MOTOR = 10;
//     //public static final double REAR_LEFT_TURN_OFFSET = -Math.toRadians(347.5195);  //was 349.53
//     public static final double REAR_LEFT_TURN_OFFSET = -Math.toRadians(166.72);  //was 349.53


//     //REAR RIGHT MODULE
//     public static final int REAR_RIGHT_CANCODER = 11;
//     public static final int REAR_RIGHT_DRIVE_MOTOR = 12;
//     public static final int REAR_RIGHT_TURN_MOTOR = 13;
//     public static final double REAR_RIGHT_TURN_OFFSET = -Math.toRadians(91.5); //was 274.82
//     //public static final double REAR_RIGHT_TURN_OFFSET = -Math.toRadians(92.2852); //was 274.82

        
//     //joystick ports
//     public static final int X_AXIS_PORT = 0;
//     public static final int Y_AXIS_PORT = 1;
//     public static final int ROTATIONAL_AXIS_PORT = 2;

//     //Chassis length/width and field/robot relative mode control
//     public static final double TRANSLATION_2D_METERS = 0.625/2;
//     public static final boolean FIELD_RELATIVE_MODE = false;
//     public static final boolean ROTATIONAL_AXIS_MODE = true;

//     //MAX SPEEDS 
//     public static double MAX_METERS_PER_SECOND = 19800 / 60 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;//Change based on max speed
//     // FIXME: Make sure to take other factors into account such as radius from the center of the robot
//     public static double MAX_RADIANS_PER_SECOND = MAX_METERS_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS); //12.773732
    
//     public static final double AUTONOMOUS_VELOCITY_PER_SECOND = MAX_METERS_PER_SECOND;
//     public static final double AUTONOMOUS_RADIANS_PER_SECOND = AUTONOMOUS_VELOCITY_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS);
//     public static final double AUTONOMOUS_SLOW_MODE_MULTIPLIER = 0.2;

//     public static final double MAX_VOLTAGE = 12.5;
    
// }
