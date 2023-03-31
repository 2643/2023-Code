// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;



/**
 * The Constants class provides a convenient place for _TEAMs to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declaRED
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to REDuce verbosity.
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

    //Roles Replaced by buttons
    public static final boolean FIELD_RELATIVE_MODE = true;
    public static boolean ROTATIONAL_AXIS_MODE = false;


    //MAX SPEEDS 
    public static double MAX_METERS_PER_SECOND = 19800 / 60 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;//Change based on max speed
    // FIXME: Make sure to take other factors into account such as radius from the center of the robot
    public static double MAX_RADIANS_PER_SECOND = MAX_METERS_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS); //12.773732
    
    public static final double AUTONOMOUS_SLOW_MODE_MULTIPLIER = 0.2;
    public static final double AUTONOMOUS_VELOCITY_PER_SECOND = MAX_METERS_PER_SECOND * 0.1;
    public static final double AUTONOMOUS_RADIANS_PER_SECOND = AUTONOMOUS_VELOCITY_PER_SECOND / Math.hypot(TRANSLATION_2D_METERS, TRANSLATION_2D_METERS);

    public static final double MAX_VOLTAGE = 12.5;

    // public static final GenericEntry kP = Shuffleboard.getTab("EncoderE").add("kP", 0.2).getEntry();
    // public static final GenericEntry kI = Shuffleboard.getTab("EncoderE").add("kI", 0).getEntry();
    // public static final GenericEntry kD = Shuffleboard.getTab("EncoderE").add("kD", 0).getEntry();

    // public static final GenericEntry targetEncoderAngle = Shuffleboard.getTab("EncoderE").add("(Encoder)", 0).getEntry();
    // public static final GenericEntry currentAngle = Shuffleboard.getTab("EncoderE").add("Current Angle(Gyro)", 0).getEntry();

    public class ArmGrab {
        public static final int GRABBER_MOTOR_PORT = 17;
        public static final double GRABBER_TARGET_RPM = 7500; 
        public static final int GRABBER_LIMIT_SWITCH_PORT = 0;
        public static final double GRABBER_PERCENT_OUTPUT = 0.08;//0.143 //0.12
        public static final double GRABBER_CUBE_PERCENT_OUTPUT = 0.05/3;
        public static final double TARGET_CUBE_CURRENT_VALUE = 10;
        //TODO: Find the actual values
        public static final double TARGET_CONE_CURRENT_VALUE = 16;
        public static final double GRABBER_CONE_HARD_LIMIT = 0;
        public static final double GRABBER_CUBE_HARD_LIMIT = 0;
        public static final double GRABBER_MAX_OPEN_POS = -11.5*3;
    }

    public class ArmLift {
        public static final int ARM_LIFT_LEFT_MOTOR_PORT = 15;
        public static final int ARM_LIFT_RIGHT_MOTOR_PORT = 16;

        public static final int LIMIT_SWITCH_PORT_ONE = 1;
        public static final int LIMIT_SWITCH_PORT_TWO = 2;

        public static final int BOTTOM_ROW = 0;

        public static final double GEAR_RATIO = 153.6;
        public static final double COUNT_PER_DEGREES = 2048 * GEAR_RATIO / 360;

        public static final double TOP_SOFT_LIMIT_MOVEPOS = -58000;
        public static final double TOP_HARD_LIMIT_MOVEPOS = -63000;
        public static final double BOTTOM_SOFT_LIMIT_MOVEPOS = 58000;
        public static final double BOTTOM_HARD_LIMIT_MOVEPOS = 63000;

        //6 pot switch encoder values
        public static final int ENCODER_PORT = 2;
        public static final double REST = -57000;
        public static final double PICKUP = -6630;
        public static final double CONE = -2000;
        public static final double CUBE = 14586;
        public static final double CHARGING_STATION = 50000;
        public static final double FLOOR = 57000;
        
    }

    public class Position2d{
        //RED _PICKUP 
        public static final double STARTING_RED_PICKUP_X_VALUE = 2.161;
        public static final double STARTING_RED_PICKUP_Y_VALUE = 6.13;
        public static final double RED_TEAM_PICKUP_X_VALUE = 1.22;
        public static final double RED_TEAM_PICKUP_Y_VALUE = 6.21;

        //CUBE RED APRIL TAG ID 1
        public static final double FIRST_RED_TEAM_CUBE_X_VALUE = 14.4359;
        public static final double FIRST_RED_TEAM_CUBE_Y_VALUE = 0.9726;
       
        //CUBE RED APRIL TAG ID 2
        public static final double SECOND_RED_TEAM_CUBE_X_VALUE = 14.4359;
        public static final double SECOND_RED_TEAM_CUBE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 3;

        //CUBE RED APRIL TAG ID 3
        public static final double THIRD_RED_TEAM_CUBE_X_VALUE = 14.4359;
        public static final double THIRD_RED_TEAM_CUBE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 6;
       
        //BLUE_PICKUP
        public static final double STARTING_BLUE_PICKUP_X_VALUE = 13.45;
        public static final double STARTING_BLUE_PICKUP_Y_VALUE = 6.182;
        public static final double BLUE_TEAM_PICKUP_X_VALUE = 14.37;
        public static final double BLUE_TEAM_PICKUP_Y_VALUE = 6.182;

        //BLUE_CUBE APRIL TAG ID 6
        public static final double FIRST_BLUE_TEAM_CUBE_X_VALUE = 2.115;
        public static final double FIRST_BLUE_TEAM_CUBE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE;
 
        //BLUE_CUBE APRIL TAG ID 7
        public static final double SECOND_BLUE_TEAM_CUBE_X_VALUE = 2.115;
        public static final double SECOND_BLUE_TEAM_CUBE_Y_VALUE = SECOND_RED_TEAM_CUBE_Y_VALUE;
 
        //BLUE_CUBE APRIL TAG ID 8
        public static final double THIRD_BLUE_TEAM_CUBE_X_VALUE = 2.115;
        public static final double THIRD_BLUE_TEAM_CUBE_Y_VALUE = THIRD_RED_TEAM_CUBE_Y_VALUE;
 
        //CONE RED
        public static final double FIRST_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double FIRST_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE - 0.5588;

        public static final double SECOND_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double SECOND_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588;
 
        public static final double THIRD_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double THIRD_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 2;

        public static final double FOURTH_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double FOURTH_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 4;

        public static final double FIFTH_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double FIFTH_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 5;

        public static final double SIXTH_RED_TEAM_CONE_X_VALUE = FIRST_RED_TEAM_CUBE_X_VALUE + 0.1524;
        public static final double SIXTH_RED_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CUBE_Y_VALUE + 0.5588 * 7;

        //CONE_BLUE
        public static final double FIRST_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double FIRST_BLUE_TEAM_CONE_Y_VALUE = FIRST_RED_TEAM_CONE_Y_VALUE;

        public static final double SECOND_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double SECOND_BLUE_TEAM_CONE_Y_VALUE = SECOND_RED_TEAM_CONE_Y_VALUE;

        public static final double THIRD_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double THIRD_BLUE_TEAM_CONE_Y_VALUE = THIRD_RED_TEAM_CONE_Y_VALUE;

        public static final double FOURTH_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double FOURTH_BLUE_TEAM_CONE_Y_VALUE = FOURTH_RED_TEAM_CONE_Y_VALUE;

        public static final double FIFTH_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double FIFTH_BLUE_TEAM_CONE_Y_VALUE = FIFTH_RED_TEAM_CONE_Y_VALUE;

        public static final double SIXTH_BLUE_TEAM_CONE_X_VALUE = 1.96;
        public static final double SIXTH_BLUE_TEAM_CONE_Y_VALUE = SIXTH_RED_TEAM_CONE_Y_VALUE;
        
    }
        


}
