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

    final public static int xboxcontroller = 0;

    //Motors
    final public static int frontRightTalon = 02;
    final public static int backRightTalon = 03;
    final public static int frontLeftTalon = 04;
    final public static int backLeftTalon = 01;
    final public static int intakeSpark = 00;
    final public static int armSpark = 05;
    final public static int hangarMotor1 = 00;
    final public static int hangarMotor2 = 00;

    //autonomous
    final public static double autonomousSpeed = .5;
    final public static double autonomousTurnSpeed = .3;

    //Arm
    final public static double armHoldUp = 0.08;
    final public static double armHoldDown = 0.13;
    final public static double armTravel = 0.5;
    final public static double armTimeUp = 0.5;
    final public static double armTimeDown = 0.35;

    //Arm Variables
    final public static boolean burstMode = false;
    final public static double autoStart = 0;
    final public static boolean goForAuto = false;

    //motor speeds
    final public static double intakeSpeed = 1;
    final public static double hangarSpeed = .25;

    //arm speeds part 2 if we cannot use the copy and pasted code
    final public static double armRise = .2;
    final public static double armDescend = -.4;
    final public static double armControlledDescent = -0.05;

   
    //hangar stuff
    final public static double hangarEncoderExtend = 0;

}
