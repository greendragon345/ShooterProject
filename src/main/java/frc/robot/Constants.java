// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shood.Utils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {


    public static final int hoodM_port_num = 9;
    public static final int shootM_port_num = 5;
    public static final int controlerport=0;
    //limit switches-ports are fake
    public static final int top_lim_num = 3;
    public static final int bot_lim_num = 4;
    public static final double Kp_shooter = 0.3;
    public static final double Ki_shooter = 0;
    public static final double Kd_shooter = 0;
    public static final double Kp_hood = 0;
    public static final double Ki_hood = 0;
    public static final double Kd_hood = 0;
    public static final double hood_ks=0;
    public static final double hood_kv=0;
    public static final double inch = 0.0254;
    public static final double hoogearRatio = (((1*40)/1.5)/12)*267 ;
    public static final int pulsePerRotationShooter = 2048;
    public static final int pulsePerRotationHood = 7;
    public static final double pulsesPer360angle = (hoogearRatio *pulsePerRotationHood)/360+22.5;
    public static final double diamShoot = 4*inch;
    public static final double peremiterShoot = diamShoot*Math.PI;
    public static final int gearRatioS = 1;
    public static final double pulsesPerMeterS = gearRatioS * pulsePerRotationShooter / peremiterShoot;
    public static final double SPEED_ER = 0.1;
    public static final double ANGLE_ER = 0.1;
    public static final double WRONG_ANGLE = 0;
    public static final double WRONG_SPEED = 0;
    public static final double CALIBRATION_SPEED = 0.1;
    public static final double shooks=0.0781714895719943;
    public static final double shookv=0.0195292864738957;
    public static final double shookv2=0.000593393697301388;


    public static final double commandKp = 0;
    public static final double commandKi = 0;
    public static final double commandKd = 0;

    public static final double hoodKp = 0;
    public static final double hoodKi = 0;
    public static final double hoodKd = 0;




    public static final double[][] LOOK_UP_TABLE = new double[10][10];
    
    public static final double g=9.81;
    //robot-hieght - target hieght
    public static double hieght =1.68;
    public static double mass =0.27;
    public static double airdensity = 1.185;
    public static double aerodynmiccoef=0.47;
    public static double ballarea = Math.pow((0.24/2),2)*Math.PI;
    public static final double K=airdensity*aerodynmiccoef*ballarea/(2*g*mass);
    // 0.00475626239461;
    public static final double minspeed = Math.sqrt(1*g*2/
    Math.sin(Math.toRadians(90))*(Math.sin(Math.toRadians(90))-g*K));
    public static double hoodratio =0;
    public static double shootratio =0;
    public static final double maxH =4;
    public static double shooterexitspeedgradient  =-1;
    public static double shooterexitspeedintercept =-1;








}
