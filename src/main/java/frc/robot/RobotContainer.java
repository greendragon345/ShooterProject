// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.Util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Calibrate;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shood.Shood;
import frc.robot.subsystems.Shood.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final XboxController conrtl;
  private final JoystickButton shoootButton;
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.shoootButton.whenPressed(new ShootCommand(shooterSubsystem, tw, tr,false,utils));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(()->shooterSubsystem.setShootPower(0.8),shooterSubsystem)
    .andThen(new WaitCommand(2),new InstantCommand(()->SmartDashboard.putNumber("shooter velocity", shooterSubsystem.getShooterSpeed()),shooterSubsystem), 
    new InstantCommand(()-> shooterSubsystem.setShootPower(0),shooterSubsystem));
  }
  //0.2 5.364
  //0.4 11.7
  //0.8 12.059

  private static RobotContainer me;
  private final Shood shooterSubsystem;
  private final Tower tw;
  private final Turret tr;
  private  Utils utils;

  private RobotContainer() {
    me = this;
    this.conrtl = new XboxController(Constants.controlerport);
    this.shoootButton=new JoystickButton(this.conrtl,2);
    shooterSubsystem = new Shood();
    tw = new Tower();
    tr = new Turret();
    //userControllers = new User(shooterSubsystem.getShootingCommand());
  }

  public static RobotContainer getRobotContainer() {
    if (me==null) {
      return new RobotContainer();
    }
    return me;
  }

  public Shood getShood() {
    return this.shooterSubsystem;
  }

  public void onEnable() {
    shooterSubsystem.setNeutralModeHood(true);
    new Calibrate(shooterSubsystem).schedule();
  }

}
