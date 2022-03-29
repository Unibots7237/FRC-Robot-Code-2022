// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.midi.MidiSystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autonomous;
import frc.robot.commands.Drivebase;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Hangar;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Arm;
//import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AutonomousSub;
import frc.robot.subsystems.DrivebaseSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HangarSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public final Intake intakesub = new Intake();

  public final DrivebaseSub drivebasesub = new DrivebaseSub();
  private final Drivebase drivebasecommand = new Drivebase(drivebasesub);

  private final Arm armsub = new Arm();
  private final ArmCommand armcommand = new ArmCommand(armsub);

  public final Limelight limelight = new Limelight();

  //public final HangarSub hangarsub = new HangarSub();
  //public final Hangar hangarcommand = new Hangar(hangarsub);

  public final AutonomousSub autonomoussub = new AutonomousSub();
  private final Autonomous autonomouscommand = new Autonomous(autonomoussub, drivebasesub, intakesub);

  private final IntakeCommand intakecommand = new IntakeCommand(intakesub);

  public static XboxController xboxcontroller = new XboxController(Constants.xboxcontroller);

  public static AnalogGyro gyro = new AnalogGyro(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Gyro
    gyro.reset();

    // Configure the button bindings
    configureButtonBindings();
    drivebasesub.setDefaultCommand(drivebasecommand);
    intakesub.setDefaultCommand(intakecommand);
    //hangarsub.setDefaultCommand(hangarcommand);
    armsub.setDefaultCommand(armcommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomouscommand;
  }
}