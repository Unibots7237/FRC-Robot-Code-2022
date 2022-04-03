// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HangarSub;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Hangar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HangarSub hangarsub;
  private XboxController xboxcontroller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Hangar(HangarSub hangarsub1) {
    this.hangarsub = hangarsub1;
    this.xboxcontroller = RobotContainer.xboxcontroller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangarsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
      hangarsub.hangarMotor1.setIdleMode(IdleMode.kBrake);
  }   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xboxcontroller.getRightBumper()) {
      hangarsub.hangarMotor1Move(-.25);
    }
    if (xboxcontroller.getLeftBumper()) {
      hangarsub.hangarMotor1Move(.25);
    }
    if (!xboxcontroller.getLeftBumper() && !xboxcontroller.getRightBumper()) {
      hangarsub.hangarMotor1Move(0);
    }
    /*
    RelativeEncoder hangarEncoder1 = hangarsub.hangarEncoder1;
    RelativeEncoder hangarEncoder2 = hangarsub.hangarEncoder2;

    
    while (hangarEncoder1.getPosition() <= Constants.hangarEncoderExtend && hangarEncoder2.getPosition() <= Constants.hangarEncoderExtend) {
      if (hangarEncoder1.getPosition() <= Constants.hangarEncoderExtend) {
        hangarsub.hangarMotor1Move(Constants.hangarSpeed);
      } else{hangarsub.hangarMotor1Move(0.0);}
      if (hangarEncoder2.getPosition() <= Constants.hangarEncoderExtend) {
        hangarsub.hangarMotor2Move(Constants.hangarSpeed);
      } else{hangarsub.hangarMotor2Move(0.0);}
    } 
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
