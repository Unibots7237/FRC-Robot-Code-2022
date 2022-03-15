// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HangarSub;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Hangar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HangarSub hangarsub;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Hangar(HangarSub hangarsub1) {
    this.hangarsub = hangarsub1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangarsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
