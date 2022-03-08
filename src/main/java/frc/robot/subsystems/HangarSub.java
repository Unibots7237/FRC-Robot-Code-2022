// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class HangarSub extends SubsystemBase {
  public PWMSparkMax hangarMotor1 = new PWMSparkMax(Constants.hangarMotor1);
  public PWMSparkMax hangarMotor2 = new PWMSparkMax(Constants.hangarMotor2);

  public MotorControllerGroup hangarmotorgroup = new MotorControllerGroup(hangarMotor1, hangarMotor2);
  

  XboxController xboxcontroller = Robot.m_robotContainer.xboxcontroller;

  public HangarSub() {}

  public void hangarControl() {
    if(xboxcontroller.getRightBumperPressed()) {
      hangarmotorgroup.set(Constants.intakeSpeed);
    }
    else if(xboxcontroller.getLeftBumperPressed()){
      hangarmotorgroup.set(-Constants.hangarSpeed);
    }
    else{
      hangarmotorgroup.set(0);
    }
  }
  
}
                                         