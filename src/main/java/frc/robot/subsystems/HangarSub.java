// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class HangarSub extends SubsystemBase {
  public CANSparkMax hangarMotor1 = new CANSparkMax(Constants.hangarMotor1, MotorType.kBrushless);
  public CANSparkMax hangarMotor2 = new CANSparkMax(Constants.hangarMotor2, MotorType.kBrushless);

  public RelativeEncoder hangarEncoder1 = hangarMotor1.getEncoder();
  public RelativeEncoder hangarEncoder2 = hangarMotor2.getEncoder();

  public MotorControllerGroup hangarmotorgroup = new MotorControllerGroup(hangarMotor1, hangarMotor2);
  

  XboxController xboxcontroller = Robot.m_robotContainer.xboxcontroller;

  public HangarSub() {}

  public void hangarMotor1Move(double move) {
    hangarMotor1.set(move);
  }
  public void hangarMotor2Move(double move) {
    hangarMotor2.set(move);
  }

}
                                         