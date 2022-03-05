// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangarSub extends SubsystemBase {
  public WPI_TalonSRX hangarMotor = new WPI_TalonSRX(Constants.hangarMotor);
  
  public HangarSub() {}

  public void extendHangar() {
      hangarMotor.set(Constants.hangarMotor);
  }
  
}
                                         