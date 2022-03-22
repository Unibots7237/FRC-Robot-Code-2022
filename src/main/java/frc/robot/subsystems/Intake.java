package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    public Spark intake = new Spark(Constants.intakeSpark);
    
    XboxController xboxcontroller = Robot.m_robotContainer.xboxcontroller;

    public void intakeControl() {
        if(xboxcontroller.getAButton()){
            intake.set(Constants.intakeSpeed);
        }
        else if(xboxcontroller.getYButton()){
            intake.set(-Constants.intakeSpeed);
        }
        else{
            intake.set(0);
        }
    }

}