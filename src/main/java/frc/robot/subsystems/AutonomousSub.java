package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Drivebase;

public class AutonomousSub extends SubsystemBase{
    
    public AutonomousSub() {
    }

    public static void runAutonomous(){
        Robot.m_robotContainer.drivebasesub.teleopDrive(0.5, 0);
    
    }

    public void autoDrive(double move, double turn) {
        //Robot.m_robotContainer.drivebasesub.m_drive.arcadeDrive(.1, .1);
        Robot.m_robotContainer.drivebasesub.teleopDrive(move, turn);
    }

    public void driveLeft(double move) {
        Robot.m_robotContainer.drivebasesub.leftmotorgroup.set(move);
    }

    public void driveRight(double move) {
        Robot.m_robotContainer.drivebasesub.rightmotorgroup.set(move);
    }

}