package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;  
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivebase;

public class DrivebaseSub extends SubsystemBase{

    public WPI_TalonSRX leftFrontTalon = new WPI_TalonSRX(Constants.frontLeftTalon);
    public WPI_TalonSRX leftBackTalon = new WPI_TalonSRX(Constants.backLeftTalon);
    public WPI_TalonSRX rightFrontTalon = new WPI_TalonSRX(Constants.frontRightTalon);
    public WPI_TalonSRX rightBackTalon = new WPI_TalonSRX(Constants.backRightTalon);

    MotorControllerGroup leftmotorgroup = new MotorControllerGroup(leftFrontTalon, leftBackTalon);
    MotorControllerGroup rightmotorgroup = new MotorControllerGroup(rightFrontTalon, rightBackTalon);

    DifferentialDrive m_drive = new DifferentialDrive(leftmotorgroup, rightmotorgroup);

    //Sensor Objects 
    //public SensorCollection encoderLeft = new SensorCollection(leftFrontTalon);
    //public SensorCollection encoderRight = new SensorCollection(rightFrontTalon);

    //Gyro for testing
    public ADXRS450_Gyro gyro;

    public DrivebaseSub() {
        this.gyro = RobotContainer.gyro;
    }

    public void teleopDrive(double move, double turn) {

        SmartDashboard.putNumber("Left Encoder tele", Robot.m_robotContainer.encoderLeft.getQuadraturePosition());
        SmartDashboard.putNumber("Right Encoder tele", Robot.m_robotContainer.encoderRight.getQuadraturePosition());
        SmartDashboard.putNumber("Gyro", Robot.m_robotContainer.gyro.getAngle());

        if (Math.abs(move) <= .05) {
            move = 0;
        }
        if (Math.abs(turn) <= .05) {
            turn = 0;
        }

        m_drive.arcadeDrive(turn, move);
    }

    public void driveLeft(double move) {
        leftmotorgroup.set(move);
    }

    public void driveRight(double move) {
        rightmotorgroup.set(move);
    }
}