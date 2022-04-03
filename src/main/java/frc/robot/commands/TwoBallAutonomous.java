package frc.robot.commands;
import frc.robot.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutonomousSub;
import frc.robot.subsystems.DrivebaseSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TwoBallAutonomous extends CommandBase {

    private DrivebaseSub drivebasesub;
    private AutonomousSub autonomoussub;
    private Intake intakesub;
    private Arm arm;
   //private Limelight limelight;

    private SensorCollection leftEncoder;
    private SensorCollection rightEncoder;

    private Timer timer = new Timer();

    private ADXRS450_Gyro gyro;


    //general
    public boolean starting;

    //two ball auto
    public boolean twoballDriveForward = false;
    public boolean twoballTurn180 = false;
    public boolean twoballAdjustAngle = false;
    public boolean raiseArm = false;
    public boolean twoballReturn = false;
    public boolean twoballTaxi = false;
    

     //true makes it turn right 90 degrees, false means no turn
    private boolean startPathweaver;

    //right encoder value is positive when going forward
    //left encoder value is negative when going forward

    public void twoBallAuto(double leftEncoderValue, double rightEncoderValue, double armEncoderValue) {
        if (this.twoballDriveForward) {
            this.intakesub.intake.set(Constants.intakeSpeed);
            if (leftEncoderValue >= -15000) {
                autonomoussub.driveLeft(Constants.autonomousSpeed);
            }
            if (rightEncoderValue <= 15000) {
                autonomoussub.driveRight(-Constants.autonomousSpeed);
            }
            if (leftEncoderValue < -15000 && rightEncoderValue > 15000) {
                this.twoballDriveForward = false;
                this.twoballTurn180 = true;
                gyro.reset();
                this.leftEncoder.setQuadraturePosition(0, 0);
                this.rightEncoder.setQuadraturePosition(0, 0);
            }
        }
        if (this.twoballTurn180) {
            if (gyro.getAngle() < 180) {
                this.autonomoussub.driveRight(Constants.autonomousTurnSpeed);
                this.autonomoussub.driveLeft(Constants.autonomousTurnSpeed);
            }
            if (gyro.getAngle() >= 180) {
                this.twoballTurn180 = false;
                this.twoballAdjustAngle = true;
                this.leftEncoder.setQuadraturePosition(0, 0);
                this.rightEncoder.setQuadraturePosition(0, 0);
            }
        }
         if (this.twoballAdjustAngle) {
            if (gyro.getAngle() > 180) {
                this.autonomoussub.driveRight(-Constants.autonomousAdjustAngleSpeed);
                this.autonomoussub.driveLeft(-Constants.autonomousAdjustAngleSpeed);
            }
            if (gyro.getAngle() <= 180) {
                this.twoballAdjustAngle = false;
                this.raiseArm = true;
                this.leftEncoder.setQuadraturePosition(0, 0);
                this.rightEncoder.setQuadraturePosition(0, 0);
            }
        }
        if (this.raiseArm) {
            if (armEncoderValue < 1000) {
                this.arm.armRise();
            }
            if (armEncoderValue >= 1000) {
                this.raiseArm = false;
                this.twoballReturn = true;
            }
        }
        if (this.twoballReturn) {
            this.intakesub.intake.set(-Constants.intakeSpeed);
            if (leftEncoderValue >= -20000) {
                autonomoussub.driveLeft(Constants.autonomousSpeed);
            }
            if (rightEncoderValue <= 20000) {
                autonomoussub.driveRight(-Constants.autonomousSpeed);
            }
            if (leftEncoderValue < -20000 && rightEncoderValue > 20000) {
                this.twoballReturn = false;
                this.twoballTaxi = false;
                this.leftEncoder.setQuadraturePosition(0, 0);
                this.rightEncoder.setQuadraturePosition(0, 0);
            }
        }
        if (this.twoballTaxi) {
            if (leftEncoderValue >= -15000) {
                autonomoussub.driveLeft(Constants.autonomousSpeed*.5);
            }
            if (rightEncoderValue <= 15000) {
                autonomoussub.driveRight(-Constants.autonomousSpeed*.5);
            }
            if (leftEncoderValue < -15000 && rightEncoderValue > 15000) {
                this.starting = false;
                this.twoballTaxi = false;
                gyro.reset();
                this.leftEncoder.setQuadraturePosition(0, 0);
                this.rightEncoder.setQuadraturePosition(0, 0);
            }
        }
    }

    public TwoBallAutonomous(AutonomousSub autonomoussub1, DrivebaseSub drivebasesub1, Intake intake1, Arm arm1) {
        autonomoussub = autonomoussub1;
        drivebasesub = drivebasesub1;
        intakesub = intake1;
        arm = arm1;
        addRequirements(this.autonomoussub);
        addRequirements(this.drivebasesub);
        addRequirements(this.intakesub);
        addRequirements(this.arm);
        this.leftEncoder = RobotContainer.encoderLeft;
        this.rightEncoder = RobotContainer.encoderRight;
        this.gyro = RobotContainer.gyro;
    }
    
    @Override
    public void initialize() {
        leftEncoder.setQuadraturePosition(0,0);
        rightEncoder.setQuadraturePosition(0,0);
        gyro.reset();
        this.starting = true;
        this.twoballTurn180 = false;
        this.twoballAdjustAngle = false;
        this.twoballDriveForward = true;
        this.raiseArm = false;
        this.twoballReturn = false;
        this.twoballTaxi = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftEncoderValue = this.leftEncoder.getQuadraturePosition();
        double rightEncoderValue = this.rightEncoder.getQuadraturePosition();
        double armEncoderValue = this.arm.arm.getEncoder().getPosition();
        SmartDashboard.putNumber("Left Encoder auto", Robot.m_robotContainer.encoderLeft.getQuadraturePosition());
        SmartDashboard.putNumber("Right Encoder auto", Robot.m_robotContainer.encoderRight.getQuadraturePosition());
        SmartDashboard.putNumber("Gyro", Robot.m_robotContainer.gyro.getAngle());
        if (this.starting) {
            twoBallAuto(leftEncoderValue, rightEncoderValue, armEncoderValue);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.starting = false;
        this.twoballTurn180 = false;
        this.twoballAdjustAngle = false;
        this.twoballDriveForward = false;
        this.raiseArm = false;
        this.twoballReturn = false;
        this.twoballTaxi = false;
        leftEncoder.setQuadraturePosition(0,0);
        rightEncoder.setQuadraturePosition(0,0);
        gyro.reset();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}