package frc.robot.commands;
import frc.robot.*;
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
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Autonomous extends CommandBase {

    private DrivebaseSub drivebasesub;
    private AutonomousSub autonomoussub;
    private Intake intakesub;
   //private Limelight limelight;

    private SensorCollection leftEncoder;
    private SensorCollection rightEncoder;

    private Timer timer = new Timer();

    private AnalogGyro gyro;


    //general
    public boolean starting;

    //two ball auto
    public boolean twoballDriveForward = false;
    public boolean twoballTurn180 = false;
    public boolean twoballReturn = false;

    

     //true makes it turn right 90 degrees, false means no turn
    private boolean startPathweaver;

    public void oneBallAuto(double leftEncoderValue, double rightEncoderValue) {
        this.intakesub.intake.set(-Constants.intakeSpeed);
        if (leftEncoderValue >= -15000) {
            autonomoussub.driveLeft(-Constants.autonomousSpeed);
        }
        if (rightEncoderValue <= 15000) {
            autonomoussub.driveRight(Constants.autonomousSpeed);
        }
        if (leftEncoderValue < -15000 && rightEncoderValue > 15000) {
            this.starting = false;
        }
    }
    

    public void twoBallAuto(double leftEncoderValue, double rightEncoderValue) {
        if (this.twoballDriveForward) {
            this.intakesub.intake.set(Constants.intakeSpeed);
            if (leftEncoderValue <= 15000) {
                autonomoussub.driveLeft(Constants.autonomousSpeed);
            }
            if (rightEncoderValue >= -15000) {
                autonomoussub.driveRight(-Constants.autonomousSpeed);
            }
            if (leftEncoderValue > 15000 && rightEncoderValue < -15000) {
                this.twoballDriveForward = false;
                this.twoballTurn180 = true;
            }
        }
        if (this.twoballTurn180) {
            if (gyro.getAngle() < 180) {
                this.autonomoussub.driveRight(Constants.autonomousTurnSpeed);
            }
            if (gyro.getAngle() >= 180) {
                this.twoballTurn180 = false;
                this.twoballReturn = true;
            }
        }
        if (this.twoballReturn) {
            this.intakesub.intake.set(-Constants.intakeSpeed);
            if (leftEncoderValue >= -20000) {
                autonomoussub.driveLeft(-Constants.autonomousSpeed);
            }
            if (rightEncoderValue <=20000) {
                autonomoussub.driveRight(Constants.autonomousSpeed);
            }
            if (leftEncoderValue < -15000 && rightEncoderValue > 15000) {
                this.starting = false;
                this.twoballReturn = false;
            }
        }
    }

    public Autonomous(AutonomousSub autonomoussub1, DrivebaseSub drivebasesub1, Intake intake1) {
        autonomoussub = autonomoussub1;
        drivebasesub = drivebasesub1;
        intakesub = intake1;
        addRequirements(this.autonomoussub);
        addRequirements(this.drivebasesub);
        addRequirements(this.intakesub);
        this.leftEncoder = drivebasesub.encoderLeft;
        this.rightEncoder = drivebasesub.encoderRight;
        this.gyro = RobotContainer.gyro;
    }
    
    @Override
    public void initialize() {
        leftEncoder.setQuadraturePosition(0,0);
        rightEncoder.setQuadraturePosition(0,0);
        this.starting = true;
        this.twoballTurn180 = false;
        this.twoballDriveForward = false;
        this.twoballReturn = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftEncoderValue = this.leftEncoder.getQuadraturePosition();
        double rightEncoderValue = this.rightEncoder.getQuadraturePosition();
        if (this.starting) {
            oneBallAuto(leftEncoderValue, rightEncoderValue);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.starting = false;
        this.twoballTurn180 = false;
        this.twoballDriveForward = false;
        this.twoballReturn = false;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}