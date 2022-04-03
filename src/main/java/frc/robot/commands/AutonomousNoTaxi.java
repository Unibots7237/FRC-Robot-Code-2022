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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonomousNoTaxi extends CommandBase {

    private DrivebaseSub drivebasesub;
    private AutonomousSub autonomoussub;
    private Intake intakesub;
   //private Limelight limelight;

    private SensorCollection leftEncoder;
    private SensorCollection rightEncoder;

    private Timer timer = new Timer();

    private ADXRS450_Gyro gyro;


    //one ball auto
    public boolean starting;

    

     //true makes it turn right 90 degrees, false means no turn
    private boolean startPathweaver;

    //right encoder value is positive when going forward
    //left encoder value is negative when going forward

    public void oneBallAuto(double leftEncoderValue, double rightEncoderValue) {
        timer.reset();
        timer.start();
        if (!timer.hasElapsed(1.0)) {
            this.intakesub.intake.set(-Constants.intakeSpeed);
        } else {
            this.starting = false;
        }
    }

    public AutonomousNoTaxi(AutonomousSub autonomoussub1, DrivebaseSub drivebasesub1, Intake intake1) {
        autonomoussub = autonomoussub1;
        drivebasesub = drivebasesub1;
        intakesub = intake1;
        addRequirements(this.autonomoussub);
        addRequirements(this.drivebasesub);
        addRequirements(this.intakesub);
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
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftEncoderValue = this.leftEncoder.getQuadraturePosition();
        double rightEncoderValue = this.rightEncoder.getQuadraturePosition();
        SmartDashboard.putNumber("Left Encoder auto", Robot.m_robotContainer.encoderLeft.getQuadraturePosition());
        SmartDashboard.putNumber("Right Encoder auto", Robot.m_robotContainer.encoderRight.getQuadraturePosition());
        SmartDashboard.putNumber("Gyro", Robot.m_robotContainer.gyro.getAngle());
        if (this.starting) {
            oneBallAuto(leftEncoderValue, rightEncoderValue);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.starting = false;
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