package frc.robot.commands;
import frc.robot.*;
import frc.robot.subsystems.AutonomousSub;
import frc.robot.subsystems.DrivebaseSub;
import frc.robot.subsystems.Limelight;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Autonomous extends CommandBase {

    private DrivebaseSub drivebasesub;
    private AutonomousSub autonomoussub;
    private Limelight limelight;

    private SensorCollection leftEncoder;
    private SensorCollection rightEncoder;

    public boolean movingForward;
    public boolean starting; //when it moves back in the beginning
 
     //true makes it turn right 90 degrees, false means no turn
     private boolean findCargo;

    public Autonomous(AutonomousSub autonomoussub1, DrivebaseSub drivebasesub1, Limelight limelight1) {
        autonomoussub = autonomoussub1;
        drivebasesub = drivebasesub1;
        limelight = limelight1;
        addRequirements(this.autonomoussub);
        addRequirements(this.drivebasesub);
        addRequirements(this.limelight);
        this.leftEncoder = drivebasesub.encoderLeft;
        this.rightEncoder = drivebasesub.encoderRight;
        this.limelight = limelight;
    }
    
    @Override
    public void initialize() {
        leftEncoder.setQuadraturePosition(0,0);
        rightEncoder.setQuadraturePosition(0,0);
        this.movingForward = true;
        this.starting = true;
        this.findCargo = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftEncoderValue = leftEncoder.getQuadraturePosition();
        double rightEncoderValue = -rightEncoder.getQuadraturePosition();

        SmartDashboard.putNumber("leftEncoder_auto", leftEncoderValue);
        SmartDashboard.putNumber("rightEncoder_auto", rightEncoderValue);

        

        if (starting) {
            /*
            if (leftEncoderValue > -15000) {
                this.drivebasesub.driveLeft(-Constants.autonomousSpeed);
            }
            if (rightEncoderValue > -15000) {
                this.drivebasesub.driveRight(Constants.autonomousSpeed);
            }
            if (leftEncoderValue <= -15000 && rightEncoderValue <= -15000) {
                this.drivebasesub.encoderLeft.setQuadraturePosition(0, 0);
                this.drivebasesub.encoderRight.setQuadraturePosition(0, 0);
                starting = false;
                findCargo = true;
            }
            */
            findCargo = true;
         }
        if (findCargo) {
            if (this.limelight.haveValidTarget() == false) {
                if (this.limelight.horizontalOffset() < 0.0) {
                    System.out.println("negative degree");
                }
                if (this.limelight.horizontalOffset() > 0.5) {
                    System.out.println("positive degree");
                }
                if (this.limelight.horizontalOffset() <= 0.5 && this.limelight.horizontalOffset() >= 0.0) {
                   System.out.println("move forward"); 
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //leftEncoder.setQuadraturePosition(0,0);
        //rightEncoder.setQuadraturePosition(0,0);
        this.movingForward = true;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}