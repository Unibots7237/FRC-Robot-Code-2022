package frc.robot.commands;
import frc.robot.*;
import frc.robot.subsystems.DrivebaseSub;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Drivebase extends CommandBase {

    private DrivebaseSub drivebaseSub;

    public Drivebase(DrivebaseSub drivebasesub) {
        this.drivebaseSub = drivebasesub;
        addRequirements(this.drivebaseSub);
    }
    
    @Override
    public void initialize() {

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //double move = Robot.m_robotContainer.xboxcontroller.getLeftY();
        double turn = Robot.m_robotContainer.xboxcontroller.getLeftX();

        double accelerate = Robot.m_robotContainer.xboxcontroller.getRightTriggerAxis();
        double deaccelerate = Robot.m_robotContainer.xboxcontroller.getLeftTriggerAxis();
        double move = 0;

        if (accelerate > 0.0) {
            move = accelerate;
        }
        if (deaccelerate > 0.0) {
            move = -deaccelerate;
        }

        this.drivebaseSub.teleopDrive(move, turn);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}
