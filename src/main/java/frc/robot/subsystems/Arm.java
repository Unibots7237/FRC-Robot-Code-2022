package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    public WPI_TalonSRX arm = new WPI_TalonSRX(Constants.armTalon);
    
    XboxController xboxcontroller = RobotContainer.xboxcontroller;
    
    public boolean armUp = true;
    public double lastBurstTime = 0;

    public void armControl() {
      if(armUp){
        if(Timer.getFPGATimestamp() - lastBurstTime < Constants.armTimeUp){
          arm.set(Constants.armTravel);
        }
        else{
          arm.set(Constants.armHoldUp);
        }
      }
      else{
        if(Timer.getFPGATimestamp() - lastBurstTime < Constants.armTimeDown){
          arm.set(-Constants.armTravel);
        }
        else{
          arm.set(-Constants.armHoldDown);
        }
      
      if(xboxcontroller.getXButtonPressed() && !armUp){
        lastBurstTime = Timer.getFPGATimestamp();
        armUp = true;
      }
      else if(xboxcontroller.getBButtonPressed() && armUp){
        lastBurstTime = Timer.getFPGATimestamp();
        armUp = false;
      } 
    }
  }
}

      
            
