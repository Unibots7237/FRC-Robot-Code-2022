package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    public CANSparkMax arm = new CANSparkMax(Constants.armSpark, MotorType.kBrushless);
    
    XboxController xboxcontroller = RobotContainer.xboxcontroller;
    
    public boolean armUp = false;
    public double lastBurstTime = 0;

    public void armControl() {
      if (xboxcontroller.getBButton()) {
        arm.set(Constants.armRise);
      }
      if (xboxcontroller.getXButton()) {
        arm.set(Constants.armDescend);   
      }
      if (!xboxcontroller.getBButton() && !xboxcontroller.getXButton()) {
        arm.set(0);
      }

      /*
      
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
    }        
    if(xboxcontroller.getXButtonPressed() && !armUp){
      System.out.println("x pressed");        
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    else if(xboxcontroller.getBButtonPressed() && armUp){
      System.out.println("y pressed");
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    } 
    */
  }
}

      
            
