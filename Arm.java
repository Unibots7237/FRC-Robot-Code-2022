package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    public CANSparkMax arm = new CANSparkMax(Constants.armSpark, MotorType.kBrushless);
    
    XboxController xboxcontroller = RobotContainer.xboxcontroller;
    
    public static boolean dropDown = false;
    public static boolean controlledDescent = false;

    public static Timer timer = new Timer();

    public boolean armUp = false;
    public double lastBurstTime = 0;

    public void armRise() {
      arm.set(Constants.armRise);
    } 
    public void armDescend() {
      arm.set(Constants.armDescend);
    }
    public void armControlledDescent() {
      arm.set(Constants.armControlledDescent);
    }

    public void armControl() {
      arm.setIdleMode(IdleMode.kBrake);
      if (xboxcontroller.getBButton()) {
        //arm.set(Constants.armDescend);
        

        if (!dropDown) {
          timer.start();
          dropDown = true;
        }
        
        
      }
      if (xboxcontroller.getXButton()) {
        //arm.set(Constants.armRise);
              
        if (!dropDown) {
          arm.set(Constants.armRise);   
        }
        
      }

      
      if (!xboxcontroller.getBButton() && !xboxcontroller.getXButton()) {
        //arm.set(0);
        
        if (!dropDown) {
          arm.set(0);
        }
        
      }

      if (xboxcontroller.getRightStickButton()) {
        dropDown = false;
        controlledDescent = false;
      }

      
      if (dropDown && !controlledDescent) {
        if (!timer.hasElapsed(.075)) {
          arm.set(Constants.armDescend);
        }
        if (timer.hasElapsed(.075)) {
          controlledDescent = true;
        }
      }
      if (controlledDescent) {
        timer.reset();
        if (!timer.hasElapsed(1.0)) {
          arm.set(Constants.armControlledDescent);
        }
        if (timer.hasElapsed(1.0)) {
          timer.stop();
          timer.reset();
          dropDown = false;
          controlledDescent = false;
        }
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

      
            
