package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase {

    public NetworkTable table;

    public void Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean haveValidTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getBoolean(false);
    }

    public double horizontalOffset() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    public double verticalOffset() {
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    public double targetArea() {
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0.0);
    }
}