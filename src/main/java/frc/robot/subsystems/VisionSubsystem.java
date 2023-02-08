package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight-avis");
    private NetworkTableEntry pipeline = limeLight.getEntry("pipeline");
    private double[] pos = new double[]{0, 0, 0, 0, 0, 0};
    private int id;
    private boolean visible;
    private Field2d field2d = new Field2d();

    public VisionSubsystem() {
        pipeline.setInteger(0);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision");

        shuffleboardTab.addDoubleArray("Robot pos", () -> pos);
        shuffleboardTab.addNumber("Tag ID", () -> id);
        shuffleboardTab.addBoolean("Tag visible", () -> visible);

    }

    @Override
    public void periodic() {
        pos = limeLight.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0});
        id = (int) limeLight.getEntry("tid").getInteger(0);
        visible = limeLight.getEntry("tv").getInteger(0) == 1;

    }
}
