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
    public double[] pos = new double[]{0, 0, 0, 0, 0, 0};
    private int id;
    public boolean visible;
    private Field2d field2d = new Field2d();

    private double[] comboPos = new double[]{0, 0};
    private double[] savePos = new double[]{0, 0, 0, 0, 0, 0};
    private double savePosX = 0;
    private double savePosY = 0;

    private int count = 0;

    public VisionSubsystem() {
        pipeline.setDouble(0);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision");

        shuffleboardTab.addDoubleArray("Robot pos", () -> pos);
        shuffleboardTab.addNumber("Tag ID", () -> id);
        shuffleboardTab.addBoolean("Tag visible", () -> visible);

        shuffleboardTab.addNumber("comboPosX", () -> comboPos[0]);
        shuffleboardTab.addNumber("comboPosY", () -> comboPos[1]);
        shuffleboardTab.addNumber("posX", () -> pos[0]);
        shuffleboardTab.addNumber("posY", () -> pos[1]);

        shuffleboardTab.addNumber("rotation x", () -> pos[5]);
    }

    @Override
    public void periodic() {
        pos = limeLight.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0});
        id = (int) limeLight.getEntry("tid").getInteger(0);

        //visible = limeLight.getEntry("tv").getInteger(0) == 1;

        if (limeLight.getEntry("tv").getInteger(0) == 1 && id <= 8 && id > 0) {
            count++;
            if (count > 5) {
                visible = limeLight.getEntry("tv").getInteger(0) == 1;
            }
        } else {
            count = 0;
            visible = false;
        }

        if (visible) {
            savePos = pos;
            savePosX = DriveSubsystem.odometry.getPoseMeters().getX();
            savePosY = DriveSubsystem.odometry.getPoseMeters().getY();
        }

        setComboPos();

    }

    public double[] getComboPos() {

        return comboPos;
    }

    private void setComboPos() {
        if (visible) {
            comboPos[0] = pos[0];
            comboPos[1] = pos[1];
        } else {
            comboPos[0] = savePos[0] + (DriveSubsystem.odometry.getPoseMeters().getX() - savePosX);
            comboPos[1] = savePos[1] + (DriveSubsystem.odometry.getPoseMeters().getY() - savePosY);
        }

    }
}
