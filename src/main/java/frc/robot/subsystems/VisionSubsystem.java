package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight-avis");
    private double verticalSize;
    private double angleToTape;
    private NetworkTableEntry pipeline = limeLight.getEntry("pipeline");
    public double[] pos = new double[]{0, 0, 0, 0, 0, 0};
    private int id;
    public boolean visible;
    private Field2d field2d = new Field2d();

    private double[] comboPos = new double[]{0, 0};
    private double[] reflectivePos = new double[]{0, 0};
    private double[] savePosRef = new double[]{0, 0};
    private double[] savePos = new double[]{0, 0, 0, 0, 0, 0};
    private double savePosX = 0;
    private double savePosY = 0;

    private int sees;

    private boolean isRefPipe = false;

    private int count = 0;

    public boolean switched = false;

    private double pipelineID;

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

        shuffleboardTab.addNumber("tape d", () -> getD());
        shuffleboardTab.addNumber("tape y", () -> getY());
        shuffleboardTab.addNumber("tape x", () -> getX());

        shuffleboardTab.addNumber("reflective x", () -> getReflectivePos()[0]);
        shuffleboardTab.addNumber("reflective y", () -> getReflectivePos()[1]);

    }

    @Override
    public void periodic() {
        pos = limeLight.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0});
        id = (int) limeLight.getEntry("tid").getInteger(0);

        pipelineID = limeLight.getEntry("getpipe").getDouble(-1);

        //visible = limeLight.getEntry("tv").getInteger(0) == 1;

        verticalSize = limeLight.getEntry("tvert").getDouble(0);
        angleToTape = limeLight.getEntry("tx").getDouble(0);

        sees = (int) limeLight.getEntry("tv").getInteger(0);

        if (limeLight.getEntry("tv").getInteger(0) == 1 && ((id <= 8 && id > 0) || id == -1)) {
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
            savePosRef[0] = getX() + Constants.llOffsetX;
            savePosRef[1] = getY() + Constants.llOffsetY;
            savePosX = DriveSubsystem.odometry.getPoseMeters().getX();
            savePosY = DriveSubsystem.odometry.getPoseMeters().getY();
        }

        setComboPos();

        setReflectivePos();

    }

    public double getD() {
        return .051 / (Math.tan((verticalSize / Constants.resHeight) * 25 * (Math.PI / 180)));
    }

    public double getY() {
        return getD() * Math.sin(angleToTape * (Math.PI / 180)) + Constants.yOffset;
    }

    public double getX() {
        return getD() * Math.cos(angleToTape * (Math.PI / 180)) + Constants.xOffset;
    }

    public void conePipeline() {
        pipeline.setDouble(1);
        isRefPipe = true;
    }

    public void cubePipeline() {
        pipeline.setDouble(0);
        isRefPipe = false;
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

    public boolean isPipeline() {
//        if (id == -1 && sees == 1 && isRefPipe || (id <= 8 && id > 0) && sees == 1 && isRefPipe == false) {
//            return true;
//        } else {
//            return false;
//        }
        return pipelineID == pipeline.getDouble(-1);
    }

    private void setReflectivePos() {
        if (visible) {
            reflectivePos[0] = getX() + Constants.llOffsetX;
            reflectivePos[1] = getY() + Constants.llOffsetY;
        } else {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                reflectivePos[0] = savePosRef[0] + (DriveSubsystem.odometry.getPoseMeters().getX() - savePosX);
                reflectivePos[1] = savePosRef[1] - (DriveSubsystem.odometry.getPoseMeters().getY() - savePosY);
            } else {
                reflectivePos[0] = savePosRef[0] - (DriveSubsystem.odometry.getPoseMeters().getX() - savePosX);
                reflectivePos[1] = savePosRef[1] + (DriveSubsystem.odometry.getPoseMeters().getY() - savePosY);
            }
        }
    }

    public double[] getReflectivePos() {
        return reflectivePos;
    }
}