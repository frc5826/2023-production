package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean finished = false;

    private double[] pos = new double[]{0, 0/*, 0, 0, 0, 0*/};

    private PID pidy = new PID(1.25, 0, 0.015, 1, 0, 0.005);
    private PID pidx = new PID(1.25, 0, 0.015, 0.7, 0, 0.005);
    private PID pidTurn = new PID(0.1, 0, 0.004, 2, 0, 1);

    private double[] cubegoalY = new double[]{1, 2.75, 4.4};
    private double[] conegoalY = new double[]{0.5, 1.6, 2.2, 3.3, 3.85, 4.95};
    private double targetY;
    private double targetX;

    private boolean iscube;

    private int invertDrive = 0;

    public AutoAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean iscube) {
        pidx.setGoal(targetX);
        pidy.setGoal(targetY);
        pidTurn.setGoal(0); //might be wrong if on the other side of field

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);

        //ShuffleboardTab Tab = Shuffleboard.getTab("Vision");
        //Tab.addNumber("TargetX", () -> targetX);
        //Tab.addNumber("TargetY", () -> targetY);
        //Tab.addNumber("PIDY", pidy::getError);
        //Tab.addNumber("PIDX", pidx::getError);

        this.iscube = iscube;
    }

    @Override
    public void initialize() {
        if (iscube) {
            targetY = getClosestGoal(cubegoalY);
        }else{
            targetY = getClosestGoal(conegoalY);
        }

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            targetX = 1.8;
            invertDrive = 1; //TODO
        } else {
            targetX = 14.75;
            invertDrive = -1;
        }

        pidx.setGoal(targetX);
        pidy.setGoal(targetY);

        finished = false;
    }

    @Override
    public void execute() {

        System.out.println("current goal: " + targetX + ", " + targetY);

        pos = visionSubsystem.getComboPos();
        double angleDifference = driveSubsystem.getRotation().getDegrees() - 180;

        if(angleDifference < 180){
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(
                pidx.calculate(pos[0]) * invertDrive,
                pidy.calculate(pos[1]) * invertDrive,
                pidTurn.calculate(driveSubsystem.getRotation().getDegrees()),

                driveSubsystem.getRotation()
        );

        driveSubsystem.drive(speeds);

        if (pos[1] < targetY + 0.01 && pos[1] > targetY - 0.01 && pos[0] < targetX + 0.01 && pos[0] > targetX - 0.01) {
            finished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

//    public double getTargetY() {
//        double distance = Math.abs(Constants.yCoordDrop[0] - visionSubsystem.getComboPos()[1]);
//    }

    public double getClosestGoal(double[] goalY) {
        double y = visionSubsystem.getComboPos()[1];
        double distance = 10;
        double yeah = y;

        for (int i = 0; i < goalY.length; i++) {
            if (Math.abs(goalY[i] - y) < distance) {
                distance = Math.abs(goalY[i] - y);
                yeah = goalY[i];
            }
        }

        return yeah;
    }

}
