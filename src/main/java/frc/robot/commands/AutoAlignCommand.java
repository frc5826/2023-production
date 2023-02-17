package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean finished = false;

    private double[] pos = new double[]{0, 0/*, 0, 0, 0, 0*/};
    private double xGoal = 0;
    private double zGoal = 0;

    private PID pidz = new PID(.5, 0, 0, 0.7, 0.15, 0.1);
    private PID pidx = new PID(.4, 0, 0, 0.5, 0.15, 0.1);
    private PID pidTurn = new PID(.4, 0, 0, 0.5, 0.15, 0.1);

    private double[] goalY = new double[]{0.5, 1, 1.6, 2.2, 2.75, 3.3, 3.85, 4.4, 4.95};
    private double targetY;
    private double targetX;

    public AutoAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        pidz.setGoal(targetX);
        pidx.setGoal(targetY);
        pidTurn.setGoal(0); //might be wrong if on the other side of field

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        targetY = getClosestGoal();

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            targetX = 1.8;
        } else {
            targetX = 14.75;
        }

        finished = false;
    }

    @Override
    public void execute() {

        System.out.println("current goal: " + targetX + ", " + targetY);

        if (visionSubsystem.visible) {
            pos = visionSubsystem.getComboPos();
        }

        ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(-pidx.calculate(pos[0]), -pidz.calculate(pos[1]), 0 /*pidTurn.calculate(visionSubsystem.pos[5])*/, driveSubsystem.getRotation());
        driveSubsystem.drive(speeds);

        if (pos[1] < zGoal + 0.05 && pos[1] > zGoal - 0.05 && pos[0] < xGoal + 0.05 && pos[0] > xGoal - 0.05) {
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

    public double getClosestGoal() {
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
