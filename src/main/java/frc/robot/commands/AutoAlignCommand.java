package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean finished = false;

    private double[] pos = new double[]{0, 0/*, 0, 0, 0, 0*/};

    private PID pidy = new PID(1.25, 0, 0.015, 0.6, 0, 0.01);
    private PID pidx = new PID(1.25, 0, 0.015, 0.6, 0, 0.01);
    private PID pidTurn = new PID(0.1, 0, 0.004, 1, 0, 1); //TODO turn speed back up

    private double[] cubegoalY = new double[]{1.07, 2.75, 4.425};
    private double[] conegoalY = new double[]{0.51, 1.63, 2.18, 3.3, 3.86, 4.98};
    private double targetY;
    private double targetX;
    private double poleX;

    private boolean iscube;

    private int invertDrive = 1;
    private int turnOffset = 0;

    private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    double time = 100;

    public AutoAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean iscube, double time) {
        this(driveSubsystem, visionSubsystem, iscube);
        this.time = time;
    }

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
            visionSubsystem.cubePipeline();
        } else{
            targetY = getClosestGoal(conegoalY);
            visionSubsystem.cubePipeline();
        }

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            targetX = 1.825;
            poleX = targetX - 1;
            //invertDrive = -1;//TODO
            turnOffset = 180;
        } else {
            targetX = 14.725;
            poleX = targetX + 1;
            //invertDrive = 1;
            turnOffset = 0;
        }

        if (iscube) {
            pidx.setGoal(targetX);
            pidy.setGoal(targetY);
        } else {
            pidx.setGoal(1);
            pidy.setGoal(0);
        }


        finished = false;

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        pos = visionSubsystem.getComboPos();
        double angleDifference = driveSubsystem.getRotation().getDegrees() - turnOffset;

        if(angleDifference < -180){
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        ChassisSpeeds speeds;
        if (iscube) {
            speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(
                    pidx.calculate(pos[0]) * invertDrive,
                    pidy.calculate(pos[1]) * invertDrive,
                    pidTurn.calculate(angleDifference),

                    driveSubsystem.getRotation()
            );
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    pidx.calculate(visionSubsystem.getReflectivePos()[0]),
                    pidy.calculate(visionSubsystem.getReflectivePos()[1]),
                    pidTurn.calculate(angleDifference),
                    driveSubsystem.getRotation()
            );
        }


        driveSubsystem.drive(speeds);

        double deadband = 0.025;

        if (pos[1] < targetY + deadband && pos[1] > targetY - deadband && pos[0] < targetX + deadband && pos[0] > targetX - deadband) {
            finished = true;
        }

    }

    @Override
    public boolean isFinished() {
        if (timer.get() > time) {
            finished = true;
        }

        return finished;
    }

    @Override
    public void end(boolean yeah) {
        timer.stop();
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
