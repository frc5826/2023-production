package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean finished = false;

    private double[] pos = new double[]{0, 0, 0, 0, 0, 0};
    private double xGoal = 1.85;
    private double zGoal = 1;

    private PID pidz = new PID(.5, 0, 0, 0.7, 0.15, 0.1);
    private PID pidx = new PID(.4, 0, 0, 0.5, 0.15, 0.1);
    private PID pidTurn = new PID(.4, 0, 0, 0.5, 0.15, 0.1);

    public AutoAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        pidz.setGoal(zGoal);
        pidx.setGoal(xGoal);
        pidTurn.setGoal(0); //might be wrong if on the other side of field

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {

        if (visionSubsystem.visible) {
            pos = visionSubsystem.pos;
        }

        ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(-pidx.calculate(pos[0]), -pidz.calculate(pos[1]), 0 /* pidTurn.calculate(pos[3]) */, driveSubsystem.getRotation());
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

}
