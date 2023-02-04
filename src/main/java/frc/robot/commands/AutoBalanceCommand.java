package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;

public class AutoBalanceCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private AHRS gyro;
    private PID pidy = new PID(.8, 0, .1, 2, 0.1, Math.sin(Math.toRadians(1.25)));
    private PID pidz = new PID(.2,0,0, 2, 0.05, .7);
    //private PID pidx = new PID();


    public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
        pidy.setGoal(0);
        pidz.setGoal(0);

        gyro = driveSubsystem.gyro;

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double pitch = driveSubsystem.getGyroPitch();
        double roll = driveSubsystem.getGyroRoll();

        ChassisSpeeds speeds;

        if (Math.abs(roll) > .7) {
            speeds = new ChassisSpeeds(0, 0, Math.signum(pitch) * pidz.calculate(roll));
        } else if (Math.abs(pitch) > 0.25) { // Math.sin(Math.toRadians(1))
            speeds = new ChassisSpeeds(-pidy.calculate(Math.sin(Math.toRadians(pitch))), 0, 0);
        } else {
            speeds = new ChassisSpeeds(0, 0, 0);
        }

        driveSubsystem.drive(speeds);

    }
}
