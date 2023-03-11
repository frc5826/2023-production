package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabbinSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SetupGyroCommand extends CommandBase {

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    GrabbinSubsystem grabbinSubsystem;

    private boolean finished = false;

    public SetupGyroCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, GrabbinSubsystem grabbinSubsystem)
    {
        this.grabbinSubsystem = grabbinSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem, grabbinSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetOdometryButCool(visionSubsystem.getComboPos());

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            driveSubsystem.invertGyroYaw();
            driveSubsystem.zeroDriveGyro();
        } else {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.invertZeroDriveGyro();
        }

        driveSubsystem.zeroGyroRollPitch();

        grabbinSubsystem.close();

        finished = true;
    }
    
    @Override
    public boolean isFinished() { return finished; }
}
