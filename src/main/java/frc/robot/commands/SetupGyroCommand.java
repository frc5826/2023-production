package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SetupGyroCommand extends CommandBase {

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;

    private boolean finished = false;

    public SetupGyroCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) { //TODO might be red
            driveSubsystem.invertGyroYaw();
        } else {
            driveSubsystem.zeroGyroYaw();
        }

        driveSubsystem.resetOdometryButCool(visionSubsystem.getComboPos());
        finished = true;
    }
    
    @Override
    public boolean isFinished() { return finished; }
}
