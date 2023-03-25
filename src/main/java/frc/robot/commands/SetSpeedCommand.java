package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SetSpeedCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    private double speedInput;

    public SetSpeedCommand(DriveSubsystem driveSubsystem, double speedInput) {
        this.driveSubsystem = driveSubsystem;
        this.speedInput = speedInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Constants.cDriveSpeed = speedInput;
    }

    @Override
    public boolean isFinished() { return true; }
}
