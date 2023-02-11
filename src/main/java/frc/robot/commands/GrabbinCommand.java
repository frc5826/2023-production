package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabbinSubsystem;

public class GrabbinCommand extends CommandBase {

    GrabbinSubsystem grabbinSubsystem;

    public GrabbinCommand(GrabbinSubsystem grabbinSubsystem){
        this.grabbinSubsystem = grabbinSubsystem;
        addRequirements(grabbinSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        grabbinSubsystem.toggle();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
