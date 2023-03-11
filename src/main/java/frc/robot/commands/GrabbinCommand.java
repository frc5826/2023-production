package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabbinSubsystem;

public class GrabbinCommand extends CommandBase {

    GrabbinSubsystem grabbinSubsystem;
    private GrabType grabType;

    public GrabbinCommand(GrabbinSubsystem grabbinSubsystem, GrabType grabType){
        this.grabbinSubsystem = grabbinSubsystem;
        this.grabType = grabType;
        addRequirements(grabbinSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        switch (grabType){
            case OPEN:
                grabbinSubsystem.open();
                break;
            case CLOSE:
                grabbinSubsystem.close();
                break;
            case TOGGLE:
                grabbinSubsystem.toggle();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
