//Yes I just yoinked this from the WaitCommand

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * To those whomst may be looking at this command and saying to yourself "What the hell is this for?"
 * I'll spare you the time; this is a repurposed "wait" command that doesn't wait if our arm is
 * high enough
*/
public class MastWaitCommand extends CommandBase {

    ArmSubsystem armSubsystem;
    protected Timer timer = new Timer();
    private final double duration;

    private double cutoff;


    public MastWaitCommand(ArmSubsystem armSubsystem, double seconds, double cutoff) {
        duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
        this.cutoff = cutoff;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration) || armSubsystem.getArmDeg() >= cutoff;
    }
}
