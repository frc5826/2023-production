package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRepositionCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    double mastMoveDeg, armMoveDeg;

    public ArmRepositionCommand(ArmSubsystem armSubsystem, double mastMoveDeg, double armMoveDeg){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

        this.armMoveDeg = armMoveDeg;
        this.mastMoveDeg = mastMoveDeg;
    }

    @Override
    public void initialize() {
        super.initialize();
        armSubsystem.setArmTargetRad((armSubsystem.getArmTargetAngle() + armMoveDeg) * (Math.PI / 180));
        armSubsystem.setMastTargetRad((armSubsystem.getMastTargetAngle() + mastMoveDeg) * (Math.PI / 180));
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
