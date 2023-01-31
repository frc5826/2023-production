package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLockCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    private double baseLockAngle, middleLockAngle;


    public ArmLockCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        this.baseLockAngle = armSubsystem.getBaseEncoderPosition();
        this.middleLockAngle = armSubsystem.getMiddleEncoderPosition();

        System.out.println("Arm lock init");
    }

    @Override
    public void execute() {
        super.execute();
        armSubsystem.setBasePosition(baseLockAngle);
        armSubsystem.setMiddlePosition(middleLockAngle);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Arm lock end");
        super.end(interrupted);
    }
}
