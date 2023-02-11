package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class GrabbinSubsystem extends SubsystemBase {

    private DoubleSolenoid solenoid;

    public GrabbinSubsystem(){
        solenoid = new DoubleSolenoid(cCompresser ,PneumaticsModuleType.CTREPCM, cSolenoidFWD, cSolenoidREV);
    }

    public void extend(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle(){
        solenoid.toggle();
    }

}
