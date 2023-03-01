package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class GrabbinSubsystem extends SubsystemBase {

    private DoubleSolenoid solenoid;

    public GrabbinSubsystem(){
        solenoid = new DoubleSolenoid(cCompressor,PneumaticsModuleType.CTREPCM, cSolenoidFWD, cSolenoidREV);
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void open(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle(){
        solenoid.toggle();
    }

}
