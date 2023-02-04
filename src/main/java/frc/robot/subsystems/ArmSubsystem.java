package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private WPI_TalonSRX mastMotor, armMotor;

    private DutyCycleEncoder mastEncoder, armEncoder;

    public ArmSubsystem(){

        mastMotor = new WPI_TalonSRX(cBaseArmID);
        mastMotor.setNeutralMode(NeutralMode.Brake);

        mastMotor.configPeakOutputForward(0.4, cTimeoutMs);
        mastMotor.configPeakOutputReverse(-0.4, cTimeoutMs);

        armMotor = new WPI_TalonSRX(cMiddleArmID);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.configPeakOutputForward(0.4, cTimeoutMs);
        armMotor.configPeakOutputReverse(-0.4, cTimeoutMs);


        mastEncoder = new DutyCycleEncoder(9);
        armEncoder = new DutyCycleEncoder(8);

    }

    public double encoderToRad(double encoderReading, double encoderOffset){
        return (encoderReading - encoderOffset) * (Math.PI / cArmClicksPerRotation);
    }

    public void setMastRad(double position){
        mastMotor.set(ControlMode.Position, Math.min(Math.max(position * (cArmClicksPerRotation /Math.PI) + cMastEncoderOffset, cMastEncoderMin), cMastEncoderMax));
    }

    public void setArmRad(double position){
        armMotor.set(ControlMode.Position, Math.min(Math.max(position * (cArmClicksPerRotation /Math.PI) + cArmEncoderOffset, cArmEncoderMin), cArmEncoderMax));
    }

    public void setBaseSpeed(double speed){
        mastMotor.set(speed);
    }

    public void setMiddleSpeed(double speed){
        armMotor.set(speed);
    }

    public void setBasePosition(double position){
        mastMotor.set(ControlMode.Position, position);
    }

    public void setMiddlePosition(double position){
        armMotor.set(ControlMode.Position, position);
    }

    public double getBaseEncoderPosition(){
        return mastMotor.getSelectedSensorPosition(cArmPIDLoopID);
    }

    public double getMiddleEncoderPosition(){
        return armMotor.getSelectedSensorPosition(cArmPIDLoopID);
    }

}
