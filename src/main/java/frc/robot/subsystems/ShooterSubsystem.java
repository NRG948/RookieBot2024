package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  //TO-DO: Check if deviceIds are correct (motors, encoders)
    private final TalonFX rightMotor = new TalonFX(1); // Add actual TalonFX device ids
    private final TalonFX leftMotor = new TalonFX(2);// Add actual TalonFX device ids
    private final CANcoder rightEncoder = new CANcoder(0); // Add actual CAN device ids
    private final CANcoder leftEncoder = new CANcoder(1); // Add actual CAN device ids

    private boolean isEnabled = false;
    private double rightCurrentVelocity;
    private double leftCurrentVelocity;

    public ShooterSubsystem() {
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setInverted(true);
        // Possible Conversion Factor here for encoder
    }
    public void setMotorVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
        leftMotor.setVoltage(voltage);
        isEnabled = true;
    }
    public void setGoalRPM(double goalRPM) {
        isEnabled = true;
        rightMotor.set(goalRPM); // from 0 to 1
        leftMotor.set(goalRPM); // from 0 to 1
    }
    public void disable() {
        isEnabled = false;
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }
    public void stopMotor() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }
    @Override
    public void periodic() {
        rightCurrentVelocity = rightEncoder.getVelocity().getValueAsDouble();
        leftCurrentVelocity = leftEncoder.getVelocity().getValueAsDouble();
    }
    
    public void addShuffleBoardLayout(ShuffleboardTab tab){
        ShuffleboardLayout layout =  
            tab.getLayout("Shooter", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
        layout.addDouble("Right Motor Current Velocity", () -> rightCurrentVelocity);
        layout.addDouble("Left Motor Current Velocity", () -> leftCurrentVelocity);
        layout.addBoolean("Enabled", () -> isEnabled);
    }
}