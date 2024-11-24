package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(1);
    private final CANcoder encoder = new CANcoder(0);

    private boolean isEnabled = false;
    private double currentVelocity;
 
    public static double GEAR_RATIO = 1; //constant (change value later)
    public static double INTAKE_DIAMETER = 0.036; //diameter in meters
    public static double ENCODER_CONVERSION_FACTOR = 
        (Math.PI * INTAKE_DIAMETER) / GEAR_RATIO; 

    public static double FREE_SPEED_RPM = 6000;
    public static double MAX_VELOCITY = 
        (FREE_SPEED_RPM * Math.PI * INTAKE_DIAMETER) / (GEAR_RATIO * 60);

    public static double MAX_ACCELERATION = 
        (2
                * FREE_SPEED_RPM
                * GEAR_RATIO
                * Math.PI
                * INTAKE_DIAMETER) 
            / 0.5;

    public IntakeSubsystem() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);
        // Possible Conversion Factor here for encoder
    }
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
        isEnabled = true;
    }
    public void in() {
        isEnabled = true;
        motor.set(0.8); // from 0 to 1
    }
    public void out() {
        isEnabled = true;
        motor.set(-0.8); // from -1 to 0
    }
    public void disable() {
        isEnabled = false;
        motor.stopMotor();
    }
    public void stopMotor() {
        motor.stopMotor();
    }
    @Override
    public void periodic() {
        currentVelocity = encoder.getVelocity().getValueAsDouble();
    }
    
    public void addShuffleBoardLayout(ShuffleboardTab tab){
        ShuffleboardLayout layout = 
            tab.getLayout("Intake", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
        layout.addDouble("Current Velocity", () -> currentVelocity);
        layout.addBoolean("Enabled", () -> isEnabled);
    }
}