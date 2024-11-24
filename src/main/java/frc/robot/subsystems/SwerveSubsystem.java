// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  
  private final TalonFX frontLeftDriveMotor = new TalonFX(8);
  private final CANSparkMax frontLeftSteerMotor = new CANSparkMax(9, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX frontRightDriveMotor = new TalonFX(19);
  private final CANSparkMax frontRightSteerMotor = new CANSparkMax(14, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX backLeftDriveMotor = new TalonFX(6);
  private final CANSparkMax backLeftSteerMotor = new CANSparkMax(7, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX backRightDriveMotor = new TalonFX(13);
  private final CANSparkMax backRightSteerMotor = new CANSparkMax(18, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  
  //4 CANcoders to give us Angles
  private final CANcoder frontLeftAngle = new CANcoder(33);
  private final CANcoder frontRightAngle = new CANcoder(34);
  private final CANcoder backLeftAngle = new CANcoder(32);
  private final CANcoder backRightAngle = new CANcoder(31);

  private final SwerveModule frontLeftModule = createSwerveModule(backLeftDriveMotor, backLeftSteerMotor, backLeftAngle, 0, "FrontLeftModule");
  //TO-DO FINISH the method with the return statement
  private static SwerveModule createSwerveModule(
    TalonFX driveMotor,
    CANSparkMax steeringMotor,
    CANcoder wheelAngle,
    double angleOffset,
    String name) {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      steeringMotor.setIdleMode(IdleMode.kBrake);
      steeringMotor.setInverted(false);
      CANcoderConfigurator wheelAngleConfigurator = wheelAngle.getConfigurator();
      CANcoderConfigurator wheelAngleConfig = new CANcoderConfigurator(null);

      final double metersPerRotation = 4 * Math.PI / 8.14;
      StatusSignal<Double> drivePosition = driveMotor.getPosition();
      StatusSignal<Double> driveVelocity = driveMotor.getVelocity();
      StatusSignal<Double> WheelDirection = wheelAngle.getAbsolutePosition();
      StatusSignal<Double> AngularVelocity = wheelAngle.getVelocity();
      // change constant for SwerveModule
      return new SwerveModule(null, name);

    }

  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
