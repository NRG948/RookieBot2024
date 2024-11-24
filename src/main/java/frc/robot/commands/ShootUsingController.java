// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootUsingController extends Command {
  private final ShooterSubsystem shooter;
  private final CommandXboxController controller;
  private static double DEADBAND = 0.1;
  /** Creates a new ShootUsingController. */
  public ShootUsingController(ShooterSubsystem shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -controller.getHID().getLeftY(); // converts physical joystick movement into speed
    speed = MathUtil.applyDeadband(speed, DEADBAND); // compensates for stick drift (slight movement in joystick)
    
    double voltage = speed * 12; // battery operates at 12 volts so multiplying by speed gives voltage

    shooter.setMotorVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
