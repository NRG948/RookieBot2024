// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUsingController extends Command {
  private final IntakeSubsystem intake;
  private final CommandXboxController controller;
  private static double DEADBAND = 0.1;

  /** Creates a new IntakeUsingController. */
  public IntakeUsingController(IntakeSubsystem intake, CommandXboxController controller) {
    this.intake = intake;
    this.controller = controller;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -controller.getHID().getRightY(); // converts physical joystick movement into speed
    speed = MathUtil.applyDeadband(speed, DEADBAND); // compensates for stick drift (slight movement in joystick)

    double voltage = speed * 12; // battery operates at 12 volts so multiplying by speed gives voltage

    intake.setMotorVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
