// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveBase m_DriveBase = new DriveBase();
  // Two Logitech Attack 3 joysticks: left controls left wheel, right controls right wheel
  private final CommandJoystick m_leftJoystick =
    new CommandJoystick(OperatorConstants.kLeftJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_leftJoystick.axisGreaterThan(0, 0.1).or(m_leftJoystick.axisGreaterThan(1, 0.1))
    .whileTrue(m_DriveBase.rotateToDegree(Math.atan(m_leftJoystick.getY()/m_leftJoystick.getX())));

    (m_leftJoystick.axisLessThan(0, 0.1))
    .whileTrue(m_DriveBase.drive());
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
