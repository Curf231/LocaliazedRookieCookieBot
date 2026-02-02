// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveBase m_DriveBase = new DriveBase();
  private final PS5Controller m_driverController = new PS5Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // NOTE: getPOV() returns -1 if nothing is pressed
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // new Trigger(() ->m_driverController.getPOV() != -1)
    // .onTrue(
    //   m_DriveBase.rotateToDegree(m_driverController.getPOV())
    //   );

    // new Trigger(() ->
    //     m_driverController.getCrossButton()
    //     && m_driverController.getPOV() == -1
    // ).whileTrue(
    //     m_DriveBase.drive()
    // );




    new Trigger(() -> m_driverController.getR2Axis() > 0.1)
        .whileTrue(
            m_DriveBase.run(() ->
                m_DriveBase.drive(m_driverController.getR2Axis())
            )
        );

    new Trigger(() -> m_driverController.getL2Axis() > 0.1)
        .whileTrue(
            m_DriveBase.run(() ->
                m_DriveBase.drive(-m_driverController.getL2Axis())
            )
        );

    new Trigger(() -> Math.abs(m_driverController.getLeftX()) > 0.3)
        .whileTrue(
            m_DriveBase.run(() ->
                m_DriveBase.rotateManual(m_driverController.getLeftX())
            )
        );
  

    // new Trigger(() -> m_driverController.getPOV() == 0)
    // .and(() -> m_driverController.getCrossButton()).whileTrue(
    //   m_DriveBase.drive()
    // );
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public DriveBase getDriveBase(){
    return m_DriveBase;
  }
}
