package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class DriveBase extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(4);
    private PIDController pid = new PIDController(0.01, 0.01, 0);
    private AHRS sensor = new AHRS();
    private DifferentialDriveOdometry odometry;
    
    public DriveBase(){
        sensor.zeroYaw();
        sensor.resetDisplacement();
        sensor.enableBoardlevelYawReset(true);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(70)
            .withStatorCurrentLimitEnable(true)
            )
        .withVoltage(
            new VoltageConfigs()
            .withPeakForwardVoltage(Units.Volts.of(10))
            .withPeakReverseVoltage(Units.Volts.of(-10))
        );
        leftMotor.getConfigurator().apply(talonFXConfiguration);
        rightMotor.getConfigurator().apply(talonFXConfiguration);
    }

    public void updateOdometry(){
        odometry.update(sensor.getRotation2d(),
         leftMotor.getPosition().getValueAsDouble()*(2*Math.PI*2.8),
         rightMotor.getPosition().getValueAsDouble()*(2*Math.PI*2.8));
        System.out.println(odometry.toString());
    }

    private void move(){
        leftMotor.set(0.2);
        rightMotor.set(0.2);
    }

    private void stopMotors(){
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.set(0);
        rightMotor.set(0);
    }

    private void rotate(double speed, double angle){
        System.out.println("speed: " + speed + "  angle given: " + angle + " yaw: " + sensor.getYaw());
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public Command drive(){
        return run(() -> this.move()).finallyDo(() -> this.stopMotors());
        //return run(() -> this.move());
    }

    public Command rotateToDegree(double rotateToAngle){
        //System.out.println(rotateToAngle);
        return run(() -> rotate(
            pid.calculate(sensor.getAngle()%360, rotateToAngle), rotateToAngle)
        );
        //return run(() -> rotate(pid.calculate(sensor.getYaw(), rotateToAngle)));
    }
}
