package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class DriveBase extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(4);

    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0.001;

    private PIDController pid = new PIDController(kP, kI, kD);
    private AHRS sensor = new AHRS();
    private DifferentialDriveOdometry odometry;

    private static final double wheelRadius = 0.07; // 3 inches
    private static final double gearRatio = 25;

    
    public DriveBase(){
        // NavX
        sensor.zeroYaw();
        sensor.resetDisplacement();
        sensor.enableBoardlevelYawReset(true);

        odometry = new DifferentialDriveOdometry(sensor.getRotation2d(),
         rotationsToMeters(leftMotor.getPosition().getValueAsDouble()),
         rotationsToMeters(rightMotor.getPosition().getValueAsDouble())
        );

        // Motor configuration
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

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        pid.enableContinuousInput(-180.0, 180.0);
        pid.setTolerance(1.0);
    }

    private double rotationsToMeters(double motorRotations) {
        double wheelRotations = motorRotations / gearRatio;
        return wheelRotations * 2 * Math.PI * wheelRadius;
    }


    public void updateOdometry(){
        odometry.update(sensor.getRotation2d(),
         rotationsToMeters(leftMotor.getPosition().getValueAsDouble()),
         rotationsToMeters(rightMotor.getPosition().getValueAsDouble()));
         System.out.println(odometry.toString()
        );
    }

    private void move() {
        leftMotor.set(0.2);
        rightMotor.set(0.2);
    }

    public void stopMotors(){
        // instead of setting neutral mode to brake every time we want to stop, 
        // set once in constructor and set the motors to 0 whenever you want to stop
        // Spamming neutral mode wastes CAN bandwidth
  
        leftMotor.set(0);
        rightMotor.set(0);
    }

    private void rotate(double speed, double angle){
        System.out.println("speed: " + speed + "  angle given: " + angle + " yaw: " + sensor.getYaw());
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void rotateManual(double xAxis) {
        // Deadzone
        if (Math.abs(xAxis) > 0.2) {
         double rotationSpeed = xAxis * 0.6;
         // For turning in place, left and right motors must be opposite
            leftMotor.set(rotationSpeed);
            rightMotor.set(rotationSpeed);
        } else {
            // Stop motors when joystick is neutral
          stopMotors();
        }
    }

    public void driveManual(double speed) {
       // Forward/backward: triggers give values 0 -> 1
        if (Math.abs(speed) > 0.1) {
            leftMotor.set(speed);
            rightMotor.set(-speed);
        } else {
            stopMotors();
        }
    }
  
    
    


    public Command drive(double speed){
        return run(() -> this.move()).finallyDo(() -> this.stopMotors());
        //return run(() -> this.move());
    }

    public Command rotateToDegree(double rotateToAngle){
        //System.out.println(rotateToAngle);
        // return run(() -> rotate(
        //     pid.calculate(sensor.getAngle()%360, rotateToAngle), rotateToAngle)
        // );
        //return run(() -> rotate(pid.calculate(sensor.getYaw(), rotateToAngle)));

    return run(() -> {
        double pidOutput = pid.calculate(
            sensor.getYaw(),  // current angle
            rotateToAngle
        );

        pidOutput = MathUtil.clamp(pidOutput, -0.5, 0.5);
        rotate(pidOutput, rotateToAngle);

    }).until(() -> pid.atSetpoint())
      .finallyDo(() -> stopMotors());
    }
}
