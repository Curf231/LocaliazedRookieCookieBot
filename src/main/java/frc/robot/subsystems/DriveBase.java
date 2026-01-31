package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private VoltageOut voltage = new VoltageOut(0);
    private PIDController pid = new PIDController(0.01, 0.01, 0);
    private AHRS sensor = new AHRS();
    private DifferentialDriveOdometry odometry;

    
/*  navX.getYaw(); gets the z rotation in terms of [-180, 180]. This will save effort for tank drive player-oriented controller
        Better than the opposition, being navX.getAngle(); which returns the TOTAL yaw degrees rotated. Useless for tracking.
 * navx.zeroYaw(); sets the current rotation as the zero degrees state
 * navX.enableBoardlevelYawReset(boolean); allows hardware resets of the yaw. prob used for testing but turned off for actual robot code.
 * navX.resetDisplacement(); resets the coordinates...
 * 
 */
    
    public DriveBase(){
        sensor.zeroYaw();
        sensor.resetDisplacement();
        sensor.enableBoardlevelYawReset(true);
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        leftMotor.getConfigurator().apply(configuration);
        rightMotor.getConfigurator().apply(configuration);
        
        odometry.update(sensor.getRotation2d(),
         leftMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8),
         rightMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8));
    }

    private void move(){
        leftMotor.set(0.1);
        rightMotor.set(-0.1);
        odometry.update(sensor.getRotation2d(),
         leftMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8),
         rightMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8));
    }

    private void stopMotors(){
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.set(0);
        rightMotor.set(0);
    }

    private void rotate(double speed){
        //System.out.println(speed);
        leftMotor.set(speed/360);
        rightMotor.set(speed/360);
        odometry.update(sensor.getRotation2d(),
         leftMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8),
         rightMotor.getPosition().getValueAsDouble()/(2*Math.PI*2.8));
    }

    public Command drive(){
        return run(() -> this.move()).finallyDo(() -> this.stopMotors());
    }

    public Command rotateToDegree(double rotateToAngle){
        System.out.println(rotateToAngle);
        return run(() -> rotate(pid.calculate(sensor.getYaw(), rotateToAngle))).finallyDo(() -> this.stopMotors());
    }
}
