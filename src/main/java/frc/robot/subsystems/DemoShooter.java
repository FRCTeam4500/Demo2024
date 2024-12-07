package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import static frc.robot.CANConstants.*;

public class DemoShooter extends SubsystemBase {
    private CANSparkMax tiltMotor;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax loaderMotor;

    public DemoShooter() {
        rightMotor = new CANSparkMax(SHOOTER_ONE_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(SHOOTER_TWO_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(SHOOTER_PIVOT_ID, MotorType.kBrushless);
        loaderMotor = new CANSparkMax(LOADER_ID, MotorType.kBrushless);

        rightMotor.getPIDController().setP(0.3);
        tiltMotor.getPIDController().setP(3);
        tiltMotor.getPIDController().setOutputRange(-0.5, 0.3);
        tiltMotor.setIdleMode(IdleMode.kCoast);

        loaderMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.setSmartCurrentLimit(30);
        leftMotor.setSmartCurrentLimit(30);
        tiltMotor.setSmartCurrentLimit(30);
        loaderMotor.setSmartCurrentLimit(40);
    }

    public Command startIntake() {
        return Commands.runOnce(
            () -> {
                rightMotor.set(-0.5);
                leftMotor.set(-0.5);
                tiltMotor.set(0);
                loaderMotor.set(0);
            }
        );
    }

    public Command finishIntake() {
        return Commands.runOnce(
            () -> {
                rightMotor.set(-0.15);
                leftMotor.set(-0.15);
                loaderMotor.set(0);
                tiltMotor.getPIDController().setReference(-7, ControlType.kPosition);
            }
        ).andThen(
            Commands.waitSeconds(0.15)
        ).andThen(
            Commands.runOnce(
                () -> {
                    loaderMotor.set(0.25);
                }    
            )
        ).andThen(
            Commands.waitSeconds(0.11)
        ).andThen(
            () -> {
                rightMotor.set(0);
                leftMotor.set(0);
                loaderMotor.set(0);
                tiltMotor.getPIDController().setReference(-1, ControlType.kPosition);
            }    
        ).andThen(
            Commands.waitSeconds(0.15)
        ).andThen(
            Commands.runOnce(() -> tiltMotor.set(0))
        );

    }

    public Command shoot() {
        return Commands.runOnce(() -> {
            rightMotor.set(0.8);
            leftMotor.set(0.8);
            tiltMotor.set(0);
            loaderMotor.set(0);
        }).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            Commands.runOnce(() -> loaderMotor.set(-1))
        ).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            Commands.runOnce(() -> {
                leftMotor.set(0);
                rightMotor.set(0);
                loaderMotor.set(0);
            })
        );
    }
}
