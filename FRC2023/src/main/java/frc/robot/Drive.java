package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;

public class Drive implements IDrive {

    private MecanumDrive driveBase;

    private Mode mode;
    private IGyroscopeSensor gyroscope;
    private Runnable currentCompletionRoutine;


    // Motors
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax rearLeftMotor;
    private CANSparkMax rearRightMotor;


    // PID (Proportional gain may need to be changed, add other gains if needed)
    private PIDController rotationController;
    private double setP = 0.7;
    private double setI = 0.0;
    private double setD = 0.0;

    //balance
    private boolean balance = false;

    // Teleoperated
    private double forwardSpeed;
    private double strafeSpeed;
    private double angularSpeed;
    private double desiredAngle;

    // auto
    private double autoSpeed;
    private double autoStrafeSpeed;
    private double autoAngularSpeed;
    private Rotation2d autoAngleDegrees;
    private double desiredDistance;

    private static final double ROTATION_TOLERANCE_DEGREES = 2.0;

    public Drive(IGyroscopeSensor gyroscope) {
        this.gyroscope = gyroscope;

        frontLeftMotor = new CANSparkMax(PortMap.CAN.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(PortMap.CAN.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(PortMap.CAN.REAR_LEFT_MOTOR, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(PortMap.CAN.REAR_RIGHT_MOTOR, MotorType.kBrushless);

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        rearLeftMotor.restoreFactoryDefaults();
        rearRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(true);

        driveBase = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

        rotationController = new PIDController(setP, setI, setD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE_DEGREES));
    }

    @Override
    public Mode getCurrentDriveMode() {
        return mode;
    }

    @Override
    public void resetGyro() {
        gyroscope.resetYaw();
        desiredAngle = 0;
    }

    @Override
    public void balanceOn() {
        mode = Mode.MANUAL;
        balance = true;
    }

    @Override
    public void balanceOff() {
        mode = Mode.MANUAL;
        balance = false;
        forwardSpeed = 0;
        strafeSpeed = 0;
    }

    @Override
    public void rotateRelative(double angle) {
        mode = Mode.MANUAL;

        desiredAngle = gyroscope.getYaw() + angle;
    }

    @Override
    public void cartesianMovement(double xSpeed, double ySpeed, double zRotation, Runnable completionRoutine ) {

        driveBase.driveCartesian(xSpeed, ySpeed, zRotation);
        Debug.logPeriodic(Double.toString(frontLeftMotor.getEncoder().getPosition()));
    }

    @Override
    public void rotateAbsolute(double angle) {
        mode = Mode.MANUAL;

        desiredAngle = angle;
    }

    @Override
    public void driveSpeed(double forwardSpeed_manual, double strafeSpeed_manual, double angleSpeedManual) {
        // driveDistance(distanceInches, speed, angle, null);
        mode = Mode.MANUAL;
        forwardSpeed = forwardSpeed_manual;
        strafeSpeed = strafeSpeed_manual;
        angularSpeed = angleSpeedManual;
        
    }

    @Override
    public void driveSpeed(double forwardSpeedAuto, double strafeSpeedAuto, double angleSpeedAuto, Runnable completionRoutine) {
        mode = Mode.AUTO;
        forwardSpeed = forwardSpeedAuto;
        strafeSpeed = strafeSpeedAuto;
        angularSpeed = angleSpeedAuto;

    }

    @Override
    public void driveDistance(double distanceInches, double speed, double angle, Runnable completionRoutine) {

        mode = Mode.AUTO;

        distanceInches = distanceInches / 1.93;

        frontLeftMotor.getEncoder().setPosition(0.0);
        frontRightMotor.getEncoder().setPosition(0.0);
        rearLeftMotor.getEncoder().setPosition(0.0);
        rearRightMotor.getEncoder().setPosition(0.0);

        setCompletionRoutine(completionRoutine);
        desiredDistance = distanceInches;
        autoSpeed = speed;
        autoAngleDegrees = Rotation2d.fromDegrees(-angle);
    }

    @Override
    public void driveRevolutions(double rotations, double speed, double angle, Runnable completionRoutine) {
        mode = Mode.AUTO;

        frontLeftMotor.getEncoder().setPosition(0.0);
        frontRightMotor.getEncoder().setPosition(0.0);
        rearLeftMotor.getEncoder().setPosition(0.0);
        rearRightMotor.getEncoder().setPosition(0.0);

        setCompletionRoutine(completionRoutine);
        desiredDistance = rotations;
        autoSpeed = speed;
        autoAngleDegrees = Rotation2d.fromDegrees(-angle);

    }

    @Override
    public void rotateRelative(double angle, Runnable completionRoutine) {
        mode = Mode.AUTO;

        setCompletionRoutine(completionRoutine);
        desiredAngle = gyroscope.getYaw() + Math.toRadians(angle);
    }

    @Override
    public void move() {
        Debug.logPeriodic("frau lorenz");
        //driveManual(.2, 0);
        // driveSpeed(0.2,0,0);

        if (Math.abs(gyroscope.getRoll()) < .21) {
            stop();
            Debug.logPeriodic("Good");
        }
        else if (gyroscope.getRoll() < 0) {
            Debug.logPeriodic("less than 0");
            //-Math.exp(-gyroscope.getRoll())/12
            driveSpeed(-Math.exp(-gyroscope.getRoll())/13, 0, 0);
            try {
                Thread.sleep(100);
            } catch(InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
        else if (gyroscope.getRoll() > 0) {
            Debug.logPeriodic("greater than 0");
            driveSpeed(Math.exp(-gyroscope.getRoll())/10, 0, 0);
            try {
                Thread.sleep(100);
            } catch(InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void moveAuto() {
        //driveManual(.2, 0);
        // driveSpeed(0.2,0,0);

        if (Math.abs(gyroscope.getRoll()) < .16) {
            stop();
            Debug.logPeriodic("Good");
        }
        else if (gyroscope.getRoll() < 0) {
            Debug.logPeriodic("less than 0");
            //-Math.exp(-gyroscope.getRoll())/12
            driveSpeed(-Math.exp(-gyroscope.getRoll())/12, 0, 0, null);
            try {
                Thread.sleep(100);
            } catch(InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
        else if (gyroscope.getRoll() > 0) {
            Debug.logPeriodic("greater than 0");
            driveSpeed(Math.exp(-gyroscope.getRoll())/12, 0, 0, null);
            try {
                Thread.sleep(100);
            } catch(InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void rotateAbsolute(double angle, Runnable completionRoutine) {
        mode = Mode.AUTO;
        resetGyro();
        setCompletionRoutine(completionRoutine);
        desiredAngle = Math.toRadians(angle);
    }

    public void driveManualImplementation(double forwardSpeed, double strafeSpeed) {
        mode = Mode.MANUAL;

        double absoluteForward = 1 * (forwardSpeed * Math.cos(gyroscope.getYaw()) + strafeSpeed * Math.sin(gyroscope.getYaw()));
        double absoluteStrafe = 1 * (-forwardSpeed * Math.sin(gyroscope.getYaw()) + strafeSpeed * Math.cos(gyroscope.getYaw()));

        this.forwardSpeed = absoluteForward;
        this.strafeSpeed = absoluteStrafe;
    }

    @Override
    public void driveManual(double forwardSpeed, double strafeSpeed) {
        setCompletionRoutine(null);
        driveManualImplementation(forwardSpeed, strafeSpeed);
    }

    @Override
    public void stop() {
        driveManualImplementation(0.0, 0.0);
        desiredAngle = gyroscope.getYaw();
        angularSpeed = 0.0;

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        rearLeftMotor.restoreFactoryDefaults();
        rearRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(true);
    }

    @Override
    public void init() {
        balance = false;
        currentCompletionRoutine = null;
        stop();
        resetGyro();
    }

    private void setCompletionRoutine(Runnable completionRoutine) {
        if (currentCompletionRoutine != null) {
            throw new IllegalStateException("Tried to perform an autonomous action while one was already in progress!");
        }

        currentCompletionRoutine = completionRoutine;
    }

    private void handleActionEnd() {
        stop();

        if (currentCompletionRoutine != null) {
            Runnable oldCompletionRoutine = currentCompletionRoutine;
            currentCompletionRoutine = null;
            oldCompletionRoutine.run();
        }
    }

    private void manualControlPeriodic() {
        angularSpeed = .5 * rotationController.calculate(gyroscope.getYaw(), desiredAngle);
        Debug.logPeriodic(Double.toString(forwardSpeed));
        driveBase.driveCartesian(forwardSpeed, -strafeSpeed, angularSpeed);
    }  

    @Override
    public void periodic() {
        // Debug.logPeriodic("gyro yaw: " + Double.toString(gyroscope.getYaw()));
        // Debug.logPeriodic("gyro roll: " + Double.toString(gyroscope.getRoll()));
        // Debug.logPeriodic("gyro pitch: " + Double.toString(gyroscope.getPitch()));
        // Debug.logPeriodic("desired angle: " + Double.toString(desiredAngle));
        if (mode == Mode.MANUAL) {
            if (balance) {
                move();
            }
            manualControlPeriodic();
        } else if (mode == Mode.AUTO) {
            if (balance) {
                moveAuto();
            }
            Debug.log("Current" + Double.toString(gyroscope.getYaw()));

            Debug.log("Desired" + Double.toString(desiredAngle));
            
            Debug.logPeriodic("autodrive is running");

            autoAngularSpeed = .3 * rotationController.calculate(gyroscope.getYaw(), desiredAngle);

            //driveBase.drivePolar(autoSpeed, autoAngleDegrees, -.1);
            driveBase.driveCartesian(autoSpeed, autoStrafeSpeed , autoAngularSpeed);
            // Check if we've completed our travel
            double averageDistanceTraveledLeft = Math.abs((frontLeftMotor.getEncoder().getPosition() + rearLeftMotor.getEncoder().getPosition()) / 2);
            double averageDistanceTraveledRight = Math.abs((frontRightMotor.getEncoder().getPosition() + rearRightMotor.getEncoder().getPosition()) / 2);
            double averageDistanceTraveled = Math.abs((averageDistanceTraveledLeft + averageDistanceTraveledRight) / 2);

            //Debug.logPeriodic("Total Distance: " + averageDistanceTraveled);
            //Debug.logPeriodic("Rear left encoder: " + rearLeftMotor.getEncoder().getPosition());
            //Debug.logPeriodic("Rear right encoder: " + rearRightMotor.getEncoder().getPosition());
            //Debug.logPeriodic("Front left encoder: " + frontLeftMotor.getEncoder().getPosition());
            //Debug.logPeriodic("Front right encoder: " + frontRightMotor.getEncoder().getPosition());
            if (averageDistanceTraveled > desiredDistance) {

                Debug.log("stopped");
                resetGyro();
                try {
                    Thread.sleep(100);
                } catch(InterruptedException ie) {
                    Thread.currentThread().interrupt();
                }
                handleActionEnd();
                frontLeftMotor.restoreFactoryDefaults();
                frontRightMotor.restoreFactoryDefaults();
                rearLeftMotor.restoreFactoryDefaults();
                rearRightMotor.restoreFactoryDefaults();

                frontLeftMotor.setInverted(false);
                frontRightMotor.setInverted(true);
                rearLeftMotor.setInverted(false);
                rearRightMotor.setInverted(true);
            }
        } else {
            throw new IllegalArgumentException("The drive base controller is in an invalid drive mode.");
        }
    }
}
