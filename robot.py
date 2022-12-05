#!/usr/bin/env python3

# py -3 robot.py deploy   #to deploy code
# py -3 robot.py deploy --no-version-check   #to deploy code if using non-matching version of robotpy on PC and Rio
import wpilib
import rev
import math
from networktables import NetworkTables
import navx
import ctre

class MyRobot(wpilib.TimedRobot):
    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # ******* IMPORTANT!!
        self.testBed = False  # False if deploying to actual robot

        self.fieldCentric = True
        self.newTimer = True
        self.autoFaceAngleComplete = False  # needed? Already in swerve class init

        # Robot characteristics and starting values
        self.wheelbaseWidth = 23.5  # in inches
        self.wheelbaseLength = 23.5  # in inches
        self.rotationalGearRatio = 12.8 # Gear ratio for MK3 and MK4: 12.8 . Use 1 for testbed. Auto select in code depending on self.testBed boolean
        self.driveRotPerInch = 0.7323    # 198 inch traveled with 145 motor turns
        self.drivePowerLimit = 0.6
        self.snailMode = False
        self.snailModePwrMultiplier = 0.15
        self.rotationPowerLimit = 0.5  # was 0.5
        self.joystickDeadzone = 0.1  # Global value for all sticks
        self.gyroDeadzone = 1
        self.txDeadzone = 0.35 # was 0.15
        self.taDeadzone = 22  # in percentage. 22 = 22 %
        self.paralysisConstant = 0.005   # increasing value increases paralysis range (immobilized while in deadzone)
        self.paralysis = self.txDeadzone * self.paralysisConstant
        self.gyroProportion = 0.02

        # Declaring motors
        self.driveFR = rev.CANSparkMax(4, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.driveFR.setInverted(False)
        self.driveEncFR = self.driveFR.getEncoder()

        self.driveFL = rev.CANSparkMax(6, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.driveFL.setInverted(False)
        self.driveEncFL = self.driveFL.getEncoder()

        self.driveBL = rev.CANSparkMax(14, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.driveBL.setInverted(False)
        self.driveEncBL = self.driveBL.getEncoder()

        self.driveBR = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.driveBR.setInverted(False)
        self.driveEncBR = self.driveBR.getEncoder()

        self.rotationFR = rev.CANSparkMax(3, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotationFR.setInverted(True)
        self.rotationEncFR = self.rotationFR.getEncoder()
        self.rotationPID_FR = self.rotationFR.getPIDController()

        self.rotationFL = rev.CANSparkMax(5, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotationFL.setInverted(True)
        self.rotationEncFL = self.rotationFL.getEncoder()
        self.rotationPID_FL = self.rotationFL.getPIDController()

        self.rotationBL = rev.CANSparkMax(13, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotationBL.setInverted(True)
        self.rotationEncBL = self.rotationBL.getEncoder()
        self.rotationPID_BL = self.rotationBL.getPIDController()

        self.rotationBR = rev.CANSparkMax(2, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotationBR.setInverted(True)
        self.rotationEncBR = self.rotationBR.getEncoder()
        self.rotationPID_BR = self.rotationBR.getPIDController()

        self.absoluteEncFR = ctre.CANCoder(1)
        self.absoluteEncFL = ctre.CANCoder(2)
        self.absoluteEncBL = ctre.CANCoder(3)
        self.absoluteEncBR = ctre.CANCoder(4)

        if self.testBed == False:
            self.encOffsetFR = self.absoluteEncFR.getAbsolutePosition() / 360 * self.rotationalGearRatio
            self.encOffsetFL = self.absoluteEncFL.getAbsolutePosition() / 360 * self.rotationalGearRatio
            self.encOffsetBL = self.absoluteEncBL.getAbsolutePosition() / 360 * self.rotationalGearRatio
            self.encOffsetBR = self.absoluteEncBR.getAbsolutePosition() / 360 * self.rotationalGearRatio
            self.rotationEncFR.setPosition(0)
            self.rotationEncFL.setPosition(0)
            self.rotationEncBL.setPosition(0)
            self.rotationEncBR.setPosition(0)
        else:
            self.rotationalGearRatio = 1  # Gear ratio for MK3: 12.8 . Use 1 for testbed.
            self.driveRotPerInch = 1  #  1 for testbed for ease of following rotation

            self.encOffsetFR = 0
            self.encOffsetFL = 0
            self.encOffsetBL = 0
            self.encOffsetBR = 0

            # Declaring SwerveDrive instance
        self.swerve = SwerveDrive(self.wheelbaseWidth, self.wheelbaseLength, self.rotationalGearRatio,
                                  self.drivePowerLimit, self.joystickDeadzone, self.gyroDeadzone)

        # Declaring Solenenoids. New for 2022 with Rev controller Blame Miles  =)
        # self.limelightSolenoid = wpilib.Solenoid(wpilib.PneumaticsModuleType.REVPH, 0)
        # in periodic, simply use: self.limelightSolenoid.set(True) or self.limelightSolenoid.set(False)

        # Set up the PID Gains as constants.  These Values are Recommended by Rev
        # # PID Coefficents and Controller Output Range
        self.rotationFRkP = 0.5  # was 5e-5
        self.rotationFRkI = 0.01 # was 1e-4
        self.rotationFRkD = 0
        self.rotationFRkIz = 0
        self.rotationFRkFF = 0
        self.rotationFRkMaxOutput = self.rotationPowerLimit
        self.rotationFRkMinOutput = -self.rotationPowerLimit

        # The restoreFactoryDefaults() method can be used to reset the
        # configuration parameters in the SPARK MAX to their factory default
        # state. If no argument is passed, these parameters will not persist
        # between power cycles. ? what are the arguments --> something to set the parameters into memory?

        # self.rotationFR.restoreFactoryDefaults()   # do not use since cancels motor reversal.
        # Reversal is maintained between robot restarts even if reversal is removed.
        # The factory restore does wipe out the residual motor reversal. Rev neo by default turns counterclockwise.

        # Set rotational motor PID Constants
        self.rotationPID_FR.setP(self.rotationFRkP)
        self.rotationPID_FR.setI(self.rotationFRkI)
        self.rotationPID_FR.setD(self.rotationFRkD)
        self.rotationPID_FR.setIZone(self.rotationFRkIz)
        self.rotationPID_FR.setFF(self.rotationFRkFF)
        self.rotationPID_FR.setOutputRange(self.rotationFRkMinOutput, self.rotationFRkMaxOutput)

        self.rotationPID_FL.setP(self.rotationFRkP)
        self.rotationPID_FL.setI(self.rotationFRkI)
        self.rotationPID_FL.setD(self.rotationFRkD)
        self.rotationPID_FL.setIZone(self.rotationFRkIz)
        self.rotationPID_FL.setFF(self.rotationFRkFF)
        self.rotationPID_FL.setOutputRange(self.rotationFRkMinOutput, self.rotationFRkMaxOutput)

        self.rotationPID_BL.setP(self.rotationFRkP)
        self.rotationPID_BL.setI(self.rotationFRkI)
        self.rotationPID_BL.setD(self.rotationFRkD)
        self.rotationPID_BL.setIZone(self.rotationFRkIz)
        self.rotationPID_BL.setFF(self.rotationFRkFF)
        self.rotationPID_BL.setOutputRange(self.rotationFRkMinOutput, self.rotationFRkMaxOutput)

        self.rotationPID_BR.setP(self.rotationFRkP)
        self.rotationPID_BR.setI(self.rotationFRkI)
        self.rotationPID_BR.setD(self.rotationFRkD)
        self.rotationPID_BR.setIZone(self.rotationFRkIz)
        self.rotationPID_BR.setFF(self.rotationFRkFF)
        self.rotationPID_BR.setOutputRange(self.rotationFRkMinOutput, self.rotationFRkMaxOutput)

        self.joystick0 = wpilib.XboxController(0)  # First controller - Driver
        self.joystick1 = wpilib.XboxController(1)  # Second controller  - Gadget
        # self.joystick2 = wpilib.XboxController(2)  # Third Controller - Testing

        # self.navx = navx.AHRS.create_i2c()
        self.navx = navx.AHRS(wpilib.SerialPort.Port.kUSB1)   # use kUSB1 for outer USB, and kUSB for either. kUSB2 will crash driver station.
        self.navx.reset()

        self.timer = wpilib.Timer()
        self.timer.start()

        NetworkTables.initialize()
        self.networkTable = NetworkTables.getTable("SmartDashboard")

        # cancoder used for odometry
        self.CanCoderR = ctre.CANCoder(5)
        self.encoderValue1 = self.CanCoderR.getPosition()

        self.CanCoderL = ctre.CANCoder(6)
        self.encoderValue2 = self.CanCoderL.getPosition()

        self.CanCoderB = ctre.CANCoder(7)
        self.encoderValue3 = self.CanCoderB.getPosition()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.fieldCentric = True
        self.autonomous = True
        self.manualDriving = False
        self.autoFaceAngle = 0  # Default face angle at start

        self.drivePowerLimit = 0.6  # starting power of drive motors (on Geoff)

        self.autonomousTimer = wpilib.Timer()
        self.autonomousTimer.start()
        self.newStep = True
        self.step = 1

        self.teleopTimer = wpilib.Timer()
        self.teleopTimer.start()
        self.newStepTeleop = True
        self.stepTeleop = 1

        self.closeEnough = 0  # Deadzone allowed in inches
        self.joyR_X = 0

        self.newStepDrive = True

        self.rumblePower = 0

        self.gyroCompensation = 0  #needs to be since there are no position switches

    def autonomousPeriodic(self):
        # """This function is called periodically during autonomous."""
        self.gyroAngle = self.navx.getAngle() + self.gyroCompensation

        # if self.step == 1: # turn counterclockwise (without moving) towards 2nd ball
        #     self.autoFaceAngle = -150
        #     angle = -150  # Clockwise is positive
        #     power = 0.5  # Max of 1
        #     distance = 40  # In inches if on real robot
        #     self.autoDrive(angle, power, distance)
        # if self.step == 2: # stop
        #     self.stopDriving()
        #     self.step += 1
        # if self.step == 3:
        #     self.autoFaceAngle = -75
        #     self.autoFaceAngleTurnOnly()
        # if self.step == 233:
        #     self.stopDriving()
        #     self.step += 1
        # if self.step == 4:
        #     self.autoFaceAngle = -75
        #     angle = -80  # Clockwise is positive
        #     power = 0.5  # Max of 1
        #     distance = 36  # In inches if on real robot
        #     self.autoDrive(angle, power, distance)
        # if self.step == 5:
        #     self.stopDriving()

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""

        self.fieldCentric = True
        self.autonomous = False
        self.manualDriving = True
        self.autoFaceAngle = 0  # Not used in teleop. Only to fulfill the parameter need of a function.
        # self.runAndGun = False
        # self.runAndGunX = 0
        self.drivePowerLimit = 0.8 	# should be 1 eventually
        self.yCoord2 = 0
        self.xCoord2 = 0

        self.lastR = 0
        self.lastB = 0
        self.lastL = 0

        self.teleopTimer = wpilib.Timer()
        self.teleopTimer.start()
        self.newStepTeleop = True
        self.stepTeleop = 1

        self.snailMode = False

        self.rumblePower = 0

        self.gyroCompensation = 0  #needs to be since there are no position switches

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        """no it is not."""
        # self.fieldCentricToggle(self.joystick0.getBackButtonPressed())  # Not needed for this game since climbing is square with the field
        self.fieldCentric = True

        self.gyroAngle = self.navx.getAngle()  + self.gyroCompensation

        self.rightEncPos = -self.CanCoderR.getPosition()   # maybe invert?
        self.leftEncPos = self.CanCoderL.getPosition()
        self.backEncPos = -self.CanCoderB.getPosition()

        # Actual 2 lines needed for manual driving, needed for swervedrive class
        joyL_X, joyL_Y, joyR_X = self.readingJoysticks()
        self.swerveMasterControl(joyL_X, joyL_Y, joyR_X, self.autoFaceAngle, self.autonomous)

        self.xCoord, self.yCoord = self.odometryCoordinates(self.gyroAngle, self.rightEncPos, self.leftEncPos, self.backEncPos)



        # Publishing telemetry
        self.networkTable.putNumber("Right Encoder", self.rightEncPos)
        self.networkTable.putNumber("Left Encoder", self.leftEncPos)
        self.networkTable.putNumber("Back Encoder", self.backEncPos)
        self.networkTable.putNumber("GyroAngle", self.gyroAngle)


        self.networkTable.putNumber("x-Coordinate", self.xCoord)
        self.networkTable.putNumber("y-Coordinate", self.yCoord)

    def testInit(self):
        pass

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def odometryCoordinates(self, navx, rightEnc, leftEnc, backEnc):
        self.distR = 12.25 # distance from center of robot right wheel is
        self.distB = 18.25 # distance from center of robot the back wheel is
        self.encDiameter = 3
        self.encConstant = (self.encDiameter * math.pi)/ 360 # inches / degree ( circumference / 360 )

        angle = navx * math.pi / 180

        self.rightDist = rightEnc * self.encConstant # how far the right encoder has travelled
        self.leftDist = leftEnc * self.encConstant # how far the left encoder has travelled
        self.backDist = (backEnc * self.encConstant) - (self.distB * 2 * math.pi * angle/360) # # how far the back encoder has travelled minus the distance accrued from rotation.


        deltaR = self.deltaR - self.lastR
        deltaL = self.deltaL - self.lastL


        deltaY = self.deltaB - self.lastB

        deltaX = (deltaL + deltaR) /2 # average of right and left wheel's delta


        #xCoord = 2 * ((self.deltaB / angle) + self.distB) * (math.sin((angle / 2))) # add sin of other angle or smth

        #yCoord = 2 * ((self.deltaR / angle) + self.distR) * (math.sin((angle / 2))) # add sin of other angle or smth
        # both of these r bad


        xCoord =  deltaX * math.cos(angle) - deltaY * math.sin(angle)
        yCoord = deltaX * math.sin(angle) + deltaY * math.cos(angle)


        self.xCoord2 = self.xCoord2 + xCoord

        self.yCoord2 = self.yCoord2 + yCoord


        self.lastR = self.rightDist
        self.lastL = self.leftDist
        self.lastB = self.backDist



        return self.xCoord2, self.yCoord2


    def readingJoysticks(self):
            joyL_X = self.joystick0.getLeftX()
            joyL_Y = -self.joystick0.getLeftY()
            joyR_X = self.joystick0.getRightX()
            return joyL_X, joyL_Y, joyR_X

    def vectorConversion(self, angle, power):
        joyL_X = power * math.cos((90 - angle) * math.pi / 180) # this could just be math.sin (angle), as cos(90-something) is the same as sin(something)
        joyL_Y = power * math.sin((90 - angle) * math.pi / 180) # this could just be math.cos (angle) for above reason
        return joyL_X, joyL_Y

    # Dirty function that centralizes calls to swerve class and runs the motors to facilitate use in
    # all other run modes, including teleop and autonomous.
    def swerveMasterControl(self, joyL_X, joyL_Y, joyR_X, autoFaceAngle, autonomous):
        # snailMode status and resultant power adjustment
        if self.snailMode == False:
            self.drivePowerAdjustment = 1
        else:
            self.drivePowerAdjustment = self.snailModePwrMultiplier

        # Passing joystick input to invKinematics for initial processing
        self.swerve.invKinematics(joyL_X, joyL_Y, joyR_X, autoFaceAngle, autonomous, self.gyroAngle, self.fieldCentric, self.manualDriving, self.gyroProportion, self.paralysis)

        # Running each wheel through the outputAngleNPower function of the SwerveDrive class
        drivePowerFR, targetAngleFR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FR", self.rotationEncFR.getPosition() + self.encOffsetFR)
        self.rotationPID_FR.setReference(targetAngleFR - self.encOffsetFR, rev.CANSparkMaxLowLevel.ControlType.kPosition)  # Just goto desired angle
        self.driveFR.set(drivePowerFR * self.drivePowerAdjustment)

        drivePowerFL, targetAngleFL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FL", self.rotationEncFL.getPosition() + self.encOffsetFL)
        self.rotationPID_FL.setReference(targetAngleFL - self.encOffsetFL, rev.CANSparkMaxLowLevel.ControlType.kPosition)  # Just goto desired angle
        self.driveFL.set(drivePowerFL * self.drivePowerAdjustment)

        drivePowerBL, targetAngleBL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BL", self.rotationEncBL.getPosition() + self.encOffsetBL)
        self.rotationPID_BL.setReference(targetAngleBL - self.encOffsetBL, rev.CANSparkMaxLowLevel.ControlType.kPosition)  # Just goto desired angle
        self.driveBL.set(drivePowerBL * self.drivePowerAdjustment)

        drivePowerBR, targetAngleBR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BR", self.rotationEncBR.getPosition() + self.encOffsetBR)
        self.rotationPID_BR.setReference(targetAngleBR - self.encOffsetBR, rev.CANSparkMaxLowLevel.ControlType.kPosition)  # Just goto desired angle
        self.driveBR.set(drivePowerBR * self.drivePowerAdjustment)

    def stopDriving(self):
        self.driveFR.set(0)
        self.driveFL.set(0)
        self.driveBL.set(0)
        self.driveBR.set(0)


    def autoDrive(self, angle, power, distance):  # Swerve in autonomous with distance traveled controlled by encoder.
        # This includes faceAngle turn if driving is concurrent. For faceTurn only, use autoFaceAngleTurnOnly function
        if power != 0:
            joyL_X, joyL_Y = self.vectorConversion(angle, power)
            currentPosition = self.driveEncBR.getPosition() / self.driveRotPerInch
            if self.newStepDrive:
                self.initialPosition = currentPosition
                self.newStepDrive = False
            error = distance - abs(currentPosition - self.initialPosition)
            if error > self.closeEnough:
                # if self.runAndGun:  # selecting X input source
                #     joyR_X = self.runAndGunX
                # else:
                #     joyR_X = self.joyR_X
                # self.swerveMasterControl(joyL_X, joyL_Y, joyR_X, self.autoFaceAngle, self.autonomous)
                self.swerveMasterControl(joyL_X, joyL_Y, self.joyR_X, self.autoFaceAngle, self.autonomous)
            else:
                # self.runAndGun = False
                self.newStepDrive = True
                self.step += 1
        else:
            # self.runAndGun = False
            self.stopDriving()
            self.step += 1

    # def autoRunAndGun(self, angle, power, distance): #autodrive and shoot at same time.
    #     self.runAndGun = True
    #     self.automatedIntakeAndShootingMasterControl(0, 1)
    #     self.autoDrive(angle, power, distance)

    def autoFaceAngleTurnOnly(self):
        if self.autoFaceAngleComplete == False:
            self.swerveMasterControl(0, 0, self.joyR_X, self.autoFaceAngle, self.autonomous)
        else:
            self.autoFaceAngleComplete = False
            self.newStep = True  # Not quite sure why this is needed?
            self.step += 1

    def autoTimer(self, desiredTime):  # Timer to control an autonomous loop
        if self.newStep:
            self.autonomousTimer.reset()
            self.newStep = False
        if self.autonomousTimer.get() > desiredTime:
            # self.runAndGun = False  # Just to ensure product freshness  =)
            # self.fieldCentric = True    # Just to ensure product freshness  =)
            self.newStep = True
            # self.autoShootingMasterControl(0) # also stops shooting in auton
            self.step += 1

    def teleTimer(self, desiredTime):  # Timer to control a teleop loop
        if self.newStepTeleop:
            self.teleopTimer.reset()
            self.newStepTeleop= False
        if self.teleopTimer.get() > desiredTime:
            self.newStepTeleop = True
            self.stepTeleop += 1

class SwerveDrive:
    """to use this class, essential to define robotWidth, robotLength, and rotationalGearRatio in Main code below imports.
    They are needed to be passed as global variables into this class in the most non-Pythonic way! """

    def __init__(self, robotWidth, robotLength, rotationalGearRatio, drivePowerLimit, joystickDeadzone, gyroDeadzone):
        self.robotWidth = robotWidth
        self.robotLength = robotLength
        self.robotRadius = math.sqrt(self.robotWidth ** 2 + self.robotLength ** 2)  # hypotenuse
        self.rotationalGearRatio = rotationalGearRatio
        self.drivePowerLimit = drivePowerLimit
        self.joystickDeadzone = joystickDeadzone
        self.gyroDeadzone = gyroDeadzone   # in degrees
        self.newFaceAngle = True
        self.autoFaceAngleComplete = False

    def invKinematics(self, joyL_X, joyL_Y, joyR_X, autoFaceAngle, autonomous, gyroAngle, fieldCentric, manualDriving, gyroProportion, paralysis):  # function to calculate the rotation and drive power
        # Filtering out the deadzone for manual driving input.
        # vectorThreshold is point when drive system is made inactive so not jump to unpredicted position at deadzone
        if manualDriving:   # includes face angle lock while manually driving
            self.vectorThreshold = self.joystickDeadzone
            if abs(joyL_X) <= self.joystickDeadzone:
                joyL_X = 0
            if abs(joyL_Y) <= self.joystickDeadzone:
                joyL_Y = 0
            if abs(joyR_X) <= self.joystickDeadzone:  # Essential to control right joystick input at this stage
                joyR_X = 0
            if self.newFaceAngle: # to update the angle the robot is facing
                self.initialFaceAngle = gyroAngle
                self.newFaceAngle = False
        else:   #   includes auton face rotation/angle lock and limelight targeting
            self.vectorThreshold = paralysis
            if autonomous:    # In autonomous mode, the set autoFaceAngle overrides the initialFaceAngle
                self.initialFaceAngle = autoFaceAngle

        # For field-centric commands. The gyro angle is injected at this stage for compensation. Rotational element
        # not included and therefore not effected. Otherwise pure rotation cannot be maintained when gyro angle is changed.
        # Important to convert the gyroAngle in degrees to radians by * it by "math.pi / 180". Python math trig in radians.
        if fieldCentric:
            joyL_Y_orig = joyL_Y
            joyL_Y = joyL_Y * math.cos(gyroAngle * math.pi / 180) + joyL_X * math.sin(gyroAngle * math.pi / 180)
            joyL_X = -joyL_Y_orig * math.sin(gyroAngle * math.pi / 180) + joyL_X * math.cos(gyroAngle * math.pi / 180)

        # Original location of this code snippet before moved upwards
        # if self.newFaceAngle: # to update angle the robot is facing
        #     self.initialFaceAngle = gyroAngle
        #     self.newFaceAngle = False

        # This filter assures robot face direction lock only active when manual right stick rotation not performed.
        # Sensor guided turns (e.g. Limelight) is not effected since right stick rotation will be greater than 0.
        # This will also work for autonomous with manualDriving set to False, or left stick speed less than deadzone
        # will not be face direction corrected.
        if joyR_X == 0: # to maintain the angle the robot is facing if right joystick not moved
                # Was "abs(joyR_X) <= self.joystickDeadzone" before. Now == 0 since up top any right stick less than deadzone is 0 anyways.
                # Now, auton and virtual right stick entered of less than deadzone will not enter loop and be negated
            error = gyroAngle - self.initialFaceAngle
            # The new resulting joyR_X can be greater than the original and still enter this argument since the new joyR_X
            # is passed down to do it's deed while the new incoming loop joyR_X value is fresh and raw from the joystick.
            if abs(error) > self.gyroDeadzone:
                joyR_X = -(error * gyroProportion)
                if abs(joyR_X) > 1:   # So joyR_X remains between -1 and 1.
                    joyR_X = joyR_X / abs(joyR_X)

                self.autoFaceAngleComplete = False
            else:
                joyR_X = 0
                self.autoFaceAngleComplete = True
        else:
            self.newFaceAngle = True # to continue renewal of the angle the robot is facing

        # X and Y component of straight and rotational vectors combined for calculating inverse kinematics
        self.A_Xnet1 = joyL_X - joyR_X * (self.robotLength / self.robotRadius)
        self.B_Xnet2 = joyL_X + joyR_X * (self.robotLength / self.robotRadius)
        self.C_Ynet1 = joyL_Y - joyR_X * (self.robotWidth / self.robotRadius)
        self.D_Ynet2 = joyL_Y + joyR_X * (self.robotWidth / self.robotRadius)

        # Calculating wheel speed
        self.wheelSpeedFR = math.sqrt(self.B_Xnet2 ** 2 + self.C_Ynet1 ** 2)
        self.wheelSpeedFL = math.sqrt(self.B_Xnet2 ** 2 + self.D_Ynet2 ** 2)
        self.wheelSpeedBL = math.sqrt(self.A_Xnet1 ** 2 + self.D_Ynet2 ** 2)
        self.wheelSpeedBR = math.sqrt(self.A_Xnet1 ** 2 + self.C_Ynet1 ** 2)

        # Finding Max wheel speed and normalizing so not surpass 1.0
        self.wheelSpeedMax = max(self.wheelSpeedFR, self.wheelSpeedFL, self.wheelSpeedBL, self.wheelSpeedBR)
        if self.wheelSpeedMax > 1:
            self.wheelSpeedFR = self.wheelSpeedFR / self.wheelSpeedMax
            self.wheelSpeedFL = self.wheelSpeedFL / self.wheelSpeedMax
            self.wheelSpeedBL = self.wheelSpeedBL / self.wheelSpeedMax
            self.wheelSpeedBR = self.wheelSpeedBR / self.wheelSpeedMax

        # Calculating wheel angles- modified atan2 function to account for 0 deg at north and CW being positive
        #  math.atan2(-X,-Y)*(180/math.pi) + 180
        self.wheelAngleFR = math.atan2(-self.B_Xnet2, -self.C_Ynet1) * (180 / math.pi) + 180
        self.wheelAngleFL = math.atan2(-self.B_Xnet2, -self.D_Ynet2) * (180 / math.pi) + 180
        self.wheelAngleBL = math.atan2(-self.A_Xnet1, -self.D_Ynet2) * (180 / math.pi) + 180
        self.wheelAngleBR = math.atan2(-self.A_Xnet1, -self.C_Ynet1) * (180 / math.pi) + 180

    def outputAngleNPower(self, wheelLocation, rotationEnc):
        # to ensure netDirAngle position always from 0 to 0.99 and never negative
        # net angle is post compensating for gyro heading. wheelAngle is in degrees. wheelAngle0to1 in 0-1(post conversion)
        if wheelLocation == "FR":
            wheelAngle = self.wheelAngleFR
            wheelSpeed = self.wheelSpeedFR
        elif wheelLocation == "FL":
            wheelAngle = self.wheelAngleFL
            wheelSpeed = self.wheelSpeedFL
        elif wheelLocation == "BL":
            wheelAngle = self.wheelAngleBL
            wheelSpeed = self.wheelSpeedBL
        elif wheelLocation == "BR":
            wheelAngle = self.wheelAngleBR
            wheelSpeed = self.wheelSpeedBR

        wheelAngle0to1 = wheelAngle/360  # to convert 360 degree format into decimal format

        # to ensure desired position always from 0 to 0.99 and never negative
        fractionNetDirAngle, wholeNetDirAngle = math.modf(wheelAngle0to1)

        if wheelAngle0to1 >= 0:
            wheelAngle0to1 = fractionNetDirAngle #retrieving the fraction portion of self.wheelAngle0to1 only
        else:
            wheelAngle0to1 = 1 + fractionNetDirAngle

        # to ensure encoder position always from 0 to 0.99 and never negative
        fractionEnc, wholeEnc = math.modf(rotationEnc / self.rotationalGearRatio)

        if rotationEnc >= 0:
            encPosition = fractionEnc  #retrieving the fraction portion of encoder position only
        else:
            encPosition = 1 + fractionEnc

        # modifying the 2 angle sets so can be compared
        # New simplified code for comparison. For encoder positions more or less than 0.5, each have a potentially
        # difficult to compare zones near 0 representing the destination angles. This simply processes those zones
        # to make them now comparable.
        if encPosition < 0.5:
            if wheelAngle0to1 >= encPosition + 0.5 and wheelAngle0to1 < 1:
                wheelAngle0to1 = wheelAngle0to1 - 1
        else:
            if wheelAngle0to1 < encPosition - 0.5 and wheelAngle0to1 >= 0:
                wheelAngle0to1 = wheelAngle0to1 + 1

        # New simplified code for determining most efficient (least) angle to reach needed wheelAngle and resulting power direction
        # includes wheelAngle turning commands (via PID) and wheel power direction
        if wheelSpeed >= self.vectorThreshold:  # swerve unit functions only when input threshold (joystickDeadzone) reached
            if abs(encPosition - wheelAngle0to1) <= 0.25:  # only when desired rotation within +/- 90 deg
                drivePower = wheelSpeed * self.drivePowerLimit  # real drive is forward
                targetAngle = rotationEnc - (
                            self.rotationalGearRatio * (encPosition - wheelAngle0to1))  # Accounting for gear ratio
            else:
                drivePower = -wheelSpeed * self.drivePowerLimit  # real drive is backwards
                if encPosition < wheelAngle0to1:
                    indirectAngle = 0.5 + (encPosition - wheelAngle0to1)
                elif encPosition > wheelAngle0to1:
                    indirectAngle = (encPosition - wheelAngle0to1) - 0.5
                else:
                    indirectAngle = 0

                targetAngle = rotationEnc - (self.rotationalGearRatio * indirectAngle)  # Accounting for gear ratio
        else:
            drivePower = 0
            targetAngle = rotationEnc

        return drivePower, targetAngle, self.autoFaceAngleComplete

class VariableAdjustment:
    """To use this class, will need to create a new instance for each variable being tweaked
    in the appropriate init"""

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    # Sample code for IRT (in real time) variable adjustment. Variables used are specific to application

    # # utilizing controller buttons to adjust PIDs while running teleop, using VariableAdjustment Class
    # new, self.rotationFRkP = self.rotationFR_P.joy0_ButtonsAB(self.rotationFRkP, increments)
    # if new:  # These last 2 lines not needed if only variable need update and no need to set PID
    #     self.rotationPID_FR.setP(self.rotationFRkP)
    #
    # new, self.rotationFRkI = self.rotationFR_I.joy0_ButtonsXY(self.rotationFRkI, increments)
    # if new:
    #     self.rotationPID_FR.setI(self.rotationFRkI)
    #
    # new, self.rotationFRkD = self.rotationFR_D.joy0_BumpersLR(self.rotationFRkD, increments)
    # if new:
    #     self.rotationPID_FR.setD(self.rotationFRkD)

    def __init__(self):
        self.joystick0 = wpilib.XboxController(0)
        self.joystick1 = wpilib.XboxController(1)
        self.joystick2 = wpilib.XboxController(2)

    def joy0_ButtonsAB(self, currentValue, increments):
        newValue = currentValue
        if self.joystick0.getAButtonPressed():
            newValue -= increments
        elif self.joystick0.getBButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy0_ButtonsXY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick0.getXButtonPressed():
            newValue += increments
        elif self.joystick0.getYButtonPressed():
            newValue -= increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy0_ButtonsAY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick0.getAButtonPressed():
            newValue -= increments
        elif self.joystick0.getYButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy0_BumpersLR(self, currentValue, increments):
        newValue = currentValue
        if self.joystick0.getLeftBumperPressed():
            newValue -= increments
        elif self.joystick0.getRightBumperPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy1_ButtonsAB(self, currentValue, increments):
        newValue = currentValue
        if self.joystick1.getAButtonPressed():
            newValue -= increments
        elif self.joystick1.getBButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy1_ButtonsXY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick1.getXButtonPressed():
            newValue -= increments
        elif self.joystick1.getYButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy1_ButtonsXB(self, currentValue, increments):
        newValue = currentValue
        if self.joystick1.getXButtonPressed():
            newValue -= increments
        elif self.joystick1.getBButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy1_ButtonsAY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick1.getAButtonPressed():
            newValue -= increments
        elif self.joystick1.getYButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy1_BumpersLR(self, currentValue, increments):
        newValue = currentValue
        if self.joystick1.getLeftBumperPressed():
            newValue -= increments
        elif self.joystick1.getRightBumperPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy2_ButtonsAB(self, currentValue, increments):
        newValue = currentValue
        if self.joystick2.getAButtonPressed():
            newValue -= increments
        elif self.joystick2.getBButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy2_ButtonsXY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick2.getXButtonPressed():
            newValue += increments
        elif self.joystick2.getYButtonPressed():
            newValue -= increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy2_ButtonsAY(self, currentValue, increments):
        newValue = currentValue
        if self.joystick2.getAButtonPressed():
            newValue -= increments
        elif self.joystick2.getYButtonPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

    def joy2_BumpersLR(self, currentValue, increments):
        newValue = currentValue
        if self.joystick2.getLeftBumperPressed():
            newValue -= increments
        elif self.joystick2.getRightBumperPressed():
            newValue += increments
        if newValue != currentValue:
            return True, newValue
        else:
            return False, currentValue

if __name__ == "__main__":
    wpilib.run(MyRobot)