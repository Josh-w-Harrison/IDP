# ================================================================
#   FULL MICROPYTHON PORT OF YOUR ARDUINO LINEFOLLOWER SYSTEM
# ================================================================

from machine import Pin, PWM
from utime import sleep, ticks_ms
import time


# ================================================================
# ENUM FOR DIRECTIONS
# ================================================================
class Direction:
    straight = 0
    left = 1
    right = 2
    NONE_D = 3
    ERROR_D = 4


# ================================================================
# SIMPLE STACK (size 3)
# ================================================================
class Stack:
    def __init__(self):
        self.stack = []
        self.max_size = 3

    def isEmpty(self):
        return len(self.stack) == 0

    def add(self, dir):
        if len(self.stack) < self.max_size:
            self.stack.append(dir)
            return 0
        return -1

    def pop(self):
        if self.stack:
            return self.stack.pop()
        return Direction.ERROR_D

    def reverseStack(self):
        for i in range(len(self.stack)):
            if self.stack[i] == Direction.left:
                self.stack[i] = Direction.right
            elif self.stack[i] == Direction.right:
                self.stack[i] = Direction.left
        return 0


# ================================================================
# MOTOR CLASS (YOUR IMPLEMENTATION)
# ================================================================
class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=50):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def _move(self, direction, speed):
        self.mDir.value(direction)
        self.pwm.duty_u16(int(65535 * speed / 100))


# ================================================================
# CIRCULAR BUFFER FOR DERIVATIVE CONTROL
# ================================================================
class CircularBuffer:
    def __init__(self, size, fill=0):
        self.buf = [fill] * size
        self.size = size
        self.index = 0

    def add(self, value):
        self.buf[self.index] = value
        self.index = (self.index + 1) % self.size

    def pop(self):
        return self.buf[self.index]


# ================================================================
# SENSOR READING
# ================================================================
def read_sensor(pin_num):
    return Pin(pin_num, Pin.IN, Pin.PULL_UP).value()


# ================================================================
# OPTIONAL — TUNNEL DISTANCE FUNCTION
# (YOU CAN FILL THIS IN WITH ULTRASONIC)
# ================================================================
def getTunnelDistance():
    return 0.0


# ================================================================
# LINE FOLLOWER CLASS (FULL PORT OF YOUR C++ CODE)
# ================================================================
class LineFollower:
    def __init__(self):
        # Motor objects
        self.motor_left = Motor(4, 5)
        self.motor_right = Motor(7, 6)

        # Mutable motor outputs (leftMotor & rightMotor equivalents)
        self.leftPower = 0.0
        self.rightPower = 0.0

        # State variables
        self.inTunnel = False
        self.haveBlock = False
        self.colour = None   # "Red", "Blue", or None
        self.robotState = 1  # 1=search, 2=collect, 3=return, 4=done
        self.branchCounter = 1

        # Gains
        self.kp = 0.25
        self.kd = 10.0
        self.basePower = 0.8

        # Stack + junction probing
        self.dirStack = Stack()
        self.probeStateJ = Direction.NONE_D

        # Active function pointer equivalent
        self.activeFunc = None

        # For PD control
        self.lastError = 0
        self.bufferSize = 20
        self.prevErrors = CircularBuffer(self.bufferSize, 0)
        self.sampleDelay = 0.05
        self.lastSample = time.time()
        self.previousError = 0

    # ============================================================
    # MAIN CONTROL ENTRY
    # ============================================================
    def control(self):
        # Read sensors
        s16 = read_sensor(9)
        s17 = read_sensor(10)
        s18 = read_sensor(11)
        s19 = read_sensor(12)

        lineBinary = (s16 << 3) | (s17 << 2) | (s18 << 1) | (s19)

        if self.activeFunc is None:
            res = self.detectEnd(lineBinary)
            if res == 0:
                return

            res = self.detectJunction(lineBinary)
            if res == 0:
                return

            self.followLine(lineBinary)
        else:
            self.activeFunc(lineBinary)

        self.applyPower()

    # ============================================================
    # APPLY MOTOR POWER
    # ============================================================
    def applyPower(self):
        L = int(max(0, min(1, self.leftPower)) * 100)
        R = int(max(0, min(1, self.rightPower)) * 100)
        self.motor_left.Forward(L)
        self.motor_right.Forward(R)

    # ============================================================
    # END OF LINE
    # ============================================================
    def detectEnd(self, lineBinary):
        if lineBinary == 0:
            self.activeFunc = self.checkTunnel
            return 0
        return -1

    # ============================================================
    # JUNCTION DETECTION
    # ============================================================
    def detectJunction(self, lineBinary):
        if lineBinary == 15:
            # Full junction
            if self.haveBlock and self.colour in ["Red", "Blue"]:
                self.dirStack.reverseStack()

            nxt = self.dirStack.pop()

            if nxt == Direction.left:
                self.activeFunc = self.turnLeft
            elif nxt == Direction.right:
                self.activeFunc = self.turnRight
            elif nxt == Direction.straight:
                self.activeFunc = self.moveStraight
            else:
                self.activeFunc = self.moveStraight

            return 0

        if lineBinary == 14:
            self.activeFunc = self.probeJunction
            self.probeStateJ = Direction.left
            return 0

        if lineBinary == 7:
            self.activeFunc = self.probeJunction
            self.probeStateJ = Direction.right
            return 0

        return -1

    # ============================================================
    # PD LINE FOLLOWING
    # ============================================================
    def followLine(self, lineBinary):
        self.leftPower = self.basePower
        self.rightPower = self.basePower

        error_map = {
            6: 0,
            1: -3,
            3: -2,
            2: -1,
            4: 1,
            12: 2,
            8: 3
        }

        error = error_map.get(lineBinary, self.lastError)
        self.lastError = error

        now = time.time()
        if now - self.lastSample >= self.sampleDelay:
            self.prevErrors.add(error)
            self.previousError = self.prevErrors.pop()
            self.lastSample = now

        derivative = (error - self.previousError) / self.sampleDelay

        correction = self.kp * error + self.kd * derivative

        self.leftPower -= correction
        self.rightPower += correction

    # ============================================================
    # ACTIVE FUNCTIONS
    # ============================================================
    def turnLeft(self, _):
        self.motor_left.Reverse(40)
        self.motor_right.Forward(40)
        sleep(0.8)
        self.motor_left.off()
        self.motor_right.off()
        self.activeFunc = None

    def turnRight(self, _):
        self.motor_left.Forward(40)
        self.motor_right.Reverse(40)
        sleep(0.8)
        self.motor_left.off()
        self.motor_right.off()
        self.activeFunc = None

    def moveStraight(self, _):
        self.motor_left.Forward(60)
        self.motor_right.Forward(60)
        sleep(0.9)
        self.motor_left.off()
        self.motor_right.off()
        self.activeFunc = None

    def turnAround(self, _):
        self.motor_left.Forward(60)
        self.motor_right.Reverse(60)
        sleep(1.4)
        self.motor_left.off()
        self.motor_right.off()
        self.activeFunc = None

    # ============================================================
    # TUNNEL CHECKING
    # ============================================================
    def checkTunnel(self, _):
        dist = getTunnelDistance()
        if dist != 0.0 and dist < 10.0:
            self.inTunnel = True
            self.activeFunc = None
            return 0
        else:
            self.activeFunc = self.probeSweep
            return -1

    # ============================================================
    # PROBE SWEEP
    # ============================================================
    def probeSweep(self, lineBinary):
        # Simple: if we see any line again, return
        if lineBinary != 0:
            self.activeFunc = None
            return 0

        # Sweep
        self.motor_left.Reverse(40)
        self.motor_right.Forward(40)
        sleep(0.3)

        self.motor_left.Forward(40)
        self.motor_right.Reverse(40)
        sleep(0.3)

        self.activeFunc = None
        return 0

    # ============================================================
    # JUNCTION PROBING (SIMPLE)
    # ============================================================
    def probeJunction(self, lineBinary):
        # HOLD LOW SPEED
        self.motor_left.Forward(33)
        self.motor_right.Forward(33)

        if self.probeStateJ == Direction.left and lineBinary != 14:
            self.activeFunc = None
            return 0

        if self.probeStateJ == Direction.right and lineBinary != 7:
            self.activeFunc = None
            return 0

        # After probing → decide
        pf = self.pathfind()

        if self.probeStateJ == Direction.left:
            self.activeFunc = self.moveStraight

        elif self.probeStateJ == Direction.right:
            if not self.haveBlock:
                self.dirStack.add(Direction.right)
                self.activeFunc = self.turnRight
            elif self.haveBlock and pf == 0:
                self.activeFunc = self.deliverBlock
            else:
                self.activeFunc = self.moveStraight

        self.probeStateJ = Direction.NONE_D
        return 0

    # ============================================================
    # PATHFIND LOGIC
    # ============================================================
    def pathfind(self):
        if not self.haveBlock and self.robotState == 2:
            self.branchCounter += 1
        else:
            self.branchCounter -= 1

        if self.colour == "Blue" and self.branchCounter == 2:
            return 0
        if self.colour == "Red" and self.branchCounter == 0:
            return 0
        if self.robotState == 3 and self.branchCounter == 1:
            return 0

        return -1

    # ============================================================
    # BLOCK DELIVERY
    # ============================================================
    def deliverBlock(self, _):
        self.turnRight(_)
        self.moveStraight(_)
        self.motor_left.Reverse(60)
        self.motor_right.Reverse(60)
        sleep(1.0)

        self.haveBlock = False
        self.colour = None
        self.robotState = 3
        self.activeFunc = None
        return 0

    # ============================================================
    # RETURN HOME
    # ============================================================
    def returnHome(self, _):
        if self.probeStateJ == Direction.left:
            self.turnLeft(_)
        else:
            self.turnRight(_)

        self.moveStraight(_)
        self.robotState = 4
        self.activeFunc = None
        return 0

if __name__ == "__main__":
    lf = LineFollower()   # create robot controller
    
    print("Line follower started.")
    
    while True:
        lf.control()      # continuously run control loop
        sleep(0.01)       # small pause to avoid overloading CPU