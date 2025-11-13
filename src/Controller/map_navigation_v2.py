from machine import Pin, PWM
from utime import sleep, ticks_ms


# ------------------ MOTOR CONTROL ------------------

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

    def off(self): self.pwm.duty_u16(0)
    def Forward(self, speed=60): self._move(0, speed)
    def Reverse(self, speed=60): self._move(1, speed)

    def _move(self, direction, speed):
        self.mDir.value(direction)
        self.pwm.duty_u16(int(65535 * speed / 100))


def move_forward(left, right, speed=60, dist_m=0.3):
    base_speed = 0.2 * (speed / 60)
    t = dist_m / base_speed
    left.Forward(speed); right.Forward(speed); sleep(t)
    left.off(); right.off()


def turn(left, right, rel_dir, speed=40, t=0.8):
    # rel_dir: +1 = right 90°, -1 = left 90°, 2 = U-turn
    if rel_dir == +1:
        left.Forward(speed); right.Reverse(speed)
    elif rel_dir == -1:
        left.Reverse(speed); right.Forward(speed)
    elif rel_dir == 2:
        left.Forward(speed); right.Reverse(speed); sleep(t * 2)
        left.off(); right.off(); return
    sleep(t)
    left.off(); right.off()


# ------------------ LINE SENSING ------------------

def read_sensor(pin_num):
    return Pin(pin_num, Pin.IN, Pin.PULL_UP).value()


def line_follow(left, right):
    base, turn_spd = 60, 35
    while True:
        s16, s17, s18, s19 = [read_sensor(i) for i in (16, 17, 18, 19)]

        if s17 == 1 and s18 == 1:
            left.Forward(base); right.Forward(base)
        elif s17 == 0 and s18 == 1:
            left.Forward(turn_spd); right.Forward(base)
        elif s17 == 1 and s18 == 0:
            left.Forward(base); right.Forward(turn_spd)
        elif s16 == 1 and s19 == 1:
            print("Possible junction detected")
            left.off(); right.off()
            return "junction"
        else:
            left.off(); right.off()
            return "lost"
        sleep(0.05)


# ------------------ GRAPH NAVIGATION ------------------

# Map[node][neighbor] = direction (0=N, 1=E, 2=S, 3=W)
maze_map = {
    0: {1: 0},
    1: {2: 3},
    2: {3: 2, 4: 3},
    4: {5: 2, 6: 0}
}

def relative_turn(current_dir, new_dir):
    diff = (new_dir - current_dir) % 4
    if diff == 1:  return +1   # turn right
    if diff == 3:  return -1   # turn left
    if diff == 2:  return  2   # U-turn
    return 0                   # straight


def navigate_path(path):
    left, right = Motor(4, 5), Motor(7, 6)
    direction = 0  # starting orientation (north)
    for i in range(len(path) - 1):
        node, next_node = path[i], path[i + 1]
        new_dir = maze_map[node][next_node]
        rel = relative_turn(direction, new_dir)
        print(f"At node {node}, going to {next_node}, rel_turn={rel}")
        if rel != 0:
            turn(left, right, rel)
        line_follow(left, right)
        move_forward(left, right)
        direction = new_dir
    left.off(); right.off()


# ------------------ MAIN ------------------

if __name__ == "__main__":
    path = [0, 1, 2, 4, 5]   # example path sequence through the maze
    navigate_path(path)
