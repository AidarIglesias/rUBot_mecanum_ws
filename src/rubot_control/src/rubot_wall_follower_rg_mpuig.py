#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None
current_state = "idle"
last_state_time = 0
d = 0
vx = 0
rate = None

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    # En la primera ejecución, calculamos el factor de corrección
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
        scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
        isScanRangesLengthCorrectionFactorCalculated = True

    # Define los límites de las regiones
    back_min = int(335 * scanRangesLengthCorrectionFactor)
    back_cut_min = int(360 * scanRangesLengthCorrectionFactor - 1)
    back_cut_max = int(0 * scanRangesLengthCorrectionFactor)
    back_max = int(25 * scanRangesLengthCorrectionFactor)
    bright_min = int(25 * scanRangesLengthCorrectionFactor)
    bright_max = int(65 * scanRangesLengthCorrectionFactor)
    right_min = int(65 * scanRangesLengthCorrectionFactor)
    right_max = int(115 * scanRangesLengthCorrectionFactor)
    fright_min = int(115 * scanRangesLengthCorrectionFactor)
    fright_max = int(155 * scanRangesLengthCorrectionFactor)
    front_min = int(155 * scanRangesLengthCorrectionFactor)
    front_max = int(205 * scanRangesLengthCorrectionFactor)
    fleft_min = int(205 * scanRangesLengthCorrectionFactor)
    fleft_max = int(245 * scanRangesLengthCorrectionFactor)
    left_min = int(245 * scanRangesLengthCorrectionFactor)
    left_max = int(295 * scanRangesLengthCorrectionFactor)
    bleft_min = int(295 * scanRangesLengthCorrectionFactor)
    bleft_max = int(335 * scanRangesLengthCorrectionFactor)

    # Define las regiones del láser
    regions = {
        'back': min(min(msg.ranges[back_min:back_cut_min]), min(msg.ranges[back_cut_max:back_max]), 3),
        'left': min(min(msg.ranges[left_min:left_max]), 3),
        'bleft': min(min(msg.ranges[bleft_min:bleft_max]), 3),
        'fleft': min(min(msg.ranges[fleft_min:fleft_max]), 3),
        'right': min(min(msg.ranges[right_min:right_max]), 3),
        'bright': min(min(msg.ranges[bright_min:bright_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front': min(min(msg.ranges[front_min:front_max]), 3),
    }

    take_action(regions)


def take_action(regions):
    global current_state, last_state_time
    global vx, d

    # Define las transiciones entre estados
    if rospy.get_time() - last_state_time < 0.5:  # Temporizador para evitar cambios rápidos
        return
    last_state_time = rospy.get_time()

    msg = Twist()
    linear_x, linear_y, angular_z = 0, 0, 0
    aux = 1.5

    if current_state == "idle":
        if regions['front'] > d and regions['left'] > d and regions['right'] > d and regions['back'] > d:
            current_state = "move_forward"
        elif regions['front'] < d:
            current_state = "turn_left"

    elif current_state == "move_forward":
        if regions['front'] < d:
            current_state = "turn_left"
        elif regions['back'] < d:
            current_state = "turn_right"

    elif current_state == "turn_left":
        if regions['left'] < d:
            current_state = "move_backward"
        elif regions['fleft'] < d * aux:
            current_state = "adjust_left"

    elif current_state == "turn_right":
        if regions['right'] < d:
            current_state = "move_backward"
        elif regions['bright'] < d * aux:
            current_state = "adjust_right"

    elif current_state == "move_backward":
        if regions['back'] > d:
            current_state = "idle"

    elif current_state == "adjust_left":
        if regions['front'] > d:
            current_state = "idle"

    elif current_state == "adjust_right":
        if regions['front'] > d:
            current_state = "idle"

    # Define las acciones para cada estado
    if current_state == "move_forward":
        linear_x = vx
    elif current_state == "move_backward":
        linear_x = -vx
    elif current_state == "turn_left":
        angular_z = 0.5 * vx
    elif current_state == "turn_right":
        angular_z = -0.5 * vx
    elif current_state == "adjust_left":
        linear_y = 0.5 * vx
    elif current_state == "adjust_right":
        linear_y = -0.5 * vx

    rospy.loginfo(f"Current State: {current_state}")
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()


def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Robot stopped.")


def main():
    global pub, rate, d, vx

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(25)
    d = rospy.get_param("~distance_laser", 0.5)
    vx = rospy.get_param("~forward_speed", 0.2)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown()