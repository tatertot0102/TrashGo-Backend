import asyncio
import json
import websockets
import RPi.GPIO as GPIO
import time
from threading import Thread
from flask import Flask, Response
import cv2

# ==============================
# CONFIGURATION
# ==============================
MOTOR_PINS = {
    "LEFT_FRONT": 17,
    "LEFT_BACK": 18,
    "RIGHT_FRONT": 22,
    "RIGHT_BACK": 23,
}

RAMP_SERVO = 24
LEFT_ARM_SERVO = 25
RIGHT_ARM_SERVO = 12

PWM_FREQ = 50
ARM_OPEN_ANGLE = 0
ARM_CLOSED_ANGLE = 45
RAMP_UP_ANGLE = 0
RAMP_DOWN_ANGLE = 90

# ==============================
# GPIO SETUP
# ==============================
GPIO.setmode(GPIO.BCM)

for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

GPIO.setup(RAMP_SERVO, GPIO.OUT)
GPIO.setup(LEFT_ARM_SERVO, GPIO.OUT)
GPIO.setup(RIGHT_ARM_SERVO, GPIO.OUT)

ramp_pwm = GPIO.PWM(RAMP_SERVO, PWM_FREQ)
left_arm_pwm = GPIO.PWM(LEFT_ARM_SERVO, PWM_FREQ)
right_arm_pwm = GPIO.PWM(RIGHT_ARM_SERVO, PWM_FREQ)

ramp_pwm.start(7.5)
left_arm_pwm.start(7.5)
right_arm_pwm.start(7.5)

# ==============================
# STATE VARIABLES
# ==============================
ramp_down = False  # Start with ramp UP

# ==============================
# SERVO UTILITY FUNCTIONS
# ==============================
def set_servo(pwm, angle):
    """Convert angle (0-180) to PWM duty cycle"""
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)

# Arms
def open_arm(pwm):
    set_servo(pwm, ARM_OPEN_ANGLE)

def close_arm(pwm):
    set_servo(pwm, ARM_CLOSED_ANGLE)

# Ramp
def lower_ramp():
    global ramp_down
    set_servo(ramp_pwm, RAMP_DOWN_ANGLE)
    ramp_down = True

def raise_ramp():
    global ramp_down
    set_servo(ramp_pwm, RAMP_UP_ANGLE)
    ramp_down = False

# Initialize ramp to UP at startup
raise_ramp()
print("Ramp initialized to UP position.")

# ==============================
# MOTOR CONTROL FUNCTIONS
# ==============================
def move_forward():
    GPIO.output(MOTOR_PINS["LEFT_FRONT"], True)
    GPIO.output(MOTOR_PINS["RIGHT_FRONT"], True)
    GPIO.output(MOTOR_PINS["LEFT_BACK"], False)
    GPIO.output(MOTOR_PINS["RIGHT_BACK"], False)

def move_backward():
    GPIO.output(MOTOR_PINS["LEFT_BACK"], True)
    GPIO.output(MOTOR_PINS["RIGHT_BACK"], True)
    GPIO.output(MOTOR_PINS["LEFT_FRONT"], False)
    GPIO.output(MOTOR_PINS["RIGHT_FRONT"], False)

def turn_left():
    GPIO.output(MOTOR_PINS["LEFT_BACK"], True)
    GPIO.output(MOTOR_PINS["RIGHT_FRONT"], True)
    GPIO.output(MOTOR_PINS["LEFT_FRONT"], False)
    GPIO.output(MOTOR_PINS["RIGHT_BACK"], False)

def turn_right():
    GPIO.output(MOTOR_PINS["LEFT_FRONT"], True)
    GPIO.output(MOTOR_PINS["RIGHT_BACK"], True)
    GPIO.output(MOTOR_PINS["LEFT_BACK"], False)
    GPIO.output(MOTOR_PINS["RIGHT_FRONT"], False)

def stop_motors():
    for pin in MOTOR_PINS.values():
        GPIO.output(pin, False)

# ==============================
# PICKUP SEQUENCE (with step updates)
# ==============================
async def pickup_trash_sequence(websocket):
    global ramp_down
    if not ramp_down:
        await websocket.send(json.dumps({"status": "error", "message": "Ramp must be DOWN before pickup"}))
        return

    await websocket.send(json.dumps({"status": "progress", "step": "Starting pickup sequence"}))

    # Right Arm
    await websocket.send(json.dumps({"status": "progress", "step": "Closing Right Arm"}))
    close_arm(right_arm_pwm)
    time.sleep(0.8)

    await websocket.send(json.dumps({"status": "progress", "step": "Raising Ramp"}))
    raise_ramp()
    time.sleep(1)

    await websocket.send(json.dumps({"status": "progress", "step": "Lowering Ramp"}))
    lower_ramp()
    time.sleep(0.8)

    await websocket.send(json.dumps({"status": "progress", "step": "Opening Right Arm"}))
    open_arm(right_arm_pwm)
    time.sleep(0.5)

    # Left Arm
    await websocket.send(json.dumps({"status": "progress", "step": "Closing Left Arm"}))
    close_arm(left_arm_pwm)
    time.sleep(0.8)

    await websocket.send(json.dumps({"status": "progress", "step": "Raising Ramp"}))
    raise_ramp()
    time.sleep(1)

    await websocket.send(json.dumps({"status": "progress", "step": "Lowering Ramp"}))
    lower_ramp()
    time.sleep(0.8)

    await websocket.send(json.dumps({"status": "progress", "step": "Opening Left Arm"}))
    open_arm(left_arm_pwm)
    time.sleep(0.5)

    await websocket.send(json.dumps({"status": "complete", "message": "Pickup sequence complete"}))
    print("Pickup sequence complete.")

# ==============================
# SAFETY
# ==============================
def emergency_stop():
    stop_motors()
    raise_ramp()
    open_arm(left_arm_pwm)
    open_arm(right_arm_pwm)
    print("Emergency Stop Activated!")

# ==============================
# CAMERA STREAM (Flask)
# ==============================
app = Flask(__name__)
camera = cv2.VideoCapture(0)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

Thread(target=lambda: app.run(host='0.0.0.0', port=8081, threaded=True), daemon=True).start()

# ==============================
# WEBSOCKET SERVER
# ==============================
async def handle_client(websocket, path):
    print("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)
            command = data.get("command")
            print(f"Received command: {command}")

            # Motor commands
            if command == "move_forward":
                move_forward()
            elif command == "move_backward":
                move_backward()
            elif command == "turn_left":
                turn_left()
            elif command == "turn_right":
                turn_right()
            elif command == "stop_motors":
                stop_motors()
            # Ramp commands
            elif command == "lower_ramp":
                lower_ramp()
            elif command == "raise_ramp":
                raise_ramp()
            # Pickup sequence
            elif command == "pickup_trash_sequence":
                await pickup_trash_sequence(websocket)
            # Safety
            elif command == "emergency_stop":
                emergency_stop()
            else:
                print("Unknown command:", command)

            await websocket.send(json.dumps({"status": "ok", "command": command}))
    except websockets.ConnectionClosed:
        print("Client disconnected")

start_server = websockets.serve(handle_client, "0.0.0.0", 8080)

try:
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
except KeyboardInterrupt:
    print("Shutting down...")
    stop_motors()
    raise_ramp()
    open_arm(left_arm_pwm)
    open_arm(right_arm_pwm)
    GPIO.cleanup()