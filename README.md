TrashBot Backend
Install
bash
Copy
Edit
sudo apt-get update
sudo apt-get install python3-pip python3-opencv
git clone https://github.com/YOUR_USERNAME/trashbot-backend.git
cd trashbot-backend
pip3 install -r requirements.txt
requirements.txt

nginx
Copy
Edit
websockets
flask
opencv-python
RPi.GPIO
Run
bash
Copy
Edit
python3 robot_server.py
WebSocket: ws://<PI-IP>:8080

Camera: http://<PI-IP>:8081/video_feed

Commands
nginx
Copy
Edit
move_forward | move_backward | turn_left | turn_right
stop_motors | lower_ramp | raise_ramp
pickup_trash_sequence | emergency_stop