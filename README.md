# Dobot + YOLO9 Tic-Tac-Toe (Tic-Tac-Tao) — README

This repository contains instructions and example code to use a YOLO9 object detector together with a Dobot robot (e.g., Dobot Magician) to play a physical Tic‑Tac‑Toe (Tic‑Tac‑Tao) game. The system detects X and O tokens on the board using a camera + YOLO9, maps detections to board cells via a perspective transform, runs game logic, and commands the Dobot to place pieces or press buttons to play.

This README covers:
- Hardware & software requirements
- Installation and setup
- Camera & robot calibration
- Detection → board mapping
- Game loop and robot commands
- Running the demo and troubleshooting
- Safety notes and license

---

## Requirements

Hardware
- Dobot Magician (or similar Dobot) with appropriate end-effector (suction or gripper)
- USB/serial connection from host PC to Dobot
- USB camera (or other camera) pointed at the Tic‑Tac‑Toe board (overhead recommended)
- Physical board and tokens (X and O)

Software
- OS: Linux / macOS / Windows (instructions assume Linux)
- Python 3.8+
- PyTorch (compatible with your CUDA / CPU)
- OpenCV (opencv-python)
- YOLO9 model (weights .pt file)
- pydobot (or Dobot SDK) for controlling the robot
- ultralytics (or the YOLO9 repo you use) — adjust commands if you use a different API

Example Python packages (requirements.txt):
```
torch>=1.12
opencv-python
numpy
pydobot
ultralytics   # if using Ultralytics-style API for YOLO
```

---

## Installation

1. Clone the repo (or copy files into your project):
   git clone <your-repo-url>

2. Create and activate virtual environment:
   python -m venv venv
   source venv/bin/activate  # (Windows: venv\Scripts\activate)

3. Install dependencies:
   pip install -r requirements.txt

4. Get YOLO9 weights:
   - If using a pre-trained model for distinguishing X and O, download `yolov9_xo.pt`.
   - Or train on custom data (see "Training" below).

5. Connect Dobot and ensure pydobot (or your SDK) can open the serial port.

---

## File layout (suggested)
- play.py                 — main loop (camera → detector → game → robot)
- detector.py             — YOLO9 wrapper (load model, run inference)
- calibrate_camera.py     — compute perspective transform / homography
- calibrate_robot.py      — map board coordinates to robot coordinates
- tic_tac_toe.py          — game rules / move selection (AI or human)
- robot_control.py        — Dobot control wrappers (move, pick/place)
- requirements.txt
- README.md

---

## Camera calibration & board mapping

1. Camera mounting: Mount camera pointing straight down so board cells form a near-planar grid.

2. Detect board corners:
   - Use a calibration routine (calibrate_camera.py) that asks you to click the four board corners in the camera image in a fixed order (top-left, top-right, bottom-right, bottom-left).
   - Save image points (pixels) and corresponding board coordinates (e.g., cell centers in board units).

3. Compute homography or perspective transform:
```python
import cv2
import numpy as np

# image_pts: 4x2 pixel coordinates from the camera
# board_pts: corresponding 4x2 coordinates in board frame (e.g., [0,0],[3,0],[3,3],[0,3])
H, _ = cv2.findHomography(np.array(image_pts), np.array(board_pts))
# To map detection pixel to board coordinate:
board_pt = cv2.perspectiveTransform(np.array([[[x_pixel, y_pixel]]]), H)[0][0]
```

4. Map board coordinate to discrete cell index:
- If board coordinates range [0..3] in both axes for a 3x3 board, floor values to get cell indices (0..2).
- Apply small margin checks to avoid ambiguous detections.

---

## Robot calibration (image/board → robot coordinates)

1. Define robot coordinates for each board cell by moving the Dobot end-effector to each cell center and recording the pose (x, y, z, orientation).
2. Save a mapping table `cell_index -> robot_pose`.
3. Optionally compute an affine transform from board frame (2D) to robot XY coordinates if board is planar and Dobot base is fixed:
```python
# board_pts_2d and robot_xy correspondences (min 3 pairs)
A, _ = cv2.estimateAffine2D(np.array(board_pts_2d), np.array(robot_xy))
# Map: robot_xy = A * [board_x, board_y, 1]
```
4. Always test the mapping with slow, safe movements.

---

## YOLO9 detection approach

- Train/detect two classes: `X` and `O` (or detect markers that represent tokens).
- Use an input pipeline that gets an image frame, runs inference, filters by confidence and NMS, and returns detections with bounding boxes and class labels.

Example usage (detector.py):
```python
from ultralytics import YOLO
model = YOLO('yolov9_xo.pt')

results = model.predict(source=image, imgsz=640, conf=0.35)
# parse results to get boxes and class ids
```

- For robustness:
  - Use augmentation at train time.
  - Include various lighting/backgrounds.
  - Add classes for board artifacts (optional).

---

## Game logic

- tic_tac_toe.py contains:
  - Board state representation (3x3 matrix).
  - Turn management (which side is robot/human).
  - Move validity checks.
  - A simple AI (minimax) or heuristic to choose the robot move.

Simple flow:
1. Poll detections, update board state.
2. Check for a winner or a draw.
3. If it's robot's turn:
   - Compute move (choose empty cell).
   - Call robot_control.place_token(cell, token_type).
4. Repeat.

---

## Robot control (pydobot example)

Example robot_control.py (concept):
```python
from pydobot import Dobot

dobot = Dobot(port='/dev/ttyUSB0')

def move_to_pose(pose, speed=50):
    x, y, z, r = pose
    dobot.move_to(x, y, z, r, wait=True)

def pick_and_place(from_cell_pose, to_cell_pose):
    move_to_pose([from_cell_pose.x, from_cell_pose.y, safe_z, 0])
    # lower, actuate gripper/suction
    move_to_pose([from_cell_pose.x, from_cell_pose.y, pick_z, 0])
    # turn on suction or close gripper
    move_to_pose([from_cell_pose.x, from_cell_pose.y, safe_z, 0])
    move_to_pose([to_cell_pose.x, to_cell_pose.y, safe_z, 0])
    move_to_pose([to_cell_pose.x, to_cell_pose.y, place_z, 0])
    # release
    move_to_pose([to_cell_pose.x, to_cell_pose.y, safe_z, 0])
```

If you only need the Dobot to place tokens (not pick from a pile), adjust flows accordingly. If you use an electromagnet or servo, add commands to toggle it.

---

## Example run

1. Calibrate camera:
   python calibrate_camera.py --camera 0 --out camera_calib.json

2. Calibrate robot mapping:
   python calibrate_robot.py --port /dev/ttyUSB0 --camera_calib camera_calib.json --out robot_map.json

3. Start the game loop (robot plays O, human plays X):
   python play.py --camera 0 --model yolov9_xo.pt --port /dev/ttyUSB0 \
     --camera_calib camera_calib.json --robot_map robot_map.json --robot_token O

Flags you might want:
- --debug (shows frames and overlays)
- --simulate (runs without commanding robot)
- --save-frames / --log

---

## Training YOLO9 on X/O tokens (brief)

1. Collect images of the board with X and O tokens in different positions, lighting, and backgrounds.
2. Annotate with your tool (LabelImg, Roboflow). Classes: X, O.
3. Split train/val and export to YOLO format.
4. Train:
   yolov9 train data=data.yaml model=yolov9s.pt epochs=50 imgsz=640
5. Export/choose best weights `yolov9_xo.pt`.

---

## Safety

- Always test robot motions in `--simulate` or with the robot powered off (dry-run) before commanding real moves.
- Keep an emergency stop available and set safe speeds.
- Keep hands clear while the Dobot is moving.
- Use small, safe z-heights during testing.

---

## Troubleshooting

- Detections drift / misclassifications:
  - Re-train with more images and augmentations.
  - Increase confidence threshold.
  - Improve camera lighting and mounting stability.

- Mapping errors:
  - Re-run camera and robot calibration.
  - Use more calibration points and verify transforms.

- Dobot connection problems:
  - Verify serial port and permissions (on Linux, add your user to dialout or use sudo for testing).
  - Confirm Dobot firmware version and that pydobot supports it.

---

## Notes & tips

- Prefer an overhead camera and rigidly mount the board to minimize recalibration.
- If tokens are small, consider adding a colored marker to increase detection reliability.
- Use homography to map any board tilt; this avoids retraining for small perspective changes.

---

## License

Add your preferred license here (e.g., MIT). Example: MIT License — see LICENSE file.

---

If you'd like, I can:
- Generate a ready-to-run `play.py` + helper modules for your repository,
- Create a sample `requirements.txt`, or
- Produce a calibration UI script (camera corner clicker) and a robot-mapping tool.

Tell me which pieces you want me to scaffold and whether your Dobot uses `pydobot` or another SDK, and the repo name if you'd like me to add files directly.
