import cv2
import json
import time
import numpy as np
from collections import namedtuple, Counter
import argparse

# =============================
# ROBOT CONFIG & IO
# =============================
RUN_ROBOT = False  # set True when you want the robot to move
PORT = "/dev/ttyACM0"  # change if needed

# =============================
# ROBOT CONFIG & IO
# =============================
RUN_ROBOT = False  # set True when you want the robot to move
PORT = "/dev/ttyACM0"  # change if needed

# Speeds, heights and motion style
HOVER_Z_OFFSET = 40.0
PICK_PRESS_MM = 1.0
DROP_PRESS_MM = 2.0
SETTLE_SEC = 0.25
SLOW_SPEED = (60, 60)
FAST_SPEED = (100, 100)

# ✅ camera-home you measured (no rotation)
HOME = {"mode": 0, "x": 245.101806640625, "y": 0.7384543418884277, "z": 19.78293228149414, "r": 0.0}

# use same as intermediate hover point
INTERMEDIATE = HOME.copy()

def zero_r(p):
    q=dict(p); q["r"]=0.0; return q

# =============================
# PALLET A / PALLET B
# =============================
# Pallet A: 3×3 cell centers (camera view: row 0=top, col 0=left)
PALLET_A_GRID = {
    (0, 0): {"x": 326.4212341308594, "y": 25.79114532470703, "z": -45.471961975097656, "r": 0.0},  # (1,1)
    (1, 0): {"x": 307.7861022949219, "y": 28.36910057067871, "z": -44.78849792480469, "r": 0.0},  # (2,1)
    (2, 0): {"x": 286.1927185058594, "y": 26.416584014892578, "z": -42.40446090698242, "r": 0.0},  # (3,1)
    (0, 1): {"x": 327.3711242675781, "y": 5.295122146606445, "z": -45.41124725341797, "r": 0.0},  # (1,2)
    (1, 1): {"x": 309.05072021484375,"y": 4.974523067474365, "z": -43.895790100097656, "r": 0.0},  # (2,2)
    (2, 1): {"x": 287.3177490234375, "y": 8.101576805114746, "z": -42.36698532104492, "r": 0.0},  # (3,2)
    (0, 2): {"x": 327.08251953125, "y": -15.264838218688965, "z": -45.46723937988281, "r": 0.0},  # (1,3)
    (1, 2): {"x": 305.7072448730469, "y": -14.25012493133545, "z": -43.98912048339844, "r": 0.0},  # (2,3)
    (2, 2): {"x": 288.9286804199219,"y": -13.5654878616333, "z": -45.035884857177734, "r": 0.0},  # (3,3)
}

# Pallet B: storage rows
# GREEN = X, YELLOW = O
PALLET_B_COLOR_GREEN = [  # Green blocks (X)
    {"x": 268.88775634765625, "y": -90.33350372314453, "z": -44.90547332763672, "r": 0.0},  # green1
    {"x": 292.0614318847656, "y": -90.0417251586914, "z": -44.791019439697266, "r": 0.0},  # green2
    {"x": 309.9480895996094, "y": -90.0417251586914, "z": -44.85551452636719, "r": 0.0},  # green3
    {"x": 329.80242919921875, "y": -90.0417251586914, "z": -44.953033447265625, "r": 0.0},  # green4
]

PALLET_B_COLOR_YELLOW = [  # Yellow blocks (O)
    {"x": 268.7389831542969, "y": -71.92486572265625, "z": -44.953033447265625, "r": 0.0},  # yellow1
    {"x": 289.2863464355469, "y": -71.92486572265625, "z": -44.953033447265625, "r": 0.0},  # yellow2
    {"x": 310.5854797363281, "y": -71.92486572265625, "z": -44.953033447265625, "r": 0.0},  # yellow3
    {"x": 328.5643310546875, "y": -71.92486572265625, "z": -44.953033447265625, "r": 0.0},  # yellow4
]

# =============================
# VISION + GAME SETTINGS
# =============================
CAM_INDEX = 1
BOARD_SIZE_PX = 600
GRID_N = 3
CELL_PAD = 0.10
CORE_PAD = 0.20
THRESH_X = 0.04  # green
THRESH_O = 0.06  # yellow
HSV_MARGIN = (14, 90, 90)
CALIB_FILE = "penpaper_calib.json"

# 🟩 Green = X (human or robot), 🟨 Yellow = O
HSV_X_DEFAULT = (np.array([35, 70, 50], np.uint8), np.array([85, 255, 255], np.uint8))  # green
HSV_O_DEFAULT = (np.array([20, 80, 80], np.uint8), np.array([35, 255, 255], np.uint8))  # yellow

CHECK_DELAY_SEC = 10.0
STABLE_SAMPLES = 8
STABLE_SPAN_SEC = 1.2

# =============================
# TIMER SETTINGS
# =============================
GAME_TIME_LIMIT_SEC = 300  # 5 minutes total game time
MOVE_TIME_LIMIT_SEC = 60   # 1 minute per move
SHOW_TIMER = True

HSVRange = namedtuple("HSVRange", ["lower", "upper"])

# =============================
# ROBOT IO
# =============================
device = None

def robot_connect():
    global device
    if not RUN_ROBOT:
        return
    if device is not None:  # <-- already connected: avoid COM access denied
        return
    import pydobot
    from pydobot.dobot import MODE_PTP
    HOME["mode"] = int(MODE_PTP.MOVJ_XYZ)
    INTERMEDIATE["mode"] = int(MODE_PTP.MOVJ_XYZ)
    device = pydobot.Dobot(port=PORT)
    device.home()  # firmware home
    device.speed(*FAST_SPEED)
    device.move_to(**HOME)  # go to YOUR camera home right away
    device.suck(False)

def robot_close():
    global device
    if device is not None:
        device.suck(False)
        device.close()
        device = None

def moveto(pose_dict):
    if not RUN_ROBOT:
        return
    device.move_to(**zero_r(pose_dict))

def approach_pose(xyzr, dz=HOVER_Z_OFFSET):
    p = zero_r(xyzr)
    return {"mode": HOME["mode"], "x": p["x"], "y": p["y"], "z": p["z"] + dz, "r": 0.0}

def linear_pose(x, y, z):
    return {"mode": 1, "x": x, "y": y, "z": z, "r": 0.0}

def pick_block(xyzr):
    if not RUN_ROBOT:
        return
    moveto(INTERMEDIATE)
    moveto(approach_pose(xyzr))
    device.speed(*SLOW_SPEED)
    p = zero_r(xyzr)
    moveto(linear_pose(p["x"], p["y"], p["z"] - PICK_PRESS_MM))
    device.suck(True)
    time.sleep(0.6)
    moveto(approach_pose(xyzr))
    device.speed(*FAST_SPEED)
    moveto(INTERMEDIATE)

def drop_block(xyzr):
    if not RUN_ROBOT:
        return
    moveto(approach_pose(xyzr))
    device.speed(*SLOW_SPEED)
    p = zero_r(xyzr)
    moveto(linear_pose(p["x"], p["y"], p["z"]))
    time.sleep(SETTLE_SEC)
    moveto(linear_pose(p["x"], p["y"], p["z"] - DROP_PRESS_MM))
    device.suck(False)
    time.sleep(0.2)
    moveto(approach_pose(xyzr))
    device.speed(*FAST_SPEED)
    moveto(INTERMEDIATE)

robot_stock_idx = {"GREEN": 0, "YELLOW": 0}

def robot_place_at_cell(cell_rc, color):
    """ color: 'GREEN' or 'YELLOW' """
    global robot_stock_idx
    stack = PALLET_B_COLOR_GREEN if color == "GREEN" else PALLET_B_COLOR_YELLOW
    idx = robot_stock_idx[color] % len(stack)
    src = stack[idx]
    dst = PALLET_A_GRID[cell_rc]
    print(f"[robot] pick {color} stock #{idx} → place A cell {cell_rc}")
    pick_block(src)
    drop_block(dst)
    robot_stock_idx[color] += 1

# =============================
# VISION HELPERS
# =============================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def make_range_from_hsv(hsv, margin=HSV_MARGIN):
    h, s, v = [int(x) for x in hsv]
    dh, ds, dv = margin
    lo = np.array([clamp(h-dh, 0,179), clamp(s-ds,0,255), clamp(v-dv,0,255)], np.uint8)
    hi = np.array([clamp(h+dh, 0,179), clamp(s+ds,0,255), clamp(v+dv,0,255)], np.uint8)
    return HSVRange(lo, hi)

def save_calib(corners, hsv_x, hsv_o, path=CALIB_FILE):
    data = {
        "corners": [list(map(float, p)) for p in corners],
        "hsv_x_low": hsv_x.lower.tolist(),
        "hsv_x_high": hsv_x.upper.tolist(),
        "hsv_o_low": hsv_o.lower.tolist(),
        "hsv_o_high": hsv_o.upper.tolist(),
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[✓] saved → {path}")

def load_calib(path=CALIB_FILE):
    with open(path, "r") as f:
        d = json.load(f)
    corners = [tuple(d["corners"][i]) for i in range(4)]
    hx = HSVRange(np.array(d["hsv_x_low"], np.uint8), np.array(d["hsv_x_high"], np.uint8))
    ho = HSVRange(np.array(d["hsv_o_low"], np.uint8), np.array(d["hsv_o_high"], np.uint8))
    print(f"[✓] loaded ← {path}")
    return corners, hx, ho

def homography_from_corners(corners):
    src = np.float32(corners)
    dst = np.float32([[0,0],[BOARD_SIZE_PX-1,0],[BOARD_SIZE_PX-1,BOARD_SIZE_PX-1],[BOARD_SIZE_PX-1,BOARD_SIZE_PX-1]])  # ^^^ DELIBERATE? No — this line was wrong. Keep original mapping:
    # I'll correct it below to keep your original behavior:
    dst = np.float32([[0,0],[BOARD_SIZE_PX-1,0],[BOARD_SIZE_PX-1,BOARD_SIZE_PX-1],[0,BOARD_SIZE_PX-1]])
    return cv2.getPerspectiveTransform(src, dst)

def warp_board(frame, H):
    return cv2.warpPerspective(frame, H, (BOARD_SIZE_PX, BOARD_SIZE_PX))

def largest_area_fraction(mask):
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return 0.0
    areas = stats[1:, cv2.CC_STAT_AREA]
    return float(areas.max()) / float(mask.shape[0] * mask.shape[1])

def classify_cell(cell_bgr, hsv_x, hsv_o):
    h, w = cell_bgr.shape[:2]
    dx, dy = int(w * CORE_PAD), int(h * CORE_PAD)
    x0, y0 = dx, dy
    x1, y1 = w - dx, h - dy
    core = cell_bgr if (x1 <= x0 or y1 <= y0) else cell_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(core, cv2.COLOR_BGR2HSV)
    mask_x = cv2.inRange(hsv, hsv_x.lower, hsv_x.upper)  # green (X)
    mask_o = cv2.inRange(hsv, hsv_o.lower, hsv_o.upper)  # yellow (O)
    k = np.ones((3,3), np.uint8)
    mask_x = cv2.morphologyEx(mask_x, cv2.MORPH_OPEN, k, iterations=1)
    mask_o = cv2.morphologyEx(mask_o, cv2.MORPH_OPEN, k, iterations=1)
    mask_x = cv2.erode(mask_x, k, iterations=1)
    mask_o = cv2.erode(mask_o, k, iterations=1)
    px = largest_area_fraction(mask_x)
    po = largest_area_fraction(mask_o)
    if px < THRESH_X and po < THRESH_O:
        return ' '
    if px >= po * 1.15 and px >= THRESH_X:
        return 'X'
    if po >= px * 1.15 and po >= THRESH_O:
        return 'O'
    if px >= THRESH_X and px > po:
        return 'X'
    if po >= THRESH_O and po > px:
        return 'O'
    return ' '

def extract_board_state(board_bgr, hsv_x, hsv_o):
    h, w = board_bgr.shape[:2]
    cw, ch = w//GRID_N, h//GRID_N
    pad_w, pad_h = int(cw*CELL_PAD), int(ch*CELL_PAD)
    state, overlay = [], board_bgr.copy()
    for r in range(GRID_N):
        row = []
        for c in range(GRID_N):
            x0, y0 = c*cw + pad_w, r*ch + pad_h  # FIX: removed stray ')' at the end of next line
            x1, y1 = (c+1)*cw - pad_w, (r+1)*ch - pad_h
            cell = board_bgr[y0:y1, x0:x1]
            label = classify_cell(cell, hsv_x, hsv_o)
            row.append(label)
            cv2.rectangle(overlay, (c*cw, r*ch), ((c+1)*cw-1, (r+1)*ch-1), (0,255,0), 1)
            cx, cy = c*cw + cw//2, r*ch + ch//2
            t = label if label!=' ' else '.'
            cv2.putText(overlay, t, (cx-12, cy+12), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(overlay, t, (cx-12, cy+12), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255,255,255), 2, cv2.LINE_AA)
        state.append(row)
    return state, overlay

def stable_read_state(cap, H, hsv_x, hsv_o, samples=STABLE_SAMPLES, span_sec=STABLE_SPAN_SEC):
    per_sample_states = []
    delay = span_sec / max(1, samples - 1)
    board = None
    for i in range(samples):
        ret, frame = cap.read()
        if not ret:
            break
        board = warp_board(frame, H)
        state, _ = extract_board_state(board, hsv_x, hsv_o)
        per_sample_states.append(state)
        if i < samples - 1:
            time.sleep(delay)
    if not per_sample_states:
        return None, None
    mode_state = []
    for r in range(GRID_N):
        row = []
        for c in range(GRID_N):
            votes = [per_sample_states[k][r][c] for k in range(len(per_sample_states))]
            lab = Counter(votes).most_common(1)[0][0]
            row.append(lab)
        mode_state.append(row)
    _, overlay = extract_board_state(board, hsv_x, hsv_o)
    return mode_state, overlay

def print_board(state):
    print("\nBoard:")
    for r in range(GRID_N):
        print(" " + " | ".join(state[r]))
        if r < GRID_N-1:
            print("---+---+---")

# =============================
# GAME LOGIC (UPDATED)
# =============================
def winner(board):
    """Return 'X', 'O', or None if no winner yet."""
    win_patterns = [
        [(0, 0), (0, 1), (0, 2)],  # rows
        [(1, 0), (1, 1), (1, 2)],
        [(2, 0), (2, 1), (2, 2)],
        [(0, 0), (1, 0), (2, 0)],  # columns
        [(0, 1), (1, 1), (2, 1)],
        [(0, 2), (1, 2), (2, 2)],
        [(0, 0), (1, 1), (2, 2)],  # diagonals
        [(0, 2), (1, 1), (2, 0)]
    ]
    for pattern in win_patterns:
        a, b, c = pattern
        if board[a[0]][a[1]] != ' ' and \
           board[a[0]][a[1]] == board[b[0]][b[1]] == board[c[0]][c[1]]:
            return board[a[0]][a[1]]
    return None

def is_full(board):
    return all(cell != ' ' for row in board for cell in row)

def check_winner_state(board):
    """Return dict like {winner:'X'/'O'/'Tie', pattern: [(r,c),...]} or None if game not over."""
    win_patterns = [
        [(0, 0), (0, 1), (0, 2)],
        [(1, 0), (1, 1), (1, 2)],
        [(2, 0), (2, 1), (2, 2)],
        [(0, 0), (1, 0), (2, 0)],
        [(0, 1), (1, 1), (2, 1)],
        [(0, 2), (1, 2), (2, 2)],
        [(0, 0), (1, 1), (2, 2)],
        [(0, 2), (1, 1), (2, 0)]
    ]
    for pattern in win_patterns:
        a, b, c = pattern
        if board[a[0]][a[1]] and board[a[0]][a[1]] == board[b[0]][b[1]] == board[c[0]][c[1]]:
            return {"winner": board[a[0]][a[1]], "pattern": pattern}
    if is_full(board):
        return {"winner": "Tie", "pattern": None}
    return None

def minimax(board, depth, is_maximizing):
    """Pure minimax using the JS logic."""
    result = check_winner_state(board)
    if result:
        if result["winner"] == 'O':
            return 10 - depth
        if result["winner"] == 'X':
            return depth - 10
        if result["winner"] == 'Tie':
            return 0
    if is_maximizing:
        best_score = -float('inf')
        for r in range(3):
            for c in range(3):
                if board[r][c] == ' ':
                    board[r][c] = 'O'
                    score = minimax(board, depth + 1, False)
                    board[r][c] = ' '
                    best_score = max(score, best_score)
        return best_score
    else:
        best_score = float('inf')
        for r in range(3):
            for c in range(3):
                if board[r][c] == ' ':
                    board[r][c] = 'X'
                    score = minimax(board, depth + 1, True)
                    board[r][c] = ' '
                    best_score = min(score, best_score)
        return best_score

def pick_ai_move(board, ai='O', human='X'):
    """Find best move for AI using minimax logic from your JS code."""
    best_score = -float('inf')
    best_move = None
    for r in range(3):
        for c in range(3):
            if board[r][c] == ' ':
                board[r][c] = ai
                score = minimax(board, 0, False)
                board[r][c] = ' '
                if score > best_score:
                    best_score = score
                    best_move = (r, c)
    return best_move

def count_symbol(board, sym):
    return sum(1 for row in board for cell in row if cell == sym)

def diff_cells(a, b):
    d = []
    for r in range(GRID_N):
        for c in range(GRID_N):
            if a[r][c] != b[r][c]:
                d.append((r,c,a[r][c],b[r][c]))
    return d

# =============================
# UI + MAIN
# =============================
class UI:
    def __init__(self):
        self.pending = None
        self.corners = []
        self.hsv_x = HSVRange(*HSV_X_DEFAULT)
        self.hsv_o = HSVRange(*HSV_O_DEFAULT)
        self.last = None

ui = UI()

def on_mouse(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if ui.pending == 'corners':
        ui.corners.append((x,y))
        print(f"[+] corner {len(ui.corners)}/4 @ ({x},{y})")
        if len(ui.corners)==4:
            ui.pending = None
            print("[i] corners set. the game will auto-run.")
    elif ui.pending in ('x','o') and ui.last is not None:
        x0, y0 = max(0,x-5), max(0,y-5)
        x1, y1 = min(ui.last.shape[1]-1, x+5), min(ui.last.shape[0]-1, y+5)
        patch = ui.last[y0:y1+1, x0:x1+1]
        hsv_mean = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1,3).mean(axis=0)
        rng = make_range_from_hsv(hsv_mean)
        if ui.pending == 'x':
            ui.hsv_x = rng  # FIX: removed stray ']' at end of f-string
            print(f"[✓] sampled X (green) → {rng.lower.tolist()}..{rng.upper.tolist()}")
        else:
            ui.hsv_o = rng  # FIX: removed stray ']' at end of f-string
            print(f"[✓] sampled O (yellow) → {rng.lower.tolist()}..{rng.upper.tolist()}")
        ui.pending = None

def ask_first_player():
    while True:
        ans = input("Who goes first? [h]uman or [r]obot: ").strip().lower()
        if ans in ("h", "human"):
            return "HUMAN"
        if ans in ("r", "robot"):
            return "ROBOT"
        print("Please type 'h' or 'r'.")

def start_after_calib(cap, first_player, last_board, symbols):
    H = homography_from_corners(ui.corners)
    state, overlay = stable_read_state(cap, H, ui.hsv_x, ui.hsv_o)
    if state is None:
        print("[!] Could not read board; will try again later.")
        return "EXPECT_HUMAN", [[' ']*GRID_N for _ in range(GRID_N)], time.time() + CHECK_DELAY_SEC, symbols
    cv2.imshow("Board", overlay)
    xcnt = count_symbol(state, 'X')
    ocnt = count_symbol(state, 'O')
    if first_player == "ROBOT":
        symbols["ai"], symbols["human"], symbols["locked"] = 'O', 'X', True
        print("[i] Robot goes first and will use YELLOW (O) this game.")
        if xcnt == 0 and ocnt == 0:
            mv = pick_ai_move([row[:] for row in state], ai=symbols["ai"], human=symbols["human"])
            if mv is None:
                print("[i] No valid AI move on empty board? Ending.")
                return "DONE", state, None, symbols
            r, c = mv
            print(f"[→] AI ({symbols['ai']}) move: row={r}, col={c}")
            if RUN_ROBOT:
                robot_place_at_cell((r, c), color="YELLOW")
            state[r][c] = symbols["ai"]
            last_board = [row[:] for row in state]
            return "EXPECT_HUMAN", last_board, time.time() + CHECK_DELAY_SEC, symbols
        else:
            print("[!] Board not empty; switching to human-first flow.")
            return "EXPECT_HUMAN", [[' ']*GRID_N for _ in range(GRID_N)], time.time() + CHECK_DELAY_SEC, symbols
    print("[⏱] Human goes first. I will read the board in 10s.")
    return "EXPECT_HUMAN", [[' ']*GRID_N for _ in range(GRID_N)], time.time() + CHECK_DELAY_SEC, symbols

def main():
    first_player = ask_first_player()
    print(f"[📹] Opening camera index {CAM_INDEX}...")
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {CAM_INDEX}")
    # Get camera info
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[📹] Camera {CAM_INDEX} opened: {int(width)}x{int(height)} @ {fps}fps")
    print("[📹] This should be your USB camera (Alcor Micro Corp. USB 2.0 PC Camera)")
    WINDOW = "Camera"
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1024, 768)
    cv2.setMouseCallback(WINDOW, on_mouse)
    info = "r: ROI c: 4-corners x: sample GREEN(X) o: sample YELLOW(O) l: load s: save q: quit"
    print("[i] " + info)
    if RUN_ROBOT:
        print("[🏠] Moving robot to HOME position before ROI selection...")
        robot_connect()
        moveto(HOME)
    phase = "WAIT_CALIB"
    last_board = [[' ']*GRID_N for _ in range(GRID_N)]
    next_check_t = None
    symbols = {"ai": None, "human": None, "locked": False}
    
    # Timer variables
    game_start_time = None
    move_start_time = None
    current_player = first_player
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        ui.last = frame.copy()
        disp = frame.copy()
        
        # Timer display
        timer_text = ""
        if SHOW_TIMER and game_start_time is not None:
            elapsed = time.time() - game_start_time
            game_minutes = int(elapsed // 60)
            game_seconds = int(elapsed % 60)
            timer_text = f"Game: {game_minutes:02d}:{game_seconds:02d}"
            
            if move_start_time is not None:
                move_elapsed = time.time() - move_start_time
                move_minutes = int(move_elapsed // 60)
                move_seconds = int(move_elapsed % 60)
                remaining = max(0, MOVE_TIME_LIMIT_SEC - move_elapsed)
                rem_minutes = int(remaining // 60)
                rem_seconds = int(remaining % 60)
                timer_text += f" | Move: {move_minutes:02d}:{move_seconds:02d} | Left: {rem_minutes:02d}:{rem_seconds:02d}"
                
                # Check for move time limit
                if move_elapsed >= MOVE_TIME_LIMIT_SEC:
                    timer_text += " | TIME UP!"
        
        label = f"First: {first_player} robot={symbols['ai'] or '?'} | {current_player}'s turn"
        cv2.putText(disp, f"{info} {label}", (18,28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(disp, f"{info} {label}", (18,28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        
        if timer_text:
            cv2.putText(disp, timer_text, (18,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(disp, timer_text, (18,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        for pt in ui.corners:
            cv2.circle(disp, (int(pt[0]), int(pt[1])), 6, (0,255,255), -1)
        if len(ui.corners)==4:
            cv2.polylines(disp, [np.array(ui.corners, np.int32)], True, (0,255,255), 2)
        cv2.imshow(WINDOW, disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            roi = cv2.selectROI(WINDOW, frame, showCrosshair=True, fromCenter=False)
            x, y, w, h = [int(v) for v in roi]
            if w > 0 and h > 0:
                ui.corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                print("[✓] ROI set.")
            if RUN_ROBOT:
                moveto(HOME)
            phase, last_board, next_check_t, symbols = start_after_calib(cap, first_player, last_board, symbols)
        elif key == ord('c'):
            ui.corners = []; ui.pending = 'corners'; print("[i] click 4 corners TL→TR→BR→BL")
        elif key == ord('x'):
            ui.pending = 'x'; print("[i] click a GREEN stroke (X)")
        elif key == ord('o'):
            ui.pending = 'o'; print("[i] click a YELLOW block (O)")
        elif key == ord('l'):
            try:
                corners, hx, ho = load_calib()
                ui.corners, ui.hsv_x, ui.hsv_o = corners, hx, ho
                print("[✓] Calibration loaded.")
                if RUN_ROBOT:
                    moveto(HOME)
                phase, last_board, next_check_t, symbols = start_after_calib(cap, first_player, last_board, symbols)
            except Exception as e:
                print("[!] load failed:", e)
        elif key == ord('s'):
            if len(ui.corners)==4:
                save_calib(ui.corners, ui.hsv_x, ui.hsv_o)
        if len(ui.corners)==4 and next_check_t is not None and time.time() >= next_check_t:
            H = homography_from_corners(ui.corners)
            state, overlay = stable_read_state(cap, H, ui.hsv_x, ui.hsv_o)
            if state is None:
                print("[!] Camera read failed; retrying.")
                next_check_t = time.time() + CHECK_DELAY_SEC
                continue
            cv2.imshow("Board", overlay)
            w = winner(state)
            if w:
                print_board(state); print(f"[✓] Game over. Winner: {w}")
                phase = "DONE"; next_check_t = None; continue
            if is_full(state):
                print_board(state); print(f"[✓] Game over. Draw.")
                phase = "DONE"; next_check_t = None; continue
            if phase == "EXPECT_HUMAN":
                dx = count_symbol(state, 'X') - count_symbol(last_board, 'X')
                do = count_symbol(state, 'O') - count_symbol(last_board, 'O')
                changes = diff_cells(last_board, state)
                if len(changes) == 1 and ((dx == 1 and do == 0) or (dx == 0 and do == 1)):
                    human_mark = 'X' if dx == 1 else 'O'
                    if not symbols["locked"]:
                        symbols["human"] = human_mark
                        symbols["ai"] = 'O' if human_mark == 'X' else 'X'
                        symbols["locked"] = True
                    print(f"[↯] Detected human color = {symbols['human']}. Robot will play {symbols['ai']}.")
                    print_board(state)
                    mv = pick_ai_move([row[:] for row in state], ai=symbols["ai"], human=symbols["human"])
                    if mv is None:
                        print("[i] No valid AI moves."); phase="DONE"; next_check_t=None
                    else:
                        r, c = mv
                        print(f"[→] AI ({symbols['ai']}) move: row={r}, col={c}")
                        if RUN_ROBOT:
                            if symbols["ai"] == 'X':
                                robot_place_at_cell((r, c), color="GREEN")
                            else:
                                robot_place_at_cell((r, c), color="YELLOW")
                        state[r][c] = symbols["ai"]
                        last_board = [row[:] for row in state]
                        phase = "EXPECT_HUMAN"
                        next_check_t = time.time() + CHECK_DELAY_SEC
                        print(f"[⏱] Your turn. Reading in {CHECK_DELAY_SEC:.0f}s.")
                else:
                    last_board = [row[:] for row in state]
                    phase = "EXPECT_AI_REALIZE"
                    next_check_t = time.time() + CHECK_DELAY_SEC
                    print(f"[⏱] Please place the robot color in that cell. Checking again soon.")
            elif phase == "EXPECT_AI_REALIZE":
                dx = count_symbol(state, 'X') - count_symbol(last_board, 'X')
                do = count_symbol(state, 'O') - count_symbol(last_board, 'O')
                changes = diff_cells(last_board, state)
                if len(changes) == 1 and ( (symbols["ai"] == 'X' and dx == 1 and do == 0 and changes[0][3] == 'X') or (symbols["ai"] == 'O' and do == 1 and dx == 0 and changes[0][3] == 'O')):
                    print_board(state); print("[✓] AI move recognized on board.")
                    last_board = [row[:] for row in state]
                    phase = "EXPECT_HUMAN"
                    next_check_t = time.time() + CHECK_DELAY_SEC
                    print(f"[⏱] Your turn. Reading in {CHECK_DELAY_SEC:.0f}s.")
                else:
                    print("[…] Didn't see exactly one new AI mark. Will check again.")
                    next_check_t = time.time() + CHECK_DELAY_SEC
            elif phase == "DONE":
                next_check_t = None
    if RUN_ROBOT:
        moveto(HOME); robot_close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Tic-Tac-Toe Robot with Camera Control')
    parser.add_argument('--cam', type=int, default=1, help='Camera index (0=laptop cam, 1/2=USB cams)')
    parser.add_argument('--robot', action='store_true', help='Enable robot movement (requires RUN_ROBOT=True in code)')
    args = parser.parse_args()

    # Override camera index from command line
    CAM_INDEX = args.cam

    print(f"[📹] Using camera index: {CAM_INDEX}")
    if CAM_INDEX == 0:
        print("[📹] Camera 0: Built-in laptop camera")
    elif CAM_INDEX in [1, 2]:
        print("[📹] Camera 1/2: USB camera (Alcor Micro Corp. USB 2.0 PC Camera)")
    else:
        print(f"[📹] Camera {CAM_INDEX}: Custom camera index")

    if args.robot:
        RUN_ROBOT = True
        print("[🤖] Robot movement ENABLED")

    try:
        main()
    finally:
        if RUN_ROBOT:
            robot_close()
