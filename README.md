# ROB515 Final Project - Dual-Arm Block Transfer

Two Hiwonder ArmPi manipulators coordinate a mid-air green block handoff.

- **Host Pi (MartyRobbins)**: Has camera, detects all blocks, picks up green, holds at handoff
- **Sub Pi (PatsyCline)**: Grabs block with gripper rotated 90deg, stacks it on blue block

## How It Works

### Overview

The host arm uses its camera to detect three colored blocks (red, green, blue) on the localization mat. It picks up the green block, moves it to a handoff position above the red block at z=25cm, and holds it there. The sub arm then grabs the block from the host with its gripper rotated 90 degrees (gripping top/bottom instead of sides), and places it on top of the blue block.

### Coordinate Transform

The two arms sit on opposite sides of the mat, mirrored across the crosshair (center). The crosshair is at y=20 in each arm's local coordinate system. The transform from host to sub coordinates is:

- `sub_x = -host_x` (left/right is flipped)
- `sub_y = 40 - host_y` (mirrored across crosshair at y=20)
- `sub_z = host_z` (height is the same)

Example: if the red block is at `(x=-7, y=20)` for the host, it's at `(x=7, y=20)` for the sub.

### Communication

The two Pi's communicate via simple HTTP POST signals on port 9090. The host sends:
- `ready_to_grab` - includes handoff position and blue block position (pre-transformed to sub coords)
- `released` - host has opened its gripper

The sub sends:
- `grab_confirmed` - sub has closed its gripper on the block

### Sequence

1. Host detects all 3 blocks (waits until green, red, and blue are all visible)
2. Host picks up green block from the mat
3. Host moves to handoff position (above red block, z=25cm)
4. Host signals sub with `ready_to_grab` + block positions in sub coordinates
5. Sub moves to handoff position with gripper rotated 90deg
6. Sub closes gripper (top/bottom grip), sends `grab_confirmed`
7. Host opens gripper, sends `released`
8. Sub moves green block to blue block position and stacks it (z=1.5cm)
9. Both arms return to home position

### Gripper Orientation

- Host grips the block from the **sides** (servo2 = 500, default)
- Sub grips from **top and bottom** (servo2 = 162, ~120deg rotated) so the grippers don't collide

## Running

Stop the ArmPi service first (it locks the camera):
```bash
ssh marty "sudo systemctl stop armpi.service && sudo systemctl disable armpi.service"
```

Start the sub first (it waits for the host signal), then the host:
```bash
# On PatsyCline:
sudo python3 /home/pi/finalProject/sub_transfer.py

# On MartyRobbins:
sudo python3 /home/pi/Project/host_transfer.py
```

Place all 3 blocks (red, green, blue) on the mat visible to MartyRobbins's camera. The host will show a camera window on VNC with block positions labeled.

## SSH Setup

### 1. Get the shared SSH key

```bash
git clone https://github.com/singh33-j/ROB515_Project.git
cp ROB515_Project/ssh_keys/id_armpi_project ~/.ssh/
cp ROB515_Project/ssh_keys/id_armpi_project.pub ~/.ssh/
chmod 600 ~/.ssh/id_armpi_project
```

### 2. Add SSH config

Add the following to `~/.ssh/config`:

```
Host marty
    HostName MartyRobbins.engr.oregonstate.edu
    User pi
    IdentityFile ~/.ssh/id_armpi_project
    LocalForward 8080 localhost:8080
    LocalForward 9030 localhost:9030

Host patsy
    HostName PatsyCline.engr.oregonstate.edu
    User pi
    IdentityFile ~/.ssh/id_armpi_project
    LocalForward 8081 localhost:8080
    LocalForward 9031 localhost:9030
```

### 3. Connect

```bash
ssh marty    # Host Pi (MartyRobbins)
ssh patsy    # Sub Pi (PatsyCline)
```

Port forwarding is automatic:
- **MartyRobbins**: localhost:8080 (video stream), localhost:9030 (RPC)
- **PatsyCline**: localhost:8081 (video stream), localhost:9031 (RPC)

### 4. Mount Pi filesystem locally (optional)

```bash
# Install sshfs if needed: sudo apt install sshfs
mkdir -p ~/pi-mount
sshfs pi@MartyRobbins.engr.oregonstate.edu:/home/pi ~/pi-mount
```

After editing files via SSHFS, clear Python cache or it may run stale code:
```bash
ssh marty "find /home/pi/ArmPi -name '__pycache__' -exec rm -rf {} +"
```
