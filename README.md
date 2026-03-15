# ROB515 Final Project - Dual-Arm Block Transfer

Two Hiwonder ArmPi manipulators coordinate a mid-air green block handoff.

- **Host Pi (MartyRobbins)**: Has camera, picks up green block, holds it 10cm above origin
- **Sub Pi (PatsyCline)**: Grabs block with gripper rotated 90deg, stacks it at origin

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
