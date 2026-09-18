# Roboracer Sim Setup — macOS DOCKER

This gets your Mac from nothing to a running F1TENTH simulator with RViz.
Everything runs inside a Docker container, and you view the Linux desktop in
your web browser. Your code stays on your Mac's disk.

**Works on Apple Silicon (M1–M4) and Intel Macs.** You do **not** need
XQuartz, Parallels, or a VM.

**What you'll install:** Docker Desktop, VS Code + the Dev Containers extension.
**Time:** ~30–45 min, mostly waiting on downloads and builds.

---

## Step 1 — Set up an SSH key for GitHub

The repo is cloned over SSH. Skip this step if `ssh -T git@github.com`
already greets you by username.

```bash
ssh-keygen -t ed25519 -C "your-github-email@example.com"
```

Press Enter to accept the default location. A passphrase is optional.

```bash
eval "$(ssh-agent -s)"
touch ~/.ssh/config
open -e ~/.ssh/config
```

Paste this into the file that opens, then save and close:

```
Host github.com
  AddKeysToAgent yes
  UseKeychain yes
  IdentityFile ~/.ssh/id_ed25519
```

```bash
ssh-add --apple-use-keychain ~/.ssh/id_ed25519
pbcopy < ~/.ssh/id_ed25519.pub
```

On github.com: **your profile picture → Settings → SSH and GPG keys →
New SSH key**, paste with `Cmd+V`, then save.

Test it (type `yes` if asked about the fingerprint):

```bash
ssh -T git@github.com     # should say "Hi <your-username>!"
```

---

## Step 2 — Clone the repo and make your branch

```bash
cd ~
git clone git@github.com:Roboracer-Purdue/roboracer-purdue.git
cd roboracer-purdue
git checkout Dockertest
git checkout -b <FirstName_LastName_ws>
git push -u origin <FirstName_LastName_ws>
```

Always branch off `Dockertest`, **not** `main`. That branch has the working
dev container setup.

---

## Step 3 — Install Docker Desktop

1. Check your chip: run `uname -m` in Terminal.
   - `arm64` → download **Docker Desktop for Mac – Apple Silicon**
   - `x86_64` → download **Docker Desktop for Mac – Intel chip**
2. Install it and open it once. Wait until the whale icon in the menu bar
   stops animating. You can skip the sign-in and survey.
3. **Settings → Resources:** give it at least **4 CPUs and 8 GB RAM**.
4. Verify:
   ```bash
   docker run --rm hello-world
   ```

---

## Step 4 — Install VS Code + Dev Containers

1. Install VS Code from code.visualstudio.com.
2. In VS Code, press `Cmd+Shift+X`, search for **Dev Containers**
   (by Microsoft), and install it.

---

## Step 5 — Open the repo in the container

1. VS Code → **File → Open Folder…** → select the **`roboracer-purdue`** folder
   itself (the one containing `.devcontainer/`), not a folder above it.
2. Click **Reopen in Container** in the popup (or `Cmd+Shift+P` →
   **Dev Containers: Reopen in Container**).
3. The first build takes 5–15 minutes. Let it finish.

> ⚠️ **If VS Code asks you to pick a container template, OS, or features —
> cancel.** It means you opened the wrong folder. Go back and open
> `roboracer-purdue` directly.

When it's done, the bottom-left corner of VS Code says
**Dev Container: RoboRacer ROS Dev**. Every VS Code terminal now runs
inside the container.

---

## Step 6 — Build the sim workspace

In a VS Code terminal (`Ctrl+`` ` or **Terminal → New Terminal**):

```bash
cd /workspaces/roboracer-purdue/roboracer_ws
./setup.sh
```

This takes about 5–10 minutes. It should finish with:

```
f110_gym: OK
f1tenth_gym_ros: OK
Setup complete.
```

**Close that terminal and open a new one** so the environment loads.

---

## Step 7 — Open the desktop

1. In **Chrome or Safari** (not VS Code's built-in preview), go to:
   **http://localhost:6080/vnc.html**
2. Click **Connect**. Password: **`vscode`**
3. You'll see an empty Linux desktop. That's correct: it's where RViz
   will appear.

---

## Step 8 — Run the sim

In a VS Code terminal:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz opens **in the browser tab** with the car on the Levine map. Give it
10–20 seconds.

In a **second** VS Code terminal, drive the car:

```bash
cd /workspaces/roboracer-purdue/roboracer_ws
python3 scripts/key_drive.py      # w/s = speed, a/d = steer, space = stop
```

Keep the `key_drive` terminal focused while driving.

Tip: run all commands in VS Code terminals, not the desktop's own
terminal. Windows still open in the browser, and copy-paste works normally.

---

## Daily use

**Stop:** close VS Code. The container stops; nothing is lost.

**Resume:**
1. Open Docker Desktop and wait for it to start.
2. Open `roboracer-purdue` in VS Code → **Reopen in Container** (takes seconds).
3. Open `http://localhost:6080/vnc.html` and launch the sim as in Step 8.

**Save your work to GitHub:**
```bash
cd /workspaces/roboracer-purdue
git add <files you changed>
git commit -m "describe what you changed"
git push
```

> ⚠️ **"Rebuild Container" wipes the sim install** (your code is safe; it
> lives on your Mac). After a rebuild, run `./setup.sh` again (Step 6).
> Plain **Reopen in Container** does not need this.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Permission denied (publickey)` when cloning | Redo Step 1. Check `ssh -T git@github.com` works first. |
| `fatal: not a git repository` | You're not inside `roboracer-purdue`. `cd` into it first. |
| VS Code asks for a container template | Wrong folder opened. Open `roboracer-purdue` itself (Step 5). |
| `localhost:6080` is blank | Use `http://localhost:6080/vnc.html` in Chrome/Safari, not the VS Code preview. |
| Desktop won't load at all | In a VS Code terminal: `bash .devcontainer/start-vnc.sh` from `/workspaces/roboracer-purdue`, then reload the page. |
| RViz doesn't appear / `GLXContext` errors | Run `echo $DISPLAY`. It must print `:99`. If not, `export DISPLAY=:99` and relaunch. |
| `Package 'f1tenth_gym_ros' not found` | Open a new terminal (it loads the workspace). If it persists, rerun `./setup.sh`. |
| `setup.sh` says `f1tenth_gym_ros is missing or empty` | Run `git pull` on your branch, then ask a lead. |
| `setup.sh` says a conda env is active | Run `conda deactivate`, then retry. |
| Everything is very slow | Raise Docker Desktop's CPU/RAM (Step 3). |

Still stuck? Post the **full error text** (not a screenshot of part of it)
in the club channel.
