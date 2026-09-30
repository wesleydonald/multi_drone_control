# Remote access to the lab workstation (`drones`)

The lab PC is a Dell Pro Max T2: Ubuntu 26.04, 24 cores, 60 GB RAM, RTX PRO 6000. It is reachable from anywhere through **Tailscale**, a private network between our machines. There is no port forwarding and no campus VPN. The network belongs to the lab Google account **mtrn204lab@gmail.com**; ask Wesley or the supervisor for access.

## Connect

1. Install Tailscale on your computer (`curl -fsSL https://tailscale.com/install.sh | sh`, or the app on Windows/Mac) and sign in with the account you were invited with.
2. Run `ssh <your-username>@drones`. The first time, a browser check may ask you to approve.
3. Put long jobs inside `tmux` (`tmux new -s myjob`; detach with `Ctrl-b d`; come back with `tmux attach -t myjob`). They then survive a dropped connection or a closed laptop.

SSH works with the monitor off and nobody logged in at the desk. It does **not** work if the PC is switched off. The PC never sleeps, and the BIOS powers it back on at 07:00, so **please don't shut it down**. Log out instead.

## Add a user (admin: whoever holds the lab account)

1. **Tailscale.** Pick one:
   - **Invite** (they see the lab network): login.tailscale.com/admin/users → Invite users.
   - **Share this machine only**: Machines → `drones` → "…" → Share.
2. **Linux account on the PC** (one per person; don't share the `drones` login):
   ```bash
   sudo adduser alice                 # their own home, files and password
   sudo usermod -aG docker alice      # only if they need Docker
   ```
3. **Allow them in over Tailscale SSH.** The PC carries the tag `tag:lab`: Machines → `drones` → "…" → Edit ACL tags. Then, in Access controls, add a rule:
   ```json
   "ssh": [
     {"action": "accept", "src": ["alice@gmail.com"], "dst": ["tag:lab"], "users": ["alice"]}
   ]
   ```
   Each person may log in only as their own Linux user. Removing the rule, or the user in the admin console, revokes access.
4. **Key expiry** stays disabled for `drones` (Machines → "…"), or it logs itself out after about 180 days.

## Not getting in each other's way

Several people can be logged in at once; each SSH session is independent. What is shared, and how to stay out of each other's way:

| Shared | Rule |
|---|---|
| CPU and RAM | Run `htop` before starting anything heavy. Leave cores for others: batches use at most 16 of the 24 unless the machine is idle. Simulations with wall-clock watchdogs fail when starved. |
| **ROS 2 network** | Everyone on the PC shares ROS discovery. Use **your own `ROS_DOMAIN_ID`** (table below) in every terminal (`export ROS_DOMAIN_ID=<yours>`, or put it in your `~/.bashrc`). Otherwise your nodes see and command someone else's. |
| **Gazebo** | Same problem: `export GZ_PARTITION=<yourname>` in every terminal. |
| Docker | Name containers with your user as a prefix (`--name alice_...`). Never run `docker rm -f $(docker ps -q)` or `docker system prune`: they hit everyone's containers. |
| Processes | Never `pkill -f ros`, `killall gz` or similar. Kill only your own (`pkill -u $USER -f ...`). |
| Disk | Work in your home folder. Large data goes under `~/data`, and you delete it when done (`df -h`). |
| GPU | Check `nvidia-smi` before a big job; say something in the group chat if you need all of it. |
| Reboot / shutdown | Only after asking; it cuts everyone off. |

**ROS domain IDs** (0-101 are valid; keep away from 0, the default):

| person | ROS_DOMAIN_ID | GZ_PARTITION |
|---|---|---|
| Wesley: batch runner (one per parallel run) | 40-55 | `wesley_<n>` |
| Wesley: interactive | 30 | `wesley` |
| Tejen | 60 | `tejen` |
| (next person) | 70 | `<name>` |

Add yourself to this table (the copy at `~/REMOTE_ACCESS.md` on the PC) when you start using ROS there.

## If something is wrong

- **`ssh: Could not resolve hostname drones`**: Tailscale isn't running on your side (`tailscale status`).
- **It times out:** the PC is probably off. Someone in the lab needs to switch it on.
- **Your simulation behaves oddly or sees extra topics:** check `ROS_DOMAIN_ID` and `GZ_PARTITION`, then `ros2 node list` for nodes that aren't yours.
