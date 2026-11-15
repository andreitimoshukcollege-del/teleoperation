# robot/gpu: the GPU machine at a glance

```bash
just gpu-ui            # opens http://127.0.0.1:8765 in your browser; Ctrl+C stops it
just gpu-ui 8770       # another port
```

The page shows what runs in our account on the GPU machine (`gpu_host`; its layout is in `docs/gpu-machine.md`), what
each job is doing, and about how long it has left. Each action asks for confirmation before it runs.

| Job | Shows | Actions |
|---|---|---|
| Isaac twin | robot, renderer fps, what it is recording | stop (`just isaac-stop`) |
| LeRobot watcher | its keeper, the log's tail | stop / start (`just lerobot-watch-stop` / `-start`; stop removes the keeper too) |
| Training run | phase, steps or sessions done, time left in that phase | pause, resume, stop |
| Safety batch (on this PC) | takes done, kept, set aside, time left including the dataset build | pause or stop after the take in progress, resume |
| Safety dataset build | episodes written | (none) |
| Network datasets fetch, sim fit | the log's tail | stop |

Pausing a training run freezes its process group with SIGSTOP, and its GPU memory stays held. Stop sends SIGTERM, then
SIGCONT so that a paused run gets it too.

The batch is controlled through two files the recipe checks between takes, `recordings/.drive/PAUSE` and
`recordings/.drive/STOP`. It reports its progress in `recordings/.drive/safety-batch.json`.

## How it works

- **`dashboard.py`** runs on this PC (WSL's `python3`, standard library only). It serves `index.html` on 127.0.0.1 only,
  refuses other Host headers, and requires a per-run token on every action.
- Every 5 s it pipes **`probe.py`** over one multiplexed ssh connection. `probe.py` runs on the machine's own Python,
  reads `/proc`, the logs and `nvidia-smi`, and prints one JSON object.
- **Actions** come from a fixed whitelist and are run only for a job the latest status lists. A signal to a training
  run goes through `probe.py act`, which checks again on the machine that the process group is ours and of that kind.

## Tests

```bash
python -m pytest robot/gpu/tests     # no network: ssh and the recipes are faked
```
