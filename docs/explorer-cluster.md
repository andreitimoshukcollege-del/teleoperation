# Northeastern's Explorer cluster

Explorer is Northeastern Research Computing's (RC) Slurm cluster. We use it for the trainer (`trainer/`, ADR 0021):
its H200s run the grid of training runs that would take the RTX 5090 box days, one run per GPU. Everything else (the
twins, Isaac, LeRobot) stays on the GPU machine (`docs/gpu-machine.md`).

What follows was measured on the user's account on 2026-11-23. Limits change; the commands to re-check them are
given with each.

## Getting in

**Since 2026-11-06 Explorer accepts only SSH keys that RC generated.** Passwords are refused, and so is any key you
make yourself: a public key added to `~/.ssh/authorized_keys` there is ignored, even when connecting from inside the
cluster. RC's instructions are
[SSH Instructions for Explorer](https://rc.northeastern.edu/2026/10/23/ssh-instructions-for-explorer/).

1. RC creates a key pair in your cluster home when the account is made: `~/.ssh/<user>-<UTC stamp>` and its `.pub`.
   Sign in to [Open OnDemand](https://ood.explorer.northeastern.edu) (Northeastern single sign-on, which works
   before SSH does), open Files → Home Directory, tick **Show Dotfiles**, open `.ssh`, tick both files and Download.
2. Move them into `~/.ssh` on the machine you connect from (under WSL, WSL's own `~/.ssh`, not the Windows one) and
   lock the private key down:

   ```bash
   mv /mnt/c/Users/<you>/Downloads/<user>-<stamp>* ~/.ssh/
   chmod 600 ~/.ssh/<user>-<stamp>; chmod 644 ~/.ssh/<user>-<stamp>.pub
   ```

   Do not leave the private key in Downloads, and do not put it on a shared or lab machine (use Open OnDemand there).
3. Add a host entry to `~/.ssh/config`. The `Control*` lines share one connection between the many short ssh calls
   the recipes make:

   ```
   Host explorer
       HostName login.explorer.northeastern.edu
       User <user>
       IdentityFile ~/.ssh/<user>-<stamp>
       IdentitiesOnly yes
       ControlMaster auto
       ControlPath ~/.ssh/cm-%r@%h-%p
       ControlPersist 8h
       ServerAliveInterval 30
   ```

4. `ssh explorer hostname` prints `explorer-01` or `explorer-02` (the two login nodes behind the name).

No VPN is needed; the login nodes are reachable from anywhere. If RC issues a new key, download it the same way and
change `IdentityFile`.

## What the account has

| | |
|---|---|
| Slurm account | the PI's (our only one, so no `--account` is needed) |
| Unix groups | `users` and the lab's `world-models` (the PI, the labmate and us) |
| Home | `/home/<user>`, on RC's VAST storage. Code, the trainer's images, logs. Its quota is not shown to us (`quota` fails there): ask RC before keeping many 3.9 GB images |
| Lab space | `/projects/world-models`, 11 TB, group `world-models`, new files group-writable. Results we keep |
| Scratch | `/scratch/<user>`, 20 TB and 20 M files per user. **Emptied on the first Tuesday of every month** (below) |
| Software | Rocky Linux 9.7; Apptainer 1.4.5 system-wide (no module), `--fakeroot` works on compute nodes; `module avail` has cuda (12.1-13.2), anaconda3, miniconda3, python/3.13 |
| Network | compute nodes reach the internet (PyPI answered from an H200 node) |

**Scratch is not storage.** RC deletes every eligible file on `/scratch` during the monthly maintenance window, the
first Tuesday of the month, with no warning, and it is not backed up
([scratch policy](https://rc.northeastern.edu/research-computing-policies/scratch-space-policy/),
[RC's scratch guide](https://rc.northeastern.edu/2026/10/27/getting-the-most-out-of-scratch-on-explorer/)). We put
only what a job regenerates there (the prepared training data). Runs go to the lab space.

The lab space also holds the lab's own conda install and notes (`/projects/world-models/README.md`,
`batch_job_example.sh`, the labmate's). The trainer does not use that conda: it runs from its own Apptainer image, so
nothing the lab installs can change its libraries.

## The GPUs and their limits

**Nothing is booked.** There is no calendar for the H200s: a job goes into Slurm's queue and starts when a GPU of the
kind it asked for is free, the order set by priority (fair share: recent heavy use lowers it). Ours started at once
on 2026-11-23. The limits below are what bound us, not a reservation. A reservation for a fixed time is something
only RC can make, on request; the trainer does not need one.

```bash
sinfo -o "%.18P %.5a %.11l %.6D %.25G"                     # partitions, time limits, GPUs per node
scontrol show partition gpu                                 # one partition's limits
sacctmgr -nP show qos format=name,maxtrespu,maxjobspu,maxsubmitpu,maxwall
```

| Partition | Longest job | Per user | What it has |
|---|---|---|---|
| `gpu` | 8 h | 4 GPUs, 4 jobs running, 8 submitted | 4 nodes of 8 H200 (141 GB each); A100, V100, T4 |
| `gpu-short` | 2 h | 2 GPUs, 2 jobs | the same nodes |
| `gpu-interactive` | 2 h | 2 jobs | the same nodes, for `srun --pty` |
| `sharing` | 1 h | 2 GPUs | everything else: H100, L40S, A6000, A100, ... |
| `short` | 2 days | 1024 CPUs | CPU nodes (the default partition) |
| `multigpu` | | 8 GPUs | not open to us: RC's access form first |

Ask for an H200 with `--gres=gpu:h200:1` (a bare `--gres=gpu:1` gets any GPU in the partition, often a V100). The
H200 nodes' driver is **570.86 (CUDA 12.8)**, so the trainer's image carries torch 2.11.0+cu128, not the cu130 build
the 5090 takes. A job that waits says why in `squeue`'s last column: `Priority` (others first), `Resources` (no free
GPU of that type), `QOSMaxGRESPerUser` (our 4-GPU cap).

An interactive shell on one H200, for poking at things (2 h at most; type `exit` to give it back):

```bash
ssh explorer
srun -p gpu-interactive --gres=gpu:h200:1 -n 1 -c 8 --mem=32G -t 1:00:00 --pty bash
```

Pass `-n 1`: without it `srun` started our command twice. A bare `srun --pty /bin/bash`, as the login banner
suggests, gets a CPU node on `short` and no GPU.

**The login nodes are for editing and submitting.** The banner asks for no heavy work there, and RC kills jobs run
on them. The recipes keep to that: building, testing, preparing and training are all Slurm jobs. What they do on a
login node is submit and read (`sbatch`, `squeue`, `sacct`, `tail` of logs), unpack a pack (8 MB), count a grid's
combinations (`apptainer exec ... grid --list`: 0.9 s of CPU and 40 MB, measured), and, in `explorer-fetch`, a `tar`
of the run. A fetch of a whole 30-run grid is the heaviest of these (perhaps a minute of CPU); if it grows past that,
move the tar into a job or copy through the transfer node.

## The trainer there

The `just explorer-*` recipes do all of it from the repo machine, over the `explorer` ssh name (`EXPLORER_HOST`
overrides it). The sbatch files in `trainer/slurm/` stay cluster-neutral; the recipes pass Explorer's partitions and
GPU type on the sbatch command line, which overrides their `#SBATCH` lines.

```bash
just explorer-push                  # pack this commit's trainer, copy it there, unpack: ~/armpred/current
just explorer-build                 # build the image (a CPU job), then selftest + smoke run on one H200
just explorer-status                # the queue, what is current, the runs kept, the newest logs
just explorer-train sim-pretrain    # prepare (CPU) then the grid (configs/grid.toml), 4 H200s at a time
just explorer-fetch sim-pretrain-<sha>/runs    # bring the runs back (no checkpoints, no data)
```

Measured on 2026-11-23 (commit 7dbf7c6): the build job takes 5 minutes and makes a 3.9 GB image; the selftest (109
tests) and smoke run take 10 minutes on one H200, 6 of them the smoke's evaluation; `explorer-fetch` of the smoke run
brings back 71 MB. Neither job waited in the queue.

Commit before `explorer-push`: the pack records the commit, every model card carries it, and a pack of uncommitted
work is marked `-dirty` (and replaced on the next push, image and all). Configs and grids must be committed too: the
cluster only has what the pack holds.

What goes where:

```
~/armpred/                                   home
├── packs/armpred-<sha12>.tar.gz             what explorer-push sent
├── armpred-<sha12>/armpred/                 the unpacked trainer, and armpred.sif once built
├── current -> armpred-<sha12>/armpred       the latest push; jobs pin the pack they were submitted from
└── logs/armpred-<job>-<id>.out              every job's output
/projects/world-models/<user>/armpred/       runs we keep (EXPLORER_RUNS)
├── smoke-<sha12>/                           explorer-build's smoke run
└── <config>-<sha12>/runs/<combination>/     explorer-train's runs
/scratch/<user>/armpred/<config>-<sha12>/data   the prepared data (regenerable; RC empties /scratch monthly)
/scratch/<user>/.apptainer-cache             the image build's layer cache
```

`explorer-build` takes `EXPLORER_TORCH` (`cu128`; `cu130` once the driver reaches CUDA 13) and the jobs ask for
`EXPLORER_GPU` (`h200`). `explorer-train <config> [grid] [name] [at_once]` refuses a name already used, and
`at_once` stays at 4 unless RC raises the per-user cap. A grid task gets 6 h (`train-array.sbatch`), under the `gpu`
partition's 8 h.

Each push leaves its pack in `~/armpred/<pack>` and, once built, a 3.9 GB image in it. Nothing removes old ones;
delete a pack folder by hand when it is no longer the one jobs or results refer to (`readlink ~/armpred/current` is
the newest; a run's `model-card.json` names the commit that made it).

By hand, from the unpacked folder (`cd ~/armpred/current` on a login node), the same jobs are:

```bash
sbatch -p short slurm/build.sbatch cu128
sbatch -p gpu-short --gres=gpu:h200:1 slurm/selftest.sbatch -          # selftest only
sbatch -p short slurm/prepare.sbatch configs/sim-pretrain.toml /scratch/$USER/armpred/x/data
sbatch -p gpu --gres=gpu:h200:1 --array=0-29%4 slurm/train-array.sbatch \
  configs/sim-pretrain.toml configs/grid.toml /scratch/$USER/armpred/x/data /projects/world-models/$USER/armpred/x/runs
```

`squeue -u $USER` lists our jobs, `scancel <id>` stops one, `sacct -j <id>` says how one ended.

## Moving files

The recipes stream files through ssh (`ssh explorer 'cat > f' < f`, `tar czf - | ssh`), which needs nothing else. RC
names a transfer node for large copies in its SSH instructions (`xfer.discovery.neu.edu`, for both clusters); use
it with the same key for anything big, e.g.
`rsync -av -e "ssh -i ~/.ssh/<user>-<stamp>" bundle.tar <user>@xfer.discovery.neu.edu:/projects/world-models/<user>/`.
Robot recordings and data bundles go to `<user>/teleop-data` (next section); the user asked for it on 2026-11-25.

## Data from the GPU machine

Since 2026-11-25 the recorded data lives on the cluster, not on the GPU machine. The user's request: the 5090's disk is
needed for the twins, and the lab space has 11 TB.

- **Where:** `/projects/world-models/<user>/teleop-data/`, laid out as the GPU machine's `~/data` is: `takes/`,
  `sessions/`, `lerobot/`, `network/`, `real-cameras/` (`calibration/`, `depth/`, `scenes/`). Files are group
  `world-models`, readable and writable by the group and closed to others. The labmate (cheng.qian, in that group)
  reads them there.
- **How it gets there.** The GPU machine sends straight to RC's transfer node with the RC key. `just gpu-cluster-key`
  put a copy of the key in the GPU account's `~/.ssh` (mode 600), with ssh hosts `explorer` and `explorer-xfer`.
  `scripts/gpu/cluster_archive.sh` does the copying, using rsync. It adds and updates on the cluster and never deletes
  there.
- **Recipes:**
  - `just cluster-archive [path under ~/data ...]` copies and checks, and waits until done.
  - `just cluster-archive-start` does the same detached, for the big first copy.
  - `just cluster-archive-schedule` sends new data every night at 03:30.
  - `just cluster-data-status` shows sizes on both sides and the newest log.
  - `just cluster-prune <days> [true]` frees the GPU machine (below).
- **What stores there by itself:**
  - safety batches (`isaac-safety-batch`, `isaac-safety-batch-twins`) archive their takes and datasets when they end;
  - `just camera-calib-record` and `just femto-record` store their recordings from the operator PC (`_cluster-store`,
    through the transfer node with the PC's key);
  - the nightly entry catches everything else: the headset's takes, the LeRobot converter's datasets and sessions.
- **The GPU machine keeps a working set.** Isaac and the LeRobot converter still write to its local `~/data`, because
  dataset builds and the twins need local files. `just cluster-prune 14` lists the take and session folders untouched for
  14 days whose cluster copy is identical (checksums), and `true` deletes exactly those. The user asked to see the list
  before the first deletion. Datasets are never pruned by it: a superseded dataset is deleted by hand, by its path, once
  the user agrees. A folder holding another account's file is never deleted.
- **Only our own data.** The archive reads only the GPU account's `~/data` and writes only `<user>/teleop-data`. Nothing
  in the labmate's home on the GPU machine, nor anything of other lab members in `/projects/world-models`, is touched
  (the user, 2026-11-25).

## Changing the recipes

What tripped the first version, so it does not again:

- **`$USER` in a path sent over ssh.** A recipe's ssh command is a double-quoted string, so `$USER` in it expands on
  this machine (the laptop's user), not on the cluster (the Northeastern username). `EXPLORER_RUNS` keeps a literal `$USER`, sent
  `printf %q`-quoted and replaced on the far side (`${RUNS//\$USER/$USER}`).
- **Apptainer sees only home, `/tmp` and the working directory.** Anything written under `/scratch` or `/projects` from
  inside the image needs `--bind <folder>` (the sbatch files bind each output's parent, at the same path), or it fails
  with "Read-only file system".
- **Explorer's settings live on the sbatch command line** (`--partition`, `--gres=gpu:h200:1`), not in
  `trainer/slurm/`, whose `#SBATCH` lines stay generic for any cluster. Command-line flags override `#SBATCH` lines.
- **Dependent jobs use `--kill-on-invalid-dep=yes`**, so a failed build or prepare cancels what waits on it instead
  of leaving it pending forever.
- **Jobs run from the pack folder itself** (`readlink -f ~/armpred/current`), so a later push does not change what an
  already-submitted job runs.

## When something fails

| Symptom | Cause |
|---|---|
| `Permission denied (publickey)` | Not RC's key: the `IdentityFile` must be the downloaded `<user>-<stamp>` (Getting in, step 1) |
| A job waits with `(Dependency)` | It waits for the job before it (build before selftest, prepare before the grid) |
| A job vanished after the one before it failed | `--kill-on-invalid-dep`: the recipes cancel what can no longer run. Read the first job's log |
| `selftest: FAILED` with "no kernels that run on sm_90" | The image's torch build does not suit the GPU; rebuild with the right `EXPLORER_TORCH` |
| `OSError: Read-only file system: '/scratch'` (or `/projects`) | Apptainer mounts only home, `/tmp` and the working directory; the sbatch scripts bind their output folders (`--bind`), so a hand-written `apptainer exec` must too |
| Files gone from `/scratch` on a Wednesday | The monthly purge. Prepared data is regenerated by re-running the prepare |

Help: rchelp@northeastern.edu, RC's office hours ([getting help](https://rc.northeastern.edu/support/gettinghelp/)),
and the docs at [rc-docs.northeastern.edu](https://rc-docs.northeastern.edu/en/latest/).
