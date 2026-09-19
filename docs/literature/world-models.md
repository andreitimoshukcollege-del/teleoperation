# World models and learned dynamics for robotic manipulation

Survey run 2026-09-15, branch `literature-review`, HEAD `aafa443`; full-text cost pass
2026-09-16 on the same branch. Prose only: nothing here was
built, run or measured in this repo. Every number below is a number **someone else reported on
someone else's hardware**, and per `docs/literature/CLAUDE.md` it is a candidate generator, never
evidence about this system.

## Why this file exists

`aafa443` ("Document the world-model direction, at vision level only") put a paragraph in root
`CLAUDE.md` and a section in `README.md` saying the intended next step is prediction that
understands the *physical situation* rather than only the trajectory — a model that knows the arm
will stop at the wall, or that a mass in the gripper changed how it accelerates. That commit
deliberately added no ADR, no contract and no code, and it says plainly that nothing in this
system observes the environment today. This file is the neighbourhood map for that paragraph: what
the outside world has actually built, what it reports, and — the question the direction paragraph
does not ask and this file is organised around — **what it costs per step of inference.**

## The frame that decides everything: an 11.1 ms budget

This project's frame path is 90 Hz. Eleven point one milliseconds per frame for *everything* —
predictor, reconciler, playout buffer, and whatever else is wired into `Pipeline/`. Core is
allocation-free, zero-NuGet, netstandard2.1, and any model would have to arrive behind
`Contracts/IInferenceBackend.cs`, which today has no implementations and no consumers.

So the organising question of this review is not "does the model predict well". It is: **what does
one rollout step cost, on what hardware, and is that number reported at all?** The short answer,
stated up front because it is the finding a reader most needs: *most of this literature does not
report it.* Model-based RL papers report scores per environment step and training wall-clock;
video-world-model papers report FID/PSNR and sometimes an interactive frame rate; manipulation
world-model papers report task success rate. Per-step inference latency with named hardware
appears in a minority of sources, and where it appears it is usually because the authors were
selling efficiency or interactivity (LaWAM, Efficient-WAM, GE-Act, GameNGen, DIAMOND, Oasis,
RynnWorld-Teleop, DSWAM) or because the paper's whole subject was the latency (AHEAD, Khalil & Kwon,
Embodied.cpp). Any table cell below that says **not reported** is a fact about the literature, not a
gap in this review.

The 2026-09-16 full-text pass sharpened that minority without overturning it. Sixteen of the 46
systems in the table now report an inference cost with named hardware, against nine after the
abstract-level pass — the additions being AVDC, Navigation World Models, MRO-GWM, Efficient-WAM,
DSWAM, and Cosmos's tokenizer. Six more state a latency with **no hardware named at all**, which is
its own category and is tracked as such. The rest genuinely do not measure it: for roughly twenty
systems, including the entire Dreamer lineage, the full text contains only training wall-clock.

## How to read the tables

The real-time-feasibility column ("Real-time statement made by the source", and the cost column
beside it) is a **restatement of the source's own reported numbers, with the hardware named**. It is not a judgement about whether the model would run here. Where a source
reports nothing, the cell says `not reported` — an estimate written here would be read later as a
measurement, which is precisely the failure `docs/literature/CLAUDE.md` exists to prevent.

**Second pass, 2026-09-16 — full texts, and a third cell value.** The first pass read ~30 of the
rows at abstract level and said so; it also predicted that reading their full texts would fill
about a third of the `not reported` cells, because all seven full texts it did fetch contained a
timing number the abstract omitted. That prediction was tested. Full texts were fetched for 39
abstract-only entries, and the table now distinguishes three things rather than two:

- **a number** — stated in the full text *with its hardware named*, quoted here;
- **a number with the hardware unnamed** — recorded, and explicitly flagged as unplaceable, because
  a latency without a device cannot be compared to any other row;
- **`not reported (full text checked)`** — the paper does not contain the number. This is the new
  value and it is the more useful half of the pass: it tells a reader the figure does not exist,
  rather than that nobody looked.

Nothing was converted, interpolated or inferred. Where a source gives FPS rather than a per-frame
latency, the FPS is what appears; where a source gives both, both appear. Where a source labels its
own figure an estimate (Navigation World Models does), it is recorded as the authors' estimate and
not as a measurement.

---

## Family 1 — latent-state recurrent dynamics (the RSSM lineage)

The founding family and still the one with the most complete story. An encoder compresses an
observation into a low-dimensional latent, a recurrent transition model rolls that latent forward
conditioned on actions, and decoder heads reconstruct observation/reward. Because the rollout
happens in a ~200-1000-dimensional latent rather than in pixels, imagination is *cheap relative to
the encoder* — which is the architectural fact most relevant to an 11.1 ms budget, and one this
literature states qualitatively far more often than it measures.

What these papers optimise for is **sample efficiency** (how few environment steps to learn a
task), not inference latency. Their reported wall-clock numbers are almost always *training* time.
The one place a real latency constraint shows through is DayDreamer, where the control rates on
physical robots are published — and they are low.

> **Ha & Schmidhuber, "World Models"** (NeurIPS 2018).
> <https://arxiv.org/abs/1803.10122> (accessed 2026-09-15).
> *Proposes:* the VAE + MDN-RNN + tiny linear controller decomposition that named the field; train
> the controller entirely inside the model's "dream" and transfer back to the real environment.
> *Claims:* a "very compact and simple policy" suffices once features come from the world model,
> and a policy trained wholly in the hallucinated rollout transfers.
> *Conditions:* abstract page only was fetched; the abs page states no parameter counts, no
> hardware and no per-step timing. Environments are the two named in the paper's public record
> (CarRacing and VizDoom), both low-resolution 2D video-game domains, not robots.
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/1803.10122>. It does
> report sizes: CarRacing VAE **4,348,547** parameters, MDN-RNN **422,368**, controller **867**;
> VizDoom 4,446,915 / 1,678,785 / 1,088. Latent is 32-d with a 256-unit LSTM (64-d and 512 units
> for VizDoom). Training took "less than an hour of computation time on a single GPU" and **the
> GPU is not named**. **No per-step inference timing appears anywhere in the full text.**

> **Hafner, Lillicrap, Fischer, Villegas, Ha, Lee & Davidson, "Learning Latent Dynamics for
> Planning from Pixels"** (PlaNet, ICML 2019). <https://arxiv.org/abs/1811.04551> (accessed
> 2026-09-15).
> *Proposes:* the Recurrent State-Space Model (RSSM) — a transition model with both a deterministic
> and a stochastic path — plus "latent overshooting"; plan online with CEM in latent space.
> *Claims:* matches or beats model-free algorithms on continuous control from pixels while using
> "substantially fewer episodes".
> *Conditions:* abs page only. Tasks are DeepMind Control Suite from 64x64 pixels. **No planning
> horizon, control rate, model size or hardware is stated on the abs page.** Note the structural
> cost: PlaNet plans by sampling many latent rollouts *per control step*, so its per-decision cost
> is (population x horizon) model evaluations, not one.
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/1811.04551>, which
> supplies exactly the numbers the structural note above needs: planning horizon **H = 12**, **J =
> 1000** candidate samples, **I = 10** optimisation iterations, **K = 100** elites, action repeat
> 2-8 depending on domain, and 30-dimensional diagonal-Gaussian latents. Training is "10 to 20
> hours (depending on the task) on a single Nvidia V100 GPU". **Per-step planning time is still
> not stated**, so the wall-clock cost of 1000 x 10 latent rollouts per control step is unmeasured
> in the paper itself.

> **Hafner, Lillicrap, Ba & Norouzi, "Dream to Control: Learning Behaviors by Latent Imagination"**
> (Dreamer, ICLR 2020). <https://arxiv.org/abs/1912.01603> (accessed 2026-09-15).
> *Proposes:* replace PlaNet's online CEM search with an actor-critic trained on imagined latent
> trajectories, backpropagating analytic value gradients through the learned dynamics. At control
> time only the encoder plus a feed-forward actor run — the expensive rollout moves to training.
> *Claims:* beats prior methods on data efficiency, computation time and final performance on 20
> visual control tasks.
> *Conditions:* abs page only; no imagination horizon, model size or GPU on the abs page. Tasks are
> again DeepMind Control Suite from pixels.
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/1912.01603>. Imagination
> horizon is **H = 15** (H = 10 for Atari and DeepMind Lab); latents are 30-dimensional. The only
> wall-clock figure is *training*: "about 33 hours per 10^6 environment steps on the control
> suite", on a single **Nvidia V100** GPU with 10 CPU cores. **Parameter counts and inference
> latency are not stated.** So the abstract's "computation time" claim is, in the full text, a
> training-time claim.

> **Hafner, Lillicrap, Norouzi & Ba, "Mastering Atari with Discrete World Models"** (DreamerV2,
> ICLR 2021). <https://arxiv.org/abs/2010.02193> (accessed 2026-09-15).
> *Proposes:* categorical (discrete) latents in place of Gaussian ones, plus KL balancing.
> *Claims:* first agent to reach human-level on the 55-game Atari benchmark by learning behaviours
> inside a separately trained world model; outperforms comparable **single-GPU** baselines at 200M
> frames.
> *Conditions:* abs page only. "Single GPU" is a training-budget statement, not an inference one;
> the GPU is not named on the abs page. Domain is Atari plus humanoid locomotion from pixels.
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/2010.02193>. Sizes are
> given: world model **20M** trainable parameters, actor and critic **1M** each. Imagination
> horizon **H = 15**. The single-GPU claim resolves to *training*: "200M environment steps in
> under 10 days, while using only a single **NVIDIA V100** GPU and a single environment instance".
> **Inference latency is not stated.**

> **Hafner, Pasukonis, Ba & Lillicrap, "Mastering Diverse Domains through World Models"**
> (DreamerV3). <https://arxiv.org/abs/2301.04104> (accessed 2026-09-15).
> *Proposes:* symlog prediction, normalisation/balancing/transformation tricks that make one
> hyperparameter set work across domains; explicitly studies scaling model size.
> *Claims:* strong results on over 150 tasks with fixed hyperparameters; first to collect diamonds
> in Minecraft from scratch without human data or curricula.
> *Conditions:* abs page only. **No parameter counts, GPU or training time on the abs page**, which
> is notable given that model-size scaling is one of the paper's selling points. Nothing on the abs
> page addresses inference latency.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2301.04104v2>. The scaling study
> spans **6 model sizes from 12M to 400M parameters**; the prediction/imagination horizon is **T =
> 16**; "All Dreamer agents are trained on a single **Nvidia A100** GPU each", and the Minecraft
> result used "1 GPU for 9 days". **Inference or policy-evaluation latency is not stated anywhere
> in the full text** — all wall-clock in this paper is training.

> **Wu, Escontrela, Hafner, Goldberg & Abbeel, "DayDreamer: World Models for Physical Robot
> Learning"** (CoRL 2022). <https://arxiv.org/abs/2206.14176> (accessed 2026-09-15); full text read
> via <https://ar5iv.labs.arxiv.org/html/2206.14176>.
> *Proposes:* run Dreamer online, on four physical robots, with no simulator and no resets; decouple
> a learner thread (continuously training the world model) from an actor thread (computing actions)
> "to meet latency requirements".
> *Claims:* a quadruped learns to stand and walk from scratch in 1 hour and adapts to pushes within
> 10 minutes; two arms learn pick-and-place from camera images and sparse rewards, "approaching
> human performance"; same hyperparameters throughout.
> *Conditions:* **this is the most transferable source in the family, because it publishes control
> rates on real hardware.** From the full text: the A1 quadruped's "motors are controlled at 20 Hz
> via continuous actions that represent motor angles"; the UR5 arm is controlled "at 2 Hz"; the
> XArm "at approximately 0.5 Hz"; the Sphero "at 2 Hz". Batch sizes of ~16K on a single GPU are
> mentioned for the learner, but **the GPU is not named and no inference latency is measured.**
> Prior work therefore reports a learned-world-model manipulator loop running at **2 Hz and 0.5 Hz**
> — three orders of magnitude below this project's 90 Hz frame path — under conditions where the
> world model is *also training online on the same machine*, which is not the deployment posture a
> pure predictor would take.

> **Hansen, Su & Wang, "TD-MPC2: Scalable, Robust World Models for Continuous Control"** (ICLR
> 2024). <https://arxiv.org/abs/2310.16828> (accessed 2026-09-15).
> *Proposes:* a decoder-free latent world model trained with a TD objective, used for short-horizon
> MPC (sample trajectories in latent space at every control step) with a learned terminal value.
> *Claims:* improves over baselines across 104 online RL tasks in 4 domains with a single
> hyperparameter set; a single **317M-parameter** agent performs 80 tasks across embodiments and
> action spaces.
> *Conditions:* abs page only. Domains are simulated (DMControl, Meta-World, ManiSkill2, MyoSuite).
> **No planning horizon, GPU or inference latency on the abs page.** As with PlaNet, the planning
> formulation means per-control-step cost scales with sample population x horizon, so the 317M
> figure understates per-decision compute.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2310.16828v2>. The size ladder is
> **1M / 5M / 19M / 48M / 317M**. The planning configuration, which is what actually decides per-
> decision cost, is stated: horizon **H = 3**, population **512**, **64** elites, **6** CEM
> iterations (+2 when the action dimension is >= 20). **No absolute planning time and no hardware
> is given**; the only speed statement is that code-level optimisation improved planning
> throughput "approx. 2x", which is a ratio with no baseline number attached. Control rate is not
> stated.

> **Zhang, Wang, Sun, Yuan & Huang, "STORM: Efficient Stochastic Transformer based World Models for
> Reinforcement Learning"** (NeurIPS 2023). <https://arxiv.org/abs/2310.09615> (accessed
> 2026-09-15).
> *Proposes:* replace the RSSM's GRU with a stochastic Transformer sequence model over VAE latents.
> *Claims:* 126.7% mean human performance on Atari 100k; training an agent on 1.85 hours of
> real-time interaction takes 4.3 hours on a **single NVIDIA GeForce RTX 3090**.
> *Conditions:* abstract read in full. The 4.3 h / RTX 3090 figure is *training* wall-clock on
> Atari 100k at 64x64, not inference. No per-step inference latency is given. Included here because
> it is one of the few sources in the family that names its hardware at all.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2310.09615v1>. The RTX 3090 figure
> is confirmed as training. Additionally: imagination horizon **L = 16**, 64x64 input, a 2-layer
> 512-dimensional 8-head Transformer. **Total parameter count is not stated, and no per-step
> inference latency or imagination throughput figure exists in the full text.**

> **Okada & Taniguchi, "Dreaming: Model-based Reinforcement Learning by Latent Imagination without
> Reconstruction"** (ICRA 2021). <https://arxiv.org/abs/2007.14535> (accessed 2026-09-15).
> *Proposes:* remove Dreamer's generative decoder entirely, replacing the reconstruction ELBO with a
> likelihood-free InfoMax contrastive objective, plus independent linear dynamics and random-crop
> augmentation.
> *Claims:* fixes "object vanishing" — small task-relevant objects being reconstructed away — and
> beats Dreamer and model-free baselines on 5 difficult simulated robotics tasks.
> *Conditions:* abs page only; **no speed or compute numbers on the abs page.** Recorded because it
> is the earliest clean statement of the thesis that families 4 and 6 keep rediscovering: *the
> decoder is the expensive part and control does not need it.* It also carries a warning in the
> opposite direction — the object-vanishing failure it fixes is exactly what a *predictive display*
> could not tolerate, since a human is looking at the reconstruction.
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/2007.14535>: it contains
> **no wall-clock measurement, no hardware, no parameter count, and no quantitative speed
> comparison against Dreamer at all** — the decoder-removal argument is made on task scores only.
> Overshooting distance K = 3. This is the purest instance in the review of an efficiency thesis
> with no efficiency measurement.

---

## Family 2 — action-conditioned transformer / token dynamics

The RSSM's recurrent core replaced by a causal Transformer over discrete or continuous tokens.
Architecturally this is the family whose cost is easiest to reason about — it is an autoregressive
decode, so per-step cost scales with tokens-per-frame x context length — and, unhelpfully, also the
family that most consistently reports *training* efficiency rather than inference latency.
(STORM, above, belongs here architecturally; it is filed under family 1 because it is a
drop-in replacement inside the Dreamer training loop.)

> **Micheli, Alonso & Fleuret, "Transformers are Sample-Efficient World Models"** (IRIS, ICLR
> 2023). <https://arxiv.org/abs/2209.00588> (accessed 2026-09-15).
> *Proposes:* discrete autoencoder tokenises frames; an autoregressive Transformer models the token
> sequence conditioned on actions; the agent is trained entirely inside the token rollout.
> *Claims:* mean human-normalised score 1.046 on Atari 100k, beating humans on 10 of 26 games,
> state of the art without lookahead search.
> *Conditions:* abs page only. Atari 100k = "two hours of gameplay" equivalent. **No parameter
> count, GPU or inference latency on the abs page.** Structural note relevant here: IRIS decodes
> many tokens per frame, so one imagined frame is many Transformer forward passes, not one.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2209.00588v2>. Tokens per frame **K
> = 16** (64 in an appendix variant for visually complex games), imagination horizon **H = 20**,
> 64x64 frames. Training used **8 Nvidia A100 40GB GPUs**, "around 7 days" for two Atari
> environments sharing a GPU, i.e. ~3.5 days per environment. **Parameter count and inference
> latency are not stated.** The structural note stands but remains unmeasured: 16 tokens per
> imagined frame is 16 autoregressive steps, and nobody timed one.

> **Micheli, Alonso & Fleuret, "Efficient World Models with Context-Aware Tokenization"**
> (Delta-IRIS, ICML 2024). <https://arxiv.org/abs/2406.19320> (accessed 2026-09-15).
> *Proposes:* encode the stochastic *delta* between consecutive frames rather than each frame
> whole, collapsing tokens-per-step; continuous tokens into an autoregressive Transformer.
> *Claims:* state of the art on Crafter while being "an order of magnitude faster to train than
> previous attention-based approaches".
> *Conditions:* abs page only; the "order of magnitude" is **training** speed, and no token count,
> GPU or inference latency appears on the abs page. Domain is Crafter (2D, procedural), not
> manipulation. Worth having because the *mechanism* — spend tokens only on what changed — is the
> most obviously transferable efficiency idea in the family, and a pose-stream predictor is the
> extreme case of "almost nothing changed".
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2406.19320v1>, and the check
> matters because the headline is easy to misread. Delta-IRIS is **25M** parameters against IRIS's
> 48M/50M, uses **4 tokens per frame** against IRIS's 16 or 64, imagination horizon **H = 15**,
> 64x64. Experiments ran on "a **Nvidia A100 40GB** GPU". Table 1 reports **20 FPS** for Delta-
> IRIS against **2 FPS** for IRIS-64 — but its own caption defines FPS as "the total number of
> environment frames collected divided by the training duration", so **this is training
> throughput, not rollout speed**. **Inference latency is not stated.** The 10x is a training
> number in the full text exactly as it was in the abstract.

> **Wu, Yin, Feng, He, Li, Hao & Long, "iVideoGPT: Interactive VideoGPTs are Scalable World
> Models"** (NeurIPS 2024). <https://arxiv.org/abs/2405.15223> (accessed 2026-09-15).
> *Proposes:* one token sequence over observations, actions and rewards, with "compressive
> tokenization" that spends many tokens on the context frame and few on each subsequent frame;
> pretrained on millions of human and robot trajectories.
> *Claims:* supports action-conditioned video prediction, visual planning and model-based RL from a
> single pretrained backbone.
> *Conditions:* abs page only. **No parameter counts and no rollout-speed benchmarks with hardware
> on the abs page.**
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2405.15223v2>. Sizes: transformer
> **138M** (12 layers, 768-d) and **436M** (24 layers, 1024-d); tokenizer **114M** at 64x64 and
> **310M** at 256x256. The compressive tokenization is quantified: **256 tokens** for a context
> frame against **16 tokens** per future frame, "an asymptotic 16x reduction in token sequence
> length". Rollouts are 10-15 frames. **No inference speed, latency or FPS is reported anywhere in
> the full text**, which is a notable gap for a model named "Interactive".

> **Bruce et al. (DeepMind), "Genie: Generative Interactive Environments"** (ICML 2024 best paper).
> <https://arxiv.org/abs/2402.15391> (accessed 2026-09-15).
> *Proposes:* a spatiotemporal video tokenizer + autoregressive dynamics model + **latent action
> model** learned without any action labels, trained on unlabelled internet video; the user acts
> frame by frame in the generated world.
> *Claims:* at **11B parameters**, a "foundation world model"; the learned latent action space
> transfers to imitating behaviours from unseen video.
> *Conditions:* abs page only. **No frame rate, latency or compute is stated on the abs page** — a
> notable omission for a system whose selling point is frame-by-frame interactivity. Domain is 2D
> platformer video plus a robotics ablation, not a manipulator.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2402.15391v1>, and it turns the
> abstract's silence into an explicit negative: **"Genie currently operates around 1FPS and
> requires future advances to achieve an efficient frame rate for interaction."** **No hardware is
> named for that figure.** The 11B breaks down as a **10.1B** dynamics model, a **300M** latent
> action model and a **200M** video tokenizer (stated as 10.7B combined). Training video is 160x90
> with a larger decoder to 360p for the website; sequence length 16 frames at 10 FPS, and the
> model is "still limited to 16 frames of memory". Tokens per frame are not stated. So the field's
> best-known interactive world model reports roughly **one frame per second** — worth holding next
> to Genie 3's 24 FPS claim two years later.

---

## Family 3 — video / pixel-space generative world models and interactive neural simulators

Predict the next *image*, conditioned on action. This is where the field's headline results are
now, and — because "playable" forces the question — it is also the only family that routinely
publishes a frame rate with a named accelerator. Those numbers are the most useful thing in this
review, because they bound the family from above: even the systems engineered specifically for
real-time interactivity land at **20-47 ms per frame on a datacentre GPU or TPU**, for a 360p-720p
frame, generating the *whole scene*. Nothing in this family reports anything near a 90 Hz budget,
and nothing reports running on a mobile SoC.

> **Valevski, Leviathan, Arar & Fruchter, "Diffusion Models Are Real-Time Game Engines"**
> (GameNGen, ICLR 2025). <https://arxiv.org/abs/2408.14837> (accessed 2026-09-15).
> *Proposes:* a diffusion model conditioned on past frames and actions as the entire game engine;
> conditioning augmentation for autoregressive stability; decoder fine-tuning for detail.
> *Claims:* **"runs at 20 frames per second on a single TPU"** (50 ms/frame), stable over
> multi-minute sessions; next-frame PSNR 29.4, "comparable to lossy JPEG"; human raters near chance
> at distinguishing 1.6 s clips from the real game.
> *Conditions:* abstract read in full. One game (DOOM), trained on recordings of an RL agent
> playing it; the TPU generation is not specified in the abstract. The 20 FPS is for the *whole
> rendered frame*, which is both far more than a pose predictor needs and far less controllable.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2408.14837v2>, which names the TPU
> the abstract left generic and decomposes the 50 ms. The hardware is **"a single TPU-v5"**
> (training used 128 TPU-v5e devices). Inference uses **4 DDIM sampling steps**; "a single
> denoiser step and an evaluation of the auto-encoder both takes 10ms", giving a "total U-Net cost
> of 40ms (and total inference cost of 50ms, including the auto encoder)". Frames are 320x240
> padded to 320x256, on Stable Diffusion v1.4; **parameter count is not stated**. This is the most
> completely specified real-time figure in the review: named accelerator, named step count, and a
> per-component breakdown.

> **Alonso, Jelley, Micheli, Kanervisto, Storkey, Pearce & Fleuret, "Diffusion for World Modeling:
> Visual Details Matter in Atari"** (DIAMOND, NeurIPS 2024 spotlight).
> <https://arxiv.org/abs/2405.12399> (accessed 2026-09-15); numbers below from the project page
> <https://diamond-wm.github.io/> (accessed 2026-09-15).
> *Proposes:* diffusion in pixel space as the world model, arguing discrete latents discard visual
> detail that matters for control.
> *Claims:* 1.46 mean human-normalised score on Atari 100k; a playable CS:GO neural engine.
> *Conditions:* the abs page reports no timing. The project page does: the Atari world model uses
> **3 denoising steps** and a **4.4M-parameter** dynamics model on 100k frames; the CS:GO model is
> **381M parameters** (including a 51M upsampler), trained for **12 days on an RTX 4090** on 87 h of
> human gameplay, and is "playable at **~10 FPS on an RTX 3090**" (100 ms/frame). The 3-step
> denoising figure is the single most transferable efficiency datum here — diffusion world models do
> not necessarily need 50 steps.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2405.12399v1>. The paper's own
> Atari numbers: **3 denoising steps** confirmed ("we therefore set n=3 in all of our
> experiments"), imagination horizon **H = 15**, 64x64 observations, and Table 4 reports **13M**
> parameters. Note this is *not* the same figure as the project page's 4.4M dynamics model, and
> the two sources are not reconciled here — they are recorded separately rather than merged.
> Training was "approximately 2.9 days on a single **Nvidia RTX 4090**" using ~12 GB of VRAM.
> **Inference/play FPS is not stated in the paper**; the ~10 FPS on an RTX 3090 remains a project-
> page figure and applies to the CS:GO model.

> **Decart & Etched, "Oasis: A Universe in a Transformer"**.
> <https://oasis-model.github.io/> (accessed 2026-09-15).
> *Proposes:* autoregressive, frame-by-frame diffusion-transformer generation of an interactive
> Minecraft-like world from keyboard/mouse input, with an inference stack built for peak GPU
> utilisation.
> *Claims:* "real-time output in **20 frames per second**" (~40 ms/frame on the project page's own
> arithmetic); an open 500M-parameter checkpoint, with a larger undisclosed checkpoint behind the
> live demo; forward-looking claims about an ASIC (Etched's Sohu) enabling 4K.
> *Conditions:* **this is a company publication, not a peer-reviewed paper, and the project page
> does not name the GPU**; secondary coverage states H100 and 360p/20 FPS and a 47 ms/frame figure,
> which is consistent with but not confirmed by the primary page. Treat the 20 FPS as the only
> primary-source number. No robotics content whatsoever.

> **Google DeepMind, "Genie 3: A new frontier for world models"** (blog, Aug 2025).
> <https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/> (accessed 2026-09-15).
> *Proposes:* a general-purpose, text-promptable, real-time interactive world model with emergent
> object permanence and promptable world events.
> *Claims:* "navigate in real time at **24 frames per second** ... at a resolution of **720p**",
> consistent "for a few minutes", with visual memory reaching back about a minute.
> *Conditions:* **blog post, no paper, no hardware disclosed, no model size disclosed, no
> interaction-latency figure.** 24 FPS is a throughput claim on unnamed infrastructure; for a
> teleoperation argument, throughput without the end-to-end latency it implies is not usable.
> Recorded because it is the ceiling everyone will point at, and because the ceiling is 24 FPS.

> **NVIDIA (Agarwal et al., 77 authors), "Cosmos World Foundation Model Platform for Physical AI"**.
> <https://arxiv.org/abs/2501.03575> (accessed 2026-09-15).
> *Proposes:* pretrained, openly licensed video "world foundation models" plus tokenizers and a
> curation pipeline, intended to be post-trained into application-specific world models.
> *Claims:* general-purpose world foundation models fine-tunable for downstream Physical AI.
> *Conditions:* abs page only; **no model sizes, latency or throughput on the abs page.** This is
> the substrate several manipulation systems below are built on, which matters: their latency is
> largely inherited from it.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2501.03575v1>. Released sizes are
> **7B and 14B** diffusion and **4B / 5B / 12B / 13B** autoregressive WFMs, up to 1280x704 at 121
> frames. **No latency or throughput is reported for the world foundation models themselves**, and
> no sampling-step count. The *tokenizer* is timed, and is the one hardware-named number in the
> paper: Cosmos-0.1-Tokenizer-CV4x8x8 at **34.8 ms per frame** for 720x1280 video and **62.7 ms**
> per 1024x1024 image, on an **A100 80GB**. That is the tokenizer alone — the substrate under the
> substrate — and it already exceeds this project's 11.1 ms frame three times over.

> **Zhu, Wu, Guo, Liu, Cheang & Kong, "IRASim: A Fine-Grained World Model for Robot Manipulation"**
> (ICCV 2025). <https://arxiv.org/abs/2406.14540> (accessed 2026-09-15).
> *Proposes:* a diffusion transformer with a *frame-level action-conditioning module inside every
> block*, to force action-frame alignment rather than hoping for it; an interactively controllable
> virtual arm.
> *Claims:* better video quality than baselines; simulated policy evaluation correlates with real
> evaluation; Push-T IoU 0.637 -> 0.961.
> *Conditions:* abs page only. **No model size, resolution, rollout length, or inference speed on
> the abs page.** Trajectory-conditioned video for robot arms is exactly the shape a teleoperation
> predictive display would want, and the timing is unreported.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2406.14540v2>. Model sizes span
> **33M to 679M** in the scaling study; rollouts are 15 frames (RT-1, Bridge, Language-Table) or
> 10 (RoboNet), with long trajectories "more than 150 frames"; resolutions are 256x320, 288x512
> and 256x256. **No latency, FPS or sampling-step count is reported** — but the paper states the
> conclusion in words: "a limitation of IRASim is video generation is not real-time". A negative
> real-time statement with no number attached is still more than the abstract gave.

> **Du, Yang, Dai, Dai, Nachum, Tenenbaum, Schuurmans & Abbeel, "Learning Universal Policies via
> Text-Guided Video Generation"** (UniPi, NeurIPS 2023). <https://arxiv.org/abs/2302.00111>
> (accessed 2026-09-15).
> *Proposes:* treat decision-making as text-conditioned video generation — synthesise the video of
> the plan, then extract actions from it by inverse dynamics.
> *Claims:* combinatorial generalisation to novel goals; transfer across manipulation tasks via a
> shared image space.
> *Conditions:* abs page only. **No inference time, sampling-step count, or compute details on the
> abs page** — for a method that generates a whole video per plan, this is the number that decides
> feasibility and it is absent.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2302.00111v2>. The cascade is
> **1.7B** base plus **1.7B / 1.4B / 1.2B** temporal super-resolution models; plans are 10-20
> frames in simulation and 16-32 frames for real-world, at resolutions from 48x64 up to 320x192.
> **Inference/video-generation time is not stated and neither is the sampling-step count**, so the
> number that decides whether plan-by-video is affordable is absent from the full text as well as
> the abstract.

> **Zhou, Du, Chen, Li, Yeung & Gan, "RoboDreamer: Learning Compositional World Models for Robot
> Imagination"** (ICML 2024). <https://arxiv.org/abs/2404.12377> (accessed 2026-09-15).
> *Proposes:* factorise the instruction into primitives and compose separately-conditioned video
> models, to generalise to unseen instruction combinations.
> *Claims:* synthesises video plans for unseen goals on RT-X; outperforms monolithic video
> generation; executes in simulation.
> *Conditions:* abs page only. **No inference cost, sampling parameters or quantitative benchmark
> numbers on the abs page.**
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2404.12377v1>. Plans are **8
> frames** generated at 64x64 and upsampled to 128x128 and 256x256. **No inference time, no
> parameter count, no sampling-step count and no hardware anywhere in the full text.**

> **Ko, Mao, Du, Sun & Tenenbaum, "Learning to Act from Actionless Videos through Dense
> Correspondences"** (AVDC, ICLR 2024). <https://arxiv.org/abs/2310.08576> (accessed 2026-09-15).
> *Proposes:* synthesise a video of the robot doing the task, then recover actions in closed form
> from dense frame-to-frame correspondences — no action labels anywhere.
> *Claims:* deployable across tasks from RGB video alone; tabletop manipulation and navigation.
> *Conditions:* abs page only; robots and benchmarks are not named on the abs page and **no
> inference timing is given.**
> **Full text checked 2026-09-16** via <https://ar5iv.labs.arxiv.org/html/2310.08576>, and it is
> one of the larger fills in this pass. The paper names video synthesis as its bottleneck:
> "Synthesizing a video of predicted execution is the most time-consuming step of our method,
> which takes roughly **10.57 seconds (1.51 seconds per video frame on average)**", on an **RTX
> 3080Ti** GPU. Inference uses **100 denoising steps**, which the paper says DDIM can reduce to 10
> for a stated ~10x speedup. **T = 8** frames per plan. Sizes: **201M** (Meta-World), **109M**
> (iTHOR), **166M** (Bridge). Resolutions 128x128 / 64x64 / 48x64. Benchmarks are Meta-World
> (simulated Sawyer, 11 tasks), iTHOR, Visual Pusher and Bridge (WidowX 250), plus a real Franka
> Emika Panda. So the actionless-video route costs ~1.5 s per generated frame on a consumer GPU at
> 128x128 — the same order as the off-the-shelf video models in the predictive-display benchmark
> below, on a smaller frame.

> **Bar, Zhou, Tran, Darrell & LeCun, "Navigation World Models"** (CVPR 2025 oral).
> <https://arxiv.org/abs/2412.03572> (accessed 2026-09-15).
> *Proposes:* a Conditional Diffusion Transformer predicting future egocentric observations from
> past observations plus navigation actions; plan by simulating and scoring trajectories, and
> incorporate constraints dynamically at planning time.
> *Claims:* scales to **1B parameters** on mixed human and robot egocentric video; plans from
> scratch or by ranking an external policy's samples; imagines trajectories in unseen environments
> from one image.
> *Conditions:* abs page only. **No inference or planning time and no hardware on the abs page.**
> Navigation, not manipulation.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2412.03572v2>, which does report
> runtime, in a table captioned "Runtime (seconds) on an **NVIDIA RTX 6000 Ada** card". The NWM
> row reads **30.3 +/- 0.2** seconds for the baseline, then 14.7 +/- 0.1 and 0.4 +/- 0.1 for
> successive optimisations, in the context of ranking 32 four-second trajectories; the
> supplementary describes it as the runtime of simulating a trajectory, and the exact unit of work
> behind the number is not stated more precisely than that here. **The final 0.1 is labelled
> "(est.)" by the paper itself** — a projection from the 4-bit-quantisation literature, not a
> measurement, and it is recorded here as the authors' estimate rather than as a result. Other
> stated settings: CDiT-XL at **1B** parameters, **250** denoising steps reduced to **6** by
> distillation, 2-second horizons (8 steps at 0.25 s, tested out to 16 s), 16-120 sampled
> trajectories depending on regime, 224x224 predictions.

---

## Family 4 — world models coupled to manipulation policies (the "WAM" / VLA-plus-world-model line)

This is where the field has moved since roughly 2024, and where the inference-cost question is
finally being asked out loud — because these systems have to run on a robot. The pattern is
consistent: a large pretrained video model supplies the dynamics prior, a small decoder turns
predicted futures into actions, and the paper's efficiency contribution is *avoiding the pixel
decode*. Read as a group, the family has independently converged on the same conclusion an 11.1 ms
budget forces: **predicting pixels is the expensive part, and the prediction that helps control can
live entirely in latent space.**

> **Huang, Zhang, Zou, Liu, Hu & Xu, "LaDi-WM: A Latent Diffusion-based World Model for Predictive
> Manipulation"** (CoRL 2025). <https://arxiv.org/abs/2505.11528> (accessed 2026-09-15). (A CoRL
> OpenReview forum page for this paper exists but could not be verified — see retrieval failures.)
> *Proposes:* run the diffusion process in the latent space of pretrained visual foundation models
> (DINO geometric features + CLIP semantic features) rather than in pixels; a diffusion policy then
> iteratively refines its actions using the forecast latent states.
> *Claims:* +27.9% policy performance on LIBERO-LONG and +20% in a real-world scenario; latent
> prediction is easier to learn and generalises better than pixel prediction.
> *Conditions:* abs page read; the abs page reports **no inference latency, no diffusion step count
> and no parameter count**. Benchmarks are LIBERO (simulated tabletop) and an unspecified real
> setup. The improvements are *policy success rates*, not prediction error, so they do not transfer
> to a predictive-display framing without re-measurement.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2505.11528v2> (v1 checked too), and
> the CoRL proceedings entry now verifies at <https://proceedings.mlr.press/v305/huang25a.html>
> (PMLR v305, pp. 1726-1743, accessed 2026-09-16) — the alternate route that replaces the
> unreachable OpenReview page. **Inference latency and parameter counts are not reported in the
> full text.** What is: raw resolution **128x128** with 14x14 patches, 4 historical frames, 6
> imagined future frames as the best ablation setting, and a policy that "can achieve a convergent
> performance with only **2 denoising steps**". Training used **an NVIDIA 4090** — about three
> days for the world model and six hours for the policy. Two denoising steps is the transferable
> datum; the cost of one of them is not published.

> **Chen, Wang, Chen, Chen, Gao, Tang, Li, Liu, Yao, Li, Xu & Yu, "LaWAM: Latent World Action
> Models for Efficient Dynamics-Aware Robot Policies"**. <https://arxiv.org/abs/2606.15768>
> (accessed 2026-09-15); full text via <https://arxiv.org/html/2606.15768v1>.
> *Proposes:* expose predictive dynamics to the policy as compact **latent visual subgoals** instead
> of reconstructed future video; a latent action model trained inside a frozen vision-foundation-
> model latent space.
> *Claims:* 98.6% LIBERO, 91.22% RoboTwin; **187 ms per action-chunk prediction on an A100**, up to
> **24x lower wall-clock latency than pixel-space WAMs**.
> *Conditions:* the latency claim is well-specified for once — measured over "1,000 repeated
> action-chunk predictions" on an **A100** (training on H100s). Total system 2.3B parameters, of
> which the world model (LaWM) is ~230M, "about 95% fewer world-modeling parameters than the 5B WAN
> backbone". Pixel-space baselines it reports for contrast: **LingBot-VA 4482 ms, Motus 3231 ms,
> Cosmos-Policy 1413 ms**, with the non-world-model VLA pi-0.5 at **220 ms**. The action chunk
> covers a fixed physical interval of **1.2 s** for robot teleoperation data and 0.4 s for human
> video. So: a chunk covering 1.2 s of motion costs 187 ms of A100 time — a ~6.4x real-time factor
> *per chunk*, not per frame, and only because the chunk is long.

> **Li, Guo, Ye, Zhang, Chi, Sun, Li, Lou, Huang, Lu, Guo & Zhang, "Efficient-WAM: A 1B-Parameter
> World-Action Model with Low-Cost Future Imagination"**. <https://arxiv.org/abs/2606.10040>
> (accessed 2026-09-15).
> *Proposes:* keep the imagination but make it cheap — structured world-knowledge transfer into a
> small model, **low-resolution future latents**, token-sparse video latents, and *asymmetric*
> video-action denoising that spends few sampling steps on video and more on actions.
> *Claims:* per-chunk latency "around **100 ms** during physical deployment"; a **30x** speedup over
> existing WAMs; control performance retained despite "visibly coarse future predictions".
> *Conditions:* abs page only; **the hardware behind the 100 ms is not named on the abs page**, nor
> the chunk length, so the 100 ms is not directly comparable to LaWAM's 187 ms/A100. Benchmarks are
> RoboTwin 2.0 and real-world manipulation. The finding worth carrying — that predictions can be
> *visibly wrong in appearance* and still carry the control benefit — is the family's strongest
> argument that fidelity and usefulness decouple. Note that for a **predictive display**, where a
> human looks at the prediction, that decoupling runs the other way.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2606.10040v1>, and this is the
> single most consequential fill in this pass, because the abstract's unattributed "around 100 ms"
> now has hardware under it. In simulation on an **A800**, per-chunk latency goes **2013 ms (full
> WAN) -> 430 ms (Efficient-WAM) -> 139 ms (+ asymmetric denoising)**. In real-world deployment on
> an **RTX 4090**, Efficient-WAM-RT reports **98 ms per chunk**, called a **32x** speedup over
> Motus, with **"Avg. Lat. per Step 6.1 ms"** on an Astribot S1. The chunk horizon is **H = 16**,
> so the paper reports both the per-chunk and the per-step amortisation of the same measurement
> rather than only one. Denoising is **[2, 10]** video/action steps; the observation is
> **384x320** and the predicted future is **192x160** — the low-resolution future the abstract
> described. Control rate is not stated. Against LaWAM's 187 ms/A100 per 1.2 s chunk, this is now
> a like-for-like-ish comparison for the first time: both are per chunk, both name a GPU, and the
> two GPUs differ.

> **Liao, Zhou, Huang, Yang, Chen, Jiang, Hu, Cai, Liu, Luo, Chen, Yan, Yao & Ren, "Genie
> Envisioner: A Unified World Foundation Platform for Robotic Manipulation"**.
> <https://arxiv.org/abs/2508.05635> (accessed 2026-09-15); timing numbers from
> <https://arxiv.org/html/2508.05635v3>.
> *Proposes:* one video-diffusion backbone (GE-Base) serving three roles — policy (GE-Act, a
> lightweight flow-matching decoder), neural simulator (GE-Sim) and benchmark (EWMBench).
> *Claims:* from the full text, GE-Act "achieves low-latency end-to-end control by generating
> **54-step torque trajectories within 200 ms on a commodity GPU**", and specifically "the forward
> pass for 54 steps is completed in **200 ms on an onboard NVIDIA RTX 4090 GPU**".
> *Conditions:* the action decoder is **160M parameters**; the video backbone runs at **5 Hz** while
> the action stream runs at **30 Hz**, a 1:6 temporal ratio. Pretraining used 32 A100s. This is the
> clearest published example of the architectural move that matters most for a 90 Hz budget: **run
> the expensive world model at a low rate and a cheap head at the control rate.** 54 steps at 30 Hz
> is 1.8 s of trajectory for 200 ms of RTX 4090, i.e. the cost is amortised over a long chunk, and
> the per-control-step figure is only meaningful because the chunk is open-loop.

> **Kim, Gao, Lin, Lin, Ge, Lam, Liang, Song, Liu, Finn & Gu, "Cosmos Policy: Fine-Tuning Video
> Models for Visuomotor Control and Planning"**. <https://arxiv.org/abs/2601.16163> (accessed
> 2026-09-15).
> *Proposes:* post-train a large pretrained video model (Cosmos-Predict2) into a policy in one
> stage with no architectural changes, encoding actions *and* values as latent frames inside the
> video model's own diffusion process; plan at test time by generating future states and values.
> *Claims:* 98.5% LIBERO, 67.1% RoboCasa, best average on real bimanual tasks; improves further by
> refining its world model and value function from rollout data.
> *Conditions:* abs page only; **no model size, no inference latency, no control rate on the abs
> page.** LaWAM (above) independently reports Cosmos-Policy at **1413 ms** per action chunk, which
> is a third-party measurement on a third party's hardware and should be read as such.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2601.16163v1>. The backbone is
> **Cosmos-Predict2-2B**. The paper does give an inference figure, and it is a negative one: "We
> observe substantially lower inference speed when using model-based planning (e.g., **around 5
> seconds to produce one action chunk**)" — but **the sentence names no hardware**, and the H100
> counts that do appear (64, 32 and 8 H100s for LIBERO, RoboCasa and ALOHA) are training. Direct-
> policy latency is not stated. Chunks are 16 timesteps (LIBERO), 32 with 16 executed (RoboCasa),
> and 50 timesteps = 2 s at a **25 Hz** controller (ALOHA, reduced from 50 Hz "for computational
> efficiency"). Denoising: 5 steps for LIBERO/RoboCasa as a direct policy, 10 for ALOHA, and 10
> actions + 5 future state + 5 value when planning. Resolution is not stated. LaWAM's third-party
> 1413 ms and this paper's own ~5 s are measuring different modes and should not be read as
> contradicting each other.

> **Cen, Yu, Yuan, Jiang, Huang, Guo, Li, Song, Luo, Wang, Zhao & Chen, "WorldVLA: Towards
> Autoregressive Action World Model"**. <https://arxiv.org/abs/2506.21539> (accessed 2026-09-15).
> *Proposes:* one autoregressive model that is simultaneously a VLA and a world model — actions
> and images share a token vocabulary, so each improves the other; an attention-masking scheme that
> hides prior actions during action-chunk generation to stop error accumulation.
> *Claims:* beats separate action and world models; the mask substantially improves action-chunk
> generation. **Explicitly identifies autoregressive error accumulation across an action chunk as
> the failure mode** — relevant to anyone rolling a model forward over a delay horizon.
> *Conditions:* abs page only; **no model size, compute, or benchmark numbers on the abs page.**
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2506.21539v1>. Action chunk **K =
> 10** for LIBERO-Long and **K = 5** for the other three suites; the image tokenizer "generates
> **256 tokens** for 256x256 images and **1024 tokens** for 512x512 images". **Inference latency,
> parameter count and control rate are not stated.** For a paper whose central finding is
> autoregressive error accumulation *across* a chunk, the per-token cost that sets how long a
> chunk can be is unmeasured.

> **Zhang et al., "DreamVLA: A Vision-Language-Action Model Dreamed with Comprehensive World
> Knowledge"** (NeurIPS 2025). <https://arxiv.org/abs/2507.04447> (accessed 2026-09-15).
> *Proposes:* predict *dynamic-region-guided* world knowledge (not whole frames) with spatial and
> semantic cues; block-wise structured attention to stop the prediction objective interfering with
> the action objective; a diffusion transformer that separates action from shared latents.
> *Claims:* 76.7% real-robot success; 4.44 average length on CALVIN ABC-D.
> *Conditions:* abs page only. **No model size, latency or hardware on the abs page.** The
> "dynamic-region-guided" idea is the same efficiency instinct as Delta-IRIS: predict only what moves.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2507.04447v1>. The backbone is
> GPT-2 Medium, "a 24-layer, 16-head Transformer decoder with a hidden size of 1,024 and a total
> of approximately **345 million parameters**", with a DiT-B action head whose size is not given.
> The prediction window is small: "we set K=2, corresponding to a **3-frame prediction window**
> (current + 2 future steps)", and each image view contributes 196 latent vectors plus a CLS
> token. **Inference latency, hardware and control rate are not stated.**

> **Luo, Zhang, Feng, Zheng, Xu, Xu, Xi, Fu & Lu, "Being-H0.7: A Latent World-Action Model from
> Egocentric Videos"**. <https://arxiv.org/abs/2605.00078> (accessed 2026-09-15).
> *Proposes:* the strongest version of the "skip the pixels" argument — learnable latent queries
> between perception and action, trained with a *training-only* posterior branch that sees future
> observations, so at deployment the prior branch reasons future-aware **with no visual rollout at
> all**.
> *Claims:* state-of-the-art or comparable across six simulation benchmarks and real-world tasks,
> "combining the predictive benefits of world models with the efficiency and deployability of
> direct VLA policies"; states plainly that "pixel-space prediction is a costly and indirect
> substrate for control ... introduces substantial training or inference overhead".
> *Conditions:* abs page only; **no model size, latency or hardware on the abs page**, which is
> unfortunate for a paper whose thesis is efficiency. The *mechanism* — future-conditioned training,
> future-free inference — is the cheapest form of "world model" in this review and is the one that
> would survive a hard per-frame budget.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2605.00078v1>, and it produces the
> lowest per-step figure in this entire review — with a caveat that has to travel with it. The
> paper states that "Being-H0.7 variants operate in the **3-4 ms/step** regime without adding the
> burden of test-time future generation", enabled by what it calls **Universal Asynchronous
> Chunking** (UAC), an asynchronous real-time chunking mechanism that absorbs timing variation
> without rewriting committed action prefixes. **No GPU, SoC or device is named for that figure
> anywhere in the paper**, so it cannot be placed against any other row in the table. Model size
> is **3B** (Table 1); pretraining action chunk length **T = 20**; deployed control rates are **20
> Hz** (PND Adam-U), **10 Hz** (Unitree G1) and **20 Hz** (Franka FR3). Read carefully, 3-4
> ms/step with a 10-20 Hz control rate means the model is not the binding constraint on those
> platforms — which is the strongest published support for the decoder-free thesis, and it is
> reported without hardware.

> **Zhu, Yan, Hong, Shou, Ma & Guo, "WMPO: World Model-based Policy Optimization for
> Vision-Language-Action Models"**. <https://arxiv.org/abs/2511.09515> (accessed 2026-09-15).
> *Proposes:* on-policy RL for a VLA entirely inside a pixel-based world model whose predictions are
> aligned with the VLA's pretrained features; GRPO on imagined rollouts.
> *Claims:* better sample efficiency than off-policy alternatives; emergent self-correction;
> lifelong learning in sim and on a real robot.
> *Conditions:* abs page only; no model size, rollout cost or hardware. Included to mark the
> boundary: this is world-model-**as-training-environment**, where inference cost is irrelevant
> because nothing runs at control time. A large share of this literature is this, and it is the
> share that transfers least to a latency problem.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2511.09515v1>: **no inference
> latency, no parameter count, no resolution and no control rate.** The world model is "based on a
> video diffusion backbone inherited from OpenSora", uses 4 conditioning frames and generates the
> next **K = 8** frames with a matching action-chunk length, and training used 32 H100 GPUs.
> Consistent with its role: nothing runs at control time, so nothing was timed.

> **Zhu, Zhang, Su, Liu, Wang, Xie, Huang, Ma, He, Wang, Wang, Ding & Xu, "DSWAM: A Dual-System
> World Action Foundation Model for Fine-Grained Robot Manipulation"**.
> <https://arxiv.org/abs/2607.04927> (accessed 2026-09-15).
> *Proposes:* a fast System-1 world-action executor as the default control path with an optionally
> activated slow System-2 vision-language subtask planner; for real deployment, TensorRT
> acceleration, asynchronous execution and "real-time chunking to prevent control blocking".
> *Claims:* matched-condition comparison against VLA baselines on real robots.
> *Conditions:* abs page only; **no latency numbers, model sizes or control rates on the abs page**,
> despite deployment engineering being an explicit contribution. Recorded mainly for the pattern:
> *dual-rate architecture plus asynchronous execution so the slow model never blocks the control
> loop*, which is the same shape as GE-Act's 5 Hz/30 Hz split and as this project's own separation
> of a playout path from whatever computes the sample.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2607.04927v1>, and the deployment
> engineering the abstract only gestured at is measured in Table 4 with hardware named: on an
> **NVIDIA GeForce RTX 5090** (CUDA 12.9, TensorRT 10.16.1), "BF16 TensorRT reduces warmed end-to-
> end policy latency from **198.2 ms** in PyTorch to **73.8 ms** ... giving a **2.69x** speedup".
> System 2 "observes a short-term visual history ... containing the most recent T=5 frames sampled
> at **1 Hz**". **Parameter counts, the action horizon H, and the robot's control rate are all
> still not stated.** The 2.69x is the correct order of magnitude to expect from runtime
> engineering and matches Embodied.cpp's 1.05-2.70x range from an independent codebase.

> **Assran et al. (Meta, 30 authors), "V-JEPA 2: Self-Supervised Video Models Enable Understanding,
> Prediction and Planning"**. <https://arxiv.org/abs/2506.09985> (accessed 2026-09-15); timing from
> <https://arxiv.org/html/2506.09985v1>.
> *Proposes:* a self-supervised video JEPA encoder pretrained on >1M hours of internet video, then a
> small action-conditioned predictor (V-JEPA 2-AC) post-trained on <62 h of unlabelled Franka video;
> plan zero-shot by cross-entropy-method search in representation space, with no reward and no task
> demonstrations.
> *Claims:* zero-shot pick-and-place on Franka arms in unseen labs; 77.3% SSv2, 39.7 R@5 on
> Epic-Kitchens-100 anticipation.
> *Conditions:* **the most quotable cost number in this entire review, and it is a bad one**: the
> full text states "the V-JEPA 2-AC world model requires only **16 seconds per action**" on a
> "single **NVIDIA RTX 4090 GPU**", using receding-horizon control that executes one action then
> replans. Encoder is a ViT-g at ~1B parameters; the predictor is "~300M parameters"; inputs are
> 256x256 at 4 fps. Sixteen seconds per action, for a pick-and-place — from a 2025 frontier lab, on
> a current consumer GPU. That is the honest state of *planning-by-world-model* on real hardware,
> and it is roughly **1,440x** the per-frame budget of this project's control path. (The cost is
> dominated by CEM sampling, not by one forward pass; a single-rollout predictor is a different and
> much cheaper proposition — which is exactly the distinction families 4 and 6 keep blurring.)

> **Syed, Jakobsson, Hao & Ichnowski, "Intercepting the Future: Latent-Space Predictive World Model
> for Dynamic VLA Manipulation"** (AHEAD). <https://arxiv.org/abs/2606.02486> (accessed
> 2026-09-15); timing from <https://arxiv.org/html/2606.02486v1>.
> *Proposes:* **explicitly a latency-compensation wrapper.** A frozen VLA assumes the scene is
> static between observation and execution; AHEAD adds a small motion-aware latent world model that
> forecasts future patch tokens in the VLA's own feature space, conditioned on per-token velocity
> and acceleration from optical flow, with a saliency mask restricting prediction to task-relevant
> patches and an **adaptive horizon that halts when prediction uncertainty crosses a threshold**.
> The frozen action decoder then consumes predicted tokens in place of current ones.
> *Claims:* +4.9M parameters on a frozen 7B OpenVLA; 79-97% success across 20 dynamic simulation
> scenarios where the best baseline reaches 31-58%; on a physical UFactory xArm 7, 29-30/30 on
> conveyor and rolling-ball tasks, 23/30 paddle interception, 19/30 projectile catching where
> "every baseline scores 0/30".
> *Conditions:* the full text gives a component breakdown on an **NVIDIA H100 80GB at 224x224**:
> **~20 ms** RAFT optical flow, **~40 ms** world model with 5 samples, **~70 ms** VLA forward pass,
> **~158 ms** end-to-end per action step, against a stated **~200 ms** budget. Realised adaptive
> horizon "averages 3 to 5 steps" with K_max = 10. Conveyor speeds swept 0-40 cm/s in sim and
> 0-25 cm/s on the physical arm. Two things are worth carrying: the **world-model rollout itself is
> only ~40 ms of the 158 ms** (the frozen VLA is the expensive part), and **uncertainty-gated
> adaptive horizon** is a directly recognisable mechanism — predict further only while the model
> says it can.

---

## Family 5 — 3D / 4D scene predictors (explicit-geometry world models)

Both 2026 surveys name this as its own representation family and it is the one most obviously
adjacent to a *VR* teleoperation platform, because the output is already a renderable scene rather
than a 2D frame. The pattern is 3D Gaussian Splatting extended with motion, conditioned on actions.
The relevant asymmetry — and the reason this family deserves attention despite being the least
mature — is that 3DGS **rendering** is famously cheap while 3DGS **dynamics prediction** here is a
latent diffusion transformer, i.e. the expensive half is the same expensive half as everywhere
else. None of the three sources below reports an inference time.

> **Lu, Jia, Li, Chen, Wang, Tang & Huang, "GWM: Towards Scalable Gaussian World Models for
> Robotic Manipulation"** (ICCV 2025). <https://arxiv.org/abs/2508.17600> (accessed 2026-09-15).
> *Proposes:* predict the future by inferring how **Gaussian primitives propagate** under robot
> actions; a latent Diffusion Transformer over a 3D VAE gives scene-level future reconstruction in
> Gaussian Splatting form. Usable both as a representation-learning objective for imitation and as
> a neural simulator for model-based RL.
> *Claims:* precise action-conditioned future scene prediction; policies trained on it beat the
> state of the art "by impressive margins".
> *Conditions:* abs page only. **No inference or rendering speed, no model size, no diffusion step
> count, no hardware on the abs page.** The ICCV Open Access PDF was attempted and **returned HTTP
> 403** (see retrieval failures).
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2508.17600v1> — the arXiv HTML
> rendering, which is the route that worked after the CVF Open Access PDF and its HTML landing
> page both returned 403 (see retrieval failures). It adds structure but no timing: Table A1
> states a model-based-RL "Horizon: 10" with **"Prediction horizon per inference: 1"**, and the
> VAE downsamples the reconstructed scene to **N = 2048** Gaussians with **M = 512** latent
> points. **No inference or rendering speed, no parameter count, no denoising-step count and no
> GPU is named anywhere in the full text.** For the family whose selling point is that rendering
> is cheap, that absence is the finding.

> **Chai, Deng, Shao, Zhang, Lv, Xing, Li, Zhang & Liu, "GAF: Gaussian Action Field as a 4D
> Representation for Dynamic World Modeling in Robotic Manipulation"**.
> <https://arxiv.org/abs/2506.14135> (accessed 2026-09-15).
> *Proposes:* extend 3DGS with learnable motion attributes so one representation yields current
> scene reconstruction, future frame prediction, and an initial action estimate from Gaussian
> motion, refined by action-vision-aligned denoising.
> *Claims:* +11.5385 dB PSNR, +0.3864 SSIM, -0.5574 LPIPS on reconstruction and +7.3% manipulation
> success over prior methods.
> *Conditions:* abs page only. **No inference speed, hardware or model size on the abs page.** The
> reconstruction deltas are large enough to be worth noting as a caution: a +11.5 dB PSNR
> improvement usually means the baseline was doing something structurally different, and the abs
> page does not say what the baseline was.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2506.14135v1>. It claims "real-time
> execution on a single GPU during manipulation" **without any number and without naming the
> GPU**, which is exactly the shape of claim this file exists to flag. Inference uses **3
> diffusion iterations** (training uses 50 DDIM steps), evaluation is at 128x128, and the
> representation holds **131,072** Gaussian points. **Parameter count, control rate, and any
> latency or FPS figure are all absent.**

> **Kreber, Mack & Stueckler, "Learning Action-Conditional and Object-Centric Gaussian Splatting
> World Models for Rigid Objects"** (MRO-GWM). <https://arxiv.org/abs/2606.01950> (accessed
> 2026-09-15).
> *Proposes:* the most structured member of the family — represent the scene as **per-object**
> Gaussians in canonical frames, so dynamics reduce to predicting a rigid transform per object; a
> spatio-temporal transformer predicts future rigid-body motion from Gaussian history plus future
> actions; handles occlusion-induced partial observation.
> *Claims:* works on synthetic household-object scenes with robot interaction, and in MPC for
> non-prehensile manipulation in simulation.
> *Conditions:* abs page only; **no speed, hardware, model size or benchmark numbers on the abs
> page**, and the evaluation is synthetic. Structurally this is the closest thing in the review to
> "predict a pose per rigid body", which is the representation this project's transport already
> carries — the difference being that MRO-GWM predicts *object* transforms from contact, which is
> precisely the capability the direction paragraph asks for and the wire format has no field for.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2606.01950v1>, and it is the only
> member of family 5 that times anything: **"For inference, one batch of 30 items takes 0.65 s on
> average to compute on the validation set"**, on an **Nvidia A40** GPU. Default prediction
> horizon is **p = 4** (8 and 12 also evaluated). Rates are stated throughout: the simulator is
> driven at **20 Hz** while physics runs at 100 Hz, model variants operate at **5 Hz** and **10
> Hz**, and **planning runs at 2 Hz** (with prediction horizon 8 at 5 Hz). Representation uses a 1
> cm anchor resolution with 5 Gaussians per anchor. **Parameter count is not stated.** Note what
> the numbers say: the most structurally efficient representation in the review still plans at 2
> Hz.

---

## Family 6 — the bridge: is anyone using a world model for teleoperation delay compensation?

This is the question the rest of the review exists to answer, so the answer is stated first and
plainly.

**Almost nobody, and the exceptions do not do what the name suggests.** After the searches recorded
at the end of this file, the position is:

1. **"Teleoperation" appears constantly in world-model papers, and it almost always means *data
   collection*, not delay.** Teleoperated demonstrations are the training data; the world model
   replaces the robot so more demonstrations can be collected. RynnWorld-Teleop is the clearest
   case — its title contains "Teleoperation", its abstract is entirely about the data bottleneck,
   and it has nothing to say about network delay. Anyone searching this literature for latency work
   by keyword will drown in this, and it is worth writing down once so the next person does not
   have to rediscover it.
2. **Generative *predictive display* for teleoperation exists as a named problem, and the one
   systematic benchmark of it reports that off-the-shelf video world models fail the real-time
   test outright.** That negative result (Khalil & Kwon, below) is the most directly relevant
   external finding in this entire review.
3. **Delay-compensated video for teleoperation that actually runs is built from geometry, not from
   world models** — depth estimation, point-cloud reprojection, learned inpainting — and reaches
   13 FPS (Chakraborty et al., below). The learned part is doing view synthesis, not dynamics.
4. **Latent-rollout latency compensation on real hardware exists, but it compensates the *robot's
   own* inference latency, not a network delay, and there is no human in the loop** (AHEAD, in
   family 4). Its architecture — small latent predictor wrapped around a frozen expensive model,
   uncertainty-gated horizon — is nonetheless the closest structural analogue to a predictor sitting
   in front of a display.
5. **Prediction-for-delay in teleoperation with a human in the loop is a real and active
   literature, but it predicts the *operator's command trajectory*, not the world** — LSTM/GRU
   sequence models, residual RL over delayed state, motion-primitive prediction. That is the
   dead-reckoning-with-a-bigger-hammer branch, and it is the sibling file on learned predictive
   display's territory, not this one's. The boundary is worth naming: *a world model predicts what
   the environment will do; every teleoperation delay-compensation system found here predicts what
   the human or the robot will do.* Nothing found predicts the consequences of contact for the
   operator's benefit.

So: **the specific thing the `aafa443` direction paragraph describes — a model that knows the arm
will stop at the wall, used to improve what a delayed operator sees — is not something this
literature has built.** That is an unoccupied intersection, not a solved problem, and it is a real
result. It also means there is no external number to inherit; anything in that intersection has to
be measured here.

> **Khalil & Kwon, "Towards Generative Predictive Display for Vision-Based Teleoperation: A
> Zero-Shot Benchmark of Off-the-Shelf Video Models"**. <https://arxiv.org/abs/2605.09670>
> (accessed 2026-09-15); full text via <https://arxiv.org/html/2605.09670>.
> *Proposes:* stop speculating and measure — take five publicly released video generation models,
> apply them zero-shot (no task-specific training) as predictive displays, and score prediction
> accuracy, per-rollout latency, peak GPU memory and *how the error evolves over the rollout*.
> *Claims:* the headline is a negative one — **"no tested model simultaneously achieves low rollout
> error, non-divergent per-step error behavior, and real-time inference"** at source frame rate.
> The authors conclude that deployment needs explicit short-horizon temporal supervision, in-domain
> adaptation, or aggressive inference optimisation, rather than off-the-shelf application.
> *Conditions:* full text read. Models: **LTX-Video (2B distilled and 13B), Stable Video Diffusion
> 1.1, Wan VACE 1.3B, Wan I2V 1.3B**. Domain is **CARLA simulated driving**, not manipulation, at
> **256x160 and 512x320**, two conditioning regimes (multi-frame and single-frame). Source rate
> **15 FPS**, i.e. a 66.7 ms frame budget; **88 future frames** per rollout. Measured on an **NVIDIA
> RTX 6000 Ada (48 GB)** with a Threadripper Pro 5975WX. Reported per-frame inference spans
> **1.31 +/- 0.06 s to 3.85 +/- 0.08 s**, and end-to-end rollout latency **10.48 s (SVD, 256x160) to
> 30.80 s (LTX-13B, 512x320)**; peak GPU memory **3.15 GB to 46.68 GB**. Read against its own 66.7
> ms frame budget, the *fastest* model measured is roughly **20x too slow**, and that is on a 48 GB
> workstation GPU at a resolution well below a headset's. Prior work therefore reports that
> off-the-shelf generative video prediction is between one and two orders of magnitude away from
> real-time predictive display, under those conditions.

> **Chakraborty, Fang, Schreiber, Ji, Huang, Mihigo, Wall, Almana & Driggs-Campbell, "Towards
> Real-Time Generation of Delay-Compensated Video Feeds for Outdoor Mobile Robot Teleoperation"**
> (ICRA 2025). <https://arxiv.org/abs/2409.09921> (accessed 2026-09-15); full text via
> <https://arxiv.org/html/2409.09921v2>.
> *Proposes:* a deliberately *modular, non-generative* pipeline — monocular depth (finetuned Depth
> Anything V2), a skid-steer/Dubins kinematic model to predict the future camera pose from the
> operator's own commands, point-cloud reprojection through a Pulsar sphere renderer, and a
> ResNet-18 inpainting network to fill disocclusions.
> *Claims:* more accurate delay-compensated images than state-of-the-art alternatives in their
> setting, and one of very few such methods evaluated outdoors, in real time, on data from a real
> robot.
> *Conditions:* full text read. Delays of **250 ms and 500 ms** applied to real rosbags, plus
> horizons t+1 to t+10 at 30 Hz (33-333 ms). Component rates: depth **58 FPS**, inpainting **36
> FPS**, overall system **13 FPS**, with asynchrony noted as allowing higher effective rates in the
> ROS implementation; benchmarked on a **2080** GPU (training used two A100s). Robot is a
> TerraSentia+ in agricultural crop rows. Baselines: C++S (crop-and-scale), SRVP, DMVFN, SynSin,
> Telea inpainting. **The design point is the finding:** the system that actually runs in the field
> replaces the learned dynamics with *a kinematic model plus learned rendering*, and even then lands
> at 13 FPS. It is also structurally aligned with this repo's separation of concerns — the "what
> will the pose be" part is analytic and cheap, and the learning is spent on appearance.

> **Penco, Mouret & Ivaldi, "Prescient teleoperation of humanoid robots"**.
> <https://arxiv.org/abs/2107.01281> (accessed 2026-09-15).
> *Proposes:* have the robot execute commands *before receiving them* by continuously querying a
> model trained on past whole-body trajectories and conditioned on the last received commands, so
> the operator's visual feedback appears synchronised.
> *Claims:* an operator controlled a **32-DoF humanoid** under **stochastic delays up to 2 seconds**
> across whole-body reaching, bottle-picking and box-placing.
> *Conditions:* abs page only; **no inference timing or rate is stated on the abs page.** This is
> command-trajectory prediction, not world modelling — included as the boundary marker for point 5
> above, and because "up to 2 s of stochastic delay, with a human in the loop, on a real 32-DoF
> robot" is a more aggressive operating point than most of the world-model literature attempts at
> all.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2107.01281v1>, and the rates it
> publishes are the most teleoperation-shaped in the review. "The controller is run at a frequency
> of **100 Hz**"; delayed motion-capture data arrives at 100 Hz and is transmitted to the robot
> controller at **50 Hz**. Delays tested are "stochastic round-trip delays ranging from **200ms to
> 2s**", with an average round-trip delay of 1.5 s and a forward delay of **750 +/- 300 ms**; the
> paper also names 1 s (DARPA), 10 s (NASA) and 0.8 s (METERON) as real operating points. The
> model is built from 54 demonstrations, and "2 seconds is enough to obtain very accurate
> predictions for the next 4-5 seconds". **Inference/prediction computation time with named
> hardware is still not stated**, so the 100 Hz controller rate is a system property, not a
> measured model cost.

> **Deng & Yang, "Residual Reinforcement Learning for Robot Teleoperation under Stochastic
> Delays"**. <https://arxiv.org/abs/2605.15480> (accessed 2026-09-15).
> *Proposes:* an LSTM state estimator that reconstructs a continuous state estimate from delayed
> observations, plus a residual RL policy that learns a torque correction trading tracking accuracy
> against velocity smoothness.
> *Claims:* outperforms existing methods on a Franka Panda under high-variance stochastic delays.
> *Conditions:* abs page only. **No delay values, no control rate, no inference timing on the abs
> page** — so the "high-variance" claim has no numbers attached here. Noted because the
> accuracy-versus-smoothness objective is the same tradeoff this repo calls prediction error versus
> correction cost, arrived at independently and optimised jointly rather than reported as a pair.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2605.15480v1>, and the delay
> distributions the abstract withheld are fully specified in Table 2 of its appendix. Control
> frequency is **250 Hz (dt = 4 ms)** — the highest control rate anywhere in this review, and
> close to this project's own operating regime. Three delay conditions: low delay/low variance
> **U(120, 160) ms**, total 170-210 ms; high delay/low variance **U(200, 240) ms**, total 250-290
> ms; high delay/high variance **U(40, 240) ms**, total 90-290 ms; plus a constant 50 ms action
> delay. Networks: LSTM 256 hidden units x 3 layers, SAC actor/critic [512, 256]. **Inference
> latency and hardware are not stated.** Those delay profiles are directly comparable in shape to
> the ones this project's transport work sweeps, which makes this the most operating-point-legible
> source in the bridge section.

> **Zhao, Zhao, Li, Gong, Li, Huang, Li, Zhao & Li, "RynnWorld-Teleop: An Action-Conditioned World
> Model for Digital Teleoperation"**. <https://arxiv.org/abs/2607.06558> (accessed 2026-09-15).
> *Proposes:* "digital teleoperation" — replace the physical robot with a generative world model so
> an operator's hand-pose stream synthesises egocentric video, yielding state-action trajectories
> for imitation learning without hardware. Depth-aware skeletal conditioning, progressive
> human-to-robot training on a video DiT, and **streaming autoregressive distillation into a
> single-pass inference**.
> *Claims:* "**40+ FPS**, real-time interactive generation on a single **H100** GPU"; policies
> trained only on generated data transfer zero-shot to real bimanual tasks.
> *Conditions:* abs page read in full. **This is not delay compensation** — despite the title, the
> problem being solved is data collection cost, and there is no network delay anywhere in it. It is
> included because (a) it is the keyword trap described above, and (b) the *distillation* result is
> genuinely transferable: 40+ FPS (<25 ms/frame) on one H100 is the fastest action-conditioned
> video world model found in this review, and it got there by distilling a multi-step diffusion
> rollout into one forward pass. Whatever the ceiling on generative predictive display is, it moves
> when distillation is applied.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2607.06558v1>, which sharpens the
> abstract's "40+ FPS" into a specified measurement: "a high throughput of **40.0 fps at 480x832**
> resolution" on "a single **NVIDIA H100** GPU", with per-frame latency "approximately **25 ms**"
> decomposed as skeletal action encoding ~5%, causal DiT denoising ~72%, visual decoding ~23%. The
> distilled student uses a **4-step flow-matching schedule**; clips are 81 frames at 16 FPS (~5
> s). The base model is Wan-2.2-TI2V-5B; **RynnWorld-Teleop's own parameter count is not stated.**
> The denoiser being 72% of a 25 ms frame is the useful detail — it says where the remaining
> headroom is, and it is not in the decoder.

---

## Surveys, benchmarks and evaluation

Used as maps to primary sources, per instruction, and recorded here in their own right. Both 2026
surveys were read at abstract level only; neither's headline organisation should be taken as a
finding about this project.

> **Wang, Wang, Pei, Zhang, Liang, Hu, Li, Wu, Han, Zhang, Qi, Wu, Zhang, Zheng, Pan,
> Navarro-Alarcon, Liu & Zhou, "World Models for Robotic Manipulation: A Survey"**.
> <https://arxiv.org/abs/2606.00113> (accessed 2026-09-15).
> *Proposes:* organise the field into **five representation families** — latent dynamics models,
> action-conditioned video generators, 3D/4D scene predictors, physics-informed simulators, and
> predictive modules inside VLA systems — plus a *functional* taxonomy separating integrated
> prediction-action models from explicit predictive planners.
> *Claims:* the term "world model" now spans all five and should be disambiguated before comparison.
> *Conditions:* abstract only. The five-family split is close to the organisation used above and was
> a useful check on it; the 3D/4D scene-predictor family is the one this review under-covers, and
> that is a gap in this file rather than in the survey.

> **Hou, Li, Jia, An, Guo, Leng, Geng, Ze, Harada, Torr, Mees, Pollefeys, Liu, Wu, Abbeel, Malik,
> Du & Yang, "World Model for Robot Learning: A Comprehensive Survey"**.
> <https://arxiv.org/abs/2605.00080> (accessed 2026-09-15).
> *Proposes:* organise by *functional role* — world models coupled to policies, world models as
> learned simulators for RL, and video-based formulations evolving "from imagination-based
> generation to controllable, structured, and foundation-scale formulations" — extending to
> navigation and autonomous driving, with a section on datasets, benchmarks and evaluation
> protocols.
> *Claims:* a comprehensive map of the area as of 2026.
> *Conditions:* abstract only; the abs page lists no formal taxonomy table. Note what neither survey
> abstract mentions: **inference cost as an organising dimension.** Both organise by representation
> and by role. If a reader wants to know what a family costs per step, neither abstract offers it,
> which is consistent with the pattern this file documents.

> **Yang, Shen, Mi, Zhang, Zhou, Ji, Dai, Chen, Chen & Yang, "MiraBench: Evaluating
> Action-Conditioned Reliability in Robotic World Models"**. <https://arxiv.org/abs/2605.29360>
> (accessed 2026-09-15).
> *Proposes:* a hierarchical benchmark for whether a world model's predictions are physically
> plausible *and* actually follow the commanded action, across three levels: reference-free physics
> consistency, action-following fidelity, and detection of **optimism bias** (predicting success
> when the action should fail). Over 16,000 human judgements.
> *Claims:* three findings that matter for anyone considering a world model as a predictor:
> **visual quality does not reliably predict action fidelity**; **scaling model size does not
> consistently improve action-following**; and **optimism bias is widespread** across 12
> representative configurations.
> *Conditions:* abstract only; no latency or compute measured. The optimism-bias finding is the one
> to carry: a world model used as a predictive display would be biased toward showing the operator
> a successful grasp. That is a failure mode with no analogue in a dead-reckoner, which fails
> *symmetrically* and obviously.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2605.29360v1>. The 12
> configurations are named in its Table 2: DreamDojo (2B, 14B), DreamDojo-GR1 (2B, 14B), Cosmos-
> GR1 (2B, 14B, plus a 14B text-conditioned mode), WAN 2.1 (14B), WAN 2.2 (5B), and three closed-
> access models — Happy Horse, WanX and Kling 3.0 Omni — whose sizes are not given. **No inference
> latency, no hardware, and no evaluation resolution or rollout length is stated anywhere in the
> full text.** A reliability benchmark that measures action fidelity and optimism bias but not
> cost is consistent with the pattern this file documents, and it means the optimism-bias finding
> cannot be traded off against a latency figure from the same source.

> **Mereu, Scannell, Hou, Zhao, Jitta, Dominguez, Acerbi, Storkey & Chang, "Generative World
> Modelling for Humanoids: 1X World Model Challenge Technical Report"**.
> <https://arxiv.org/abs/2510.07092> (accessed 2026-09-15); challenge description at
> <https://www.1x.tech/discover/1x-world-model-sampling-challenge> (accessed 2026-09-15).
> *Proposes:* two tracks over a released 100-hour real humanoid video dataset — *sampling*
> (predict future frames from a sequence of prior frames, scored by **PSNR**) and *compression*
> (predict future discrete latent codes, scored by Top-500 cross-entropy). The challenge page
> states "submissions should achieve a PSNR of around 26.5 or above", names GANs, diffusion models
> and MaskGIT as expected approaches, and opened its evaluation server in March 2025. The report's
> entries: Wan-2.2 TI2V-**5B** with AdaLN-Zero conditioning and LoRA post-training for sampling; a
> from-scratch spatio-temporal transformer for compression.
> *Claims:* 23.0 dB PSNR (sampling) and Top-500 CE 6.6386 (compression), first place in both.
> *Conditions:* the technical report was read at **abstract level only** and the challenge page in
> full; **no hardware and no inference speed is reported in either.** The exact context length and
> which future frame is scored are given in neither page fetched, so they are not stated here. What
> is clear is the *shape* of the metric: an appearance score (PSNR) on generated future frames.
> That is neither an error-over-horizon curve nor a correction-cost analogue, so a model that wins
> this says nothing about what a human would experience being shown its output continuously.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2510.07092v1>, which fills the
> context-length gap the first pass explicitly left open. In the sampling track "the model must
> predict the **512x512** frame observed by the robot **22s into the future**", conditioned on
> "the first 17 frames", with 16 frames as prediction targets and performance evaluated on **the
> 77th frame**; the compression track conditions on H = 3 grids of 32x32 tokens and predicts the
> next M = 3. Training ran on "a DataCrunch instant cluster equipped with 4 nodes, each with **8x
> NVIDIA B200 GPUs**", about 36 hours. **Inference or sampling time is not reported, and parameter
> counts for the entries are not stated.** A 22-second-ahead single-frame PSNR target is about as
> far from a per-frame latency budget as a world-model benchmark can be, which is worth knowing
> before anyone treats a leaderboard position as evidence of usability.

> **Efficient Foundation Models for Real-Time Embodied AI**, CoRL 2026 workshop, Austin TX,
> 12 Nov 2026 (organisers: Yu, Alsharif, Liu, Shah, Wu, Zhang).
> <https://efficient-embodied-ai.github.io/> (accessed 2026-09-15).
> *Proposes:* a venue explicitly for action chunking, efficiency architectures, hardware-aware
> design, quantisation/pruning/distillation, and "safety and latency trade-offs, including fallback
> policies under stochastic response time", with a reference challenge targeting **Jetson Orin NX**
> under stated latency and memory budgets.
> *Claims:* the call states the problem in the field's own words — foundation models for embodied AI
> "still miss the latency, memory, and power budgets required to run on the robot itself", and
> "dexterous manipulation generally wants closed-loop rates well above what these policies deliver".
> *Conditions:* a call for papers, not a result — no measurements, and nothing here is evidence
> about anything. Recorded because it is the field stating, as of late 2026, that the gap this file
> documents is real and unclosed, which is worth more than any individual latency number for
> deciding whether to wait or to build small.

---

## Family 7 — deployment engineering: how the field actually closes the gap

Not a model family, but the most practically relevant cluster in this review, because these are the
sources whose subject *is* the latency. They are recent (2025-2026), which is itself the signal: the
field only started writing this down once world models had to run on robots.

> **Motubrain Team, "World Action Models in Real Time: An Empirical Study of Smooth Execution via
> Asynchronous Deployment"**. <https://arxiv.org/abs/2608.01880> (accessed 2026-09-15).
> *Proposes:* take the latency as given and study how to hide it. Six strategies compared —
> synchronous execution, pure asynchronous switching, post-hoc action blending, **denoising-time
> blending**, inference-time velocity guidance, and **prefix-conditioned generation** — for
> overlapping inference with execution.
> *Claims:* the framing sentence is worth quoting because it is the field naming the problem
> directly: WAMs "generate fixed-horizon action chunks through iterative denoising, creating
> substantial inference latency that can cause **pauses, stale actions, and discontinuities** during
> robotic execution." Findings: **accurate temporal alignment between observations, predictions and
> executed commands is a fundamental requirement**, and alignment errors "produce persistent
> chunk-boundary discontinuities that cannot be corrected through blending alone"; direct action
> weighting is a smooth but less accurate baseline; inference-time velocity guidance "fails to
> reliably constrain committed actions"; **prefix-conditioned generation** (learning consistent
> continuations during training) gives the best overall balance.
> *Conditions:* abstract read in full; **no latency numbers, no named hardware, no chunk length on
> the abs page**, and the robot is described only as "a 10 Hz bimanual robot". Authorship is a team
> name, not individuals. Two things make it worth the entry anyway. First, its central negative
> finding — *blending cannot repair a timestamp error* — is a statement about a class of system this
> project also is. Second, the six strategies are, recognisably, a **reconciliation axis** for
> learned predictors: chunk-boundary discontinuity is the same phenomenon a reconciler exists to
> absorb, and this source reports that smoothing it after the fact costs accuracy while building
> continuation into training does not. Prior work reports that under those conditions; nothing here
> measures it on this system.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2608.01880v1>, and the absence is
> the finding: **no inference latency in milliseconds, no named hardware, no parameter count and
> no denoising-step count appear in the full text of a paper whose entire subject is hiding
> inference latency.** What it does state: "The World Action Model predicts chunks of length **H =
> 24** frames" and "We deploy all methods on a bimanual end-effector robot controlled at **10
> Hz**", i.e. a 2.4 s chunk at the control rate. The six strategies are therefore compared on
> smoothness and accuracy outcomes rather than on time, which is a legitimate design but means
> nothing here transfers as a cost.

> **Xu, Li, Wu, Han, Li, Hua, Jiang, Cao, Li, Zhong & Wang, "Embodied.cpp: A Portable Inference
> Runtime of Embodied AI Models on Heterogeneous Robots"**. <https://arxiv.org/abs/2607.02501>
> (accessed 2026-09-15).
> *Proposes:* a **C++** inference runtime for VLA and world-action models, built around what it
> calls the runtime contract of embodied deployment: "**multi-rate execution inside closed-loop
> control, latency-first batch-1 inference on heterogeneous hardware, and extensible embodied
> interfaces beyond fixed token I/O**" — explicitly contrasted with request-response serving
> runtimes. Five layers: input adapters, sequence builders, backbone execution, head plugins,
> deployment adapters.
> *Claims:* **1.05x-2.70x** inference speedup and **7%-77%** lower VRAM versus Python baselines
> across three VLA and two WAM models, "while maintaining near-baseline success for most
> configurations".
> *Conditions:* abs page only; **no edge device, SoC or absolute latency is named on the abs page**,
> and the speedups are normalised comparisons across Python and C++ quantisation configurations, not
> absolute times. Recorded because the *shape* of its contract — batch-1, latency-first, multi-rate,
> behind one backend abstraction — is nearly a restatement of what
> `core/Teleop.Core/Contracts/IInferenceBackend.cs` already specifies, and because a 1.05-2.70x
> runtime speedup is the correct order of magnitude to expect from engineering, against the 10x-1000x
> gaps in the table below.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2607.02501v1>. Table 3 does carry
> absolute times — **HY-VLA at 735.9 ms amortized environment-step latency and 1340.3 ms server-
> side inference latency; pi0.5 at 56.85 ms and 266.6 ms** — but **the paper names no device for
> them**. Deployment targets are listed only generically ("Jetson devices, RK-based platforms, x86
> edge boxes, and workstation-class systems") with no per-device benchmark. Memory is given
> concretely in at least one place ("reduces block memory from 312.2 MiB to 88.1 MiB"). Parameter
> counts and control rates are not stated. So the runtime paper that most closely matches
> `IInferenceBackend`'s contract reports latency without hardware — the exact failure mode this
> file's table column is designed to expose, in the source best placed to avoid it.

> **Qi, Yin, Zhu, Du & Yang, "Inference-Time Enhancement of Generative Robot Policies via
> Predictive World Modeling"** (GPC, IEEE RA-L). <https://arxiv.org/abs/2502.00622> (accessed
> 2026-09-15).
> *Proposes:* Generative Predictive Control — **augment a frozen diffusion policy at deployment**
> with an action-conditioned world model trained on expert demonstrations plus random exploration,
> giving test-time adaptation without retraining the policy.
> *Claims:* consistently outperforms standard behaviour cloning across manipulation tasks in
> simulation and the real world.
> *Conditions:* abs page only; **no inference cost, hardware, model size or benchmark numbers on
> the abs page** — for a method whose entire contribution is spent at inference time, that absence
> is the pattern this file documents, in its purest form. Same structural idea as AHEAD: small
> world model bolted onto a frozen large policy.
> **Full text checked 2026-09-16** via <https://arxiv.org/html/2502.00622v2>. It states the cost
> direction in words and refuses the number: GPC "is generally slower than the baseline due to
> predicting the future using the world model", with **no latency measurement and no hardware
> named**. The sampling configuration is given and is the load-bearing part: **3 diffusion
> steps**, horizon **T = 16**, and candidate counts of up to **K = 5000** for state-based tasks,
> dropping to **K = 100** or **K = 10** for vision-based ranking "due to memory constraints". That
> collapse from 5000 to 10 candidates when vision enters is the same population-versus-cost wall
> PlaNet, TD-MPC2 and V-JEPA 2-AC all hit, stated here as an implementation constraint rather than
> as a result. Parameter count and control rate are not stated.

---

## Summary table — reported inference cost against a real-time budget

**Read the last column as a quotation, not a judgement.** It restates what the source itself
reported, with the hardware the source itself named. Where the source reports nothing, the cell
says `not reported`; no cell contains an estimate, an extrapolation, or an opinion about whether
the system would fit this project's 11.1 ms frame. Several cells that look like inference numbers
are *training* numbers and are labelled as such, because conflating the two is the single easiest
mistake to make when skimming this literature.

| System | Family | Reported size | Reported cost, with the source's own hardware | Real-time statement made by the source |
|---|---|---|---|---|
| World Models (Ha & Schmidhuber) | 1 latent-recurrent | CarRacing VAE **4,348,547** + MDN-RNN **422,368** + controller **867**; VizDoom 4.45M / 1.68M / 1,088 | inference **not reported (full text checked)**; *training* "less than an hour ... on a single GPU", **GPU unnamed** | none |
| PlaNet | 1 | not reported (full text checked) | per-step planning time **not reported (full text checked)**; the planning shape is stated — **H=12**, **1000 candidates x 10 CEM iterations** per control step, 100 elites. *Training* 10-20 h on a single **Nvidia V100** | none |
| Dreamer | 1 | not reported (full text checked) | inference **not reported (full text checked)**; imagination horizon **H=15**; *training* ~**33 h per 10^6 env steps on a single Nvidia V100** + 10 CPU cores | claims better "computation time"; the full text quantifies it only as training wall-clock |
| DreamerV2 | 1 | world model **20M**, actor **1M**, critic **1M** | inference **not reported (full text checked)**; imagination horizon **H=15**; *training* 200M env steps "in under 10 days ... a single **NVIDIA V100** GPU" | none |
| DreamerV3 | 1 | **12M-400M** across 6 scaling variants | inference **not reported (full text checked)**; prediction horizon **T=16**; *training* on "a single **Nvidia A100** GPU each", Minecraft 1 GPU for 9 days | none |
| DayDreamer | 1 | not reported (batch ~16K on "a single GPU", unnamed) | inference **not reported**; *control rates on real robots are*: **A1 quadruped 20 Hz, UR5 arm 2 Hz, XArm ~0.5 Hz, Sphero 2 Hz** | decoupled actor/learner threads adopted "to meet latency requirements"; no latency measured |
| TD-MPC2 | 1 | **1M / 5M / 19M / 48M / 317M** | per-step planning time **not reported (full text checked)**; planning is **H=3**, population **512**, **6** iterations (+2 if action dim >= 20), 64 elites; only speed claim is "approx. 2x" planning throughput from code optimisation, no absolute time | none |
| STORM | 1 (transformer core) | not reported (full text checked); 2 layers, 512-d, 8 heads | **training**: 4.3 h for 1.85 h of interaction, **NVIDIA RTX 3090**; imagination horizon **L=16**, 64x64. Inference **not reported (full text checked)** | efficiency claim is about training |
| Dreaming | 1 | not reported (full text checked) | **not reported (full text checked)** — no wall-clock, no hardware, no speed comparison against Dreamer anywhere in the paper | none |
| IRIS | 2 transformer-token | not reported (full text checked) | inference **not reported (full text checked)**; **16 tokens/frame**, imagination horizon **H=20**, 64x64; *training* ~3.5 days per Atari env on **8x Nvidia A100 40GB** | none |
| Delta-IRIS | 2 | **25M** (vs IRIS 48M/50M) | inference **not reported (full text checked)**; **4 tokens/frame**, H=15, 64x64. Table 1's **20 FPS** (vs IRIS-64's 2 FPS) on an **Nvidia A100 40GB** is defined in its own caption as env frames collected / *training* duration | efficiency claim is about training |
| iVideoGPT | 2 | transformer **138M** / **436M**; tokenizer **114M** @64x64, **310M** @256x256 | inference speed **not reported (full text checked)**; **256 tokens** per context frame vs **16** per future frame ("16x reduction"); rollouts 10-15 frames | none |
| Genie | 2 / 3 | **11B** total: dynamics **10.1B** + latent action **300M** + tokenizer **200M** (stated as 10.7B combined) | **"around 1FPS"**, **no hardware named for that figure**; 16-frame context, 160x90 training video (360p decoder for the website) | **negative in the paper's own words** — 1 FPS, "requires future advances to achieve an efficient frame rate for interaction" |
| GameNGen | 3 pixel-generative | Stable Diffusion v1.4; parameter count not reported (full text checked) | **20 FPS (50 ms/frame) on "a single TPU-v5"**; **4 DDIM steps**; breakdown "a single denoiser step and an evaluation of the auto-encoder both takes 10ms", "total U-Net cost of 40ms ... total inference cost of 50ms"; 320x240. *Training* on 128 TPU-v5e | yes — "real-time game engine", 20 FPS, multi-minute stability |
| DIAMOND | 3 | paper Table 4: **13M** (Atari). Project page: dynamics **4.4M**, CS:GO **381M** incl. 51M upsampler. Two sources, not reconciled here | Atari: **3 denoising steps**, H=15, 64x64; *training* ~2.9 days, ~12 GB VRAM, single **Nvidia RTX 4090**. Play/inference FPS **not reported in the paper**; "~10 FPS on an RTX 3090" is the project page's CS:GO figure | yes — playable at ~10 FPS (project page) |
| Oasis | 3 | **500M** open checkpoint; demo checkpoint undisclosed | **20 FPS**; **the project page names no GPU** | yes — "real-time output in 20 frames per second" |
| Genie 3 | 3 | not disclosed | **24 FPS at 720p**; **no hardware disclosed**; no latency figure | yes — "navigate in real time at 24 frames per second" |
| Cosmos WFM platform | 3 | diffusion **7B/14B**; autoregressive **4B/5B/12B/13B** | WFM inference **not reported (full text checked)**; the **tokenizer** is timed — **34.8 ms/frame** at 720x1280 and **62.7 ms** per 1024x1024 image, on an **A100 80GB** | none |
| IRASim | 3 | **33M to 679M** across scaling runs | **not reported (full text checked)**, but stated in words: "a limitation of IRASim is video generation is not real-time". Rollouts 10-15 frames (150+ for long), 256x320 / 288x512 / 256x256 | **negative in words, no number** |
| UniPi | 3 | base **1.7B** + super-resolution cascade (**1.7B / 1.4B / 1.2B**) | not reported (full text checked); sampling-step count also not stated; 8-32 frames per plan, up to 320x192 | none |
| RoboDreamer | 3 | not reported (full text checked) | not reported (full text checked); 8 frames per plan at 64x64, upsampled to 256x256 | none |
| AVDC | 3 | **201M** (Meta-World), **109M** (iTHOR), **166M** (Bridge) | **10.57 s per plan, "1.51 seconds per video frame on average", on an RTX 3080Ti**; **100 denoising steps** (paper states DDIM cuts this to 10 for ~10x); **T=8** frames; 128x128 / 64x64 / 48x64 | the paper calls video generation "the most time-consuming step of our method" |
| Navigation World Models | 3 | CDiT-XL **1B** | Table 8, "Runtime (seconds) on an **NVIDIA RTX 6000 Ada** card": **30.3 +/- 0.2 s** baseline, then 14.7 +/- 0.1 and 0.4 +/- 0.1 with successive optimisations, in the context of ranking 32 four-second trajectories; the final **0.1 is labelled "(est.)" by the paper** — a 4-bit-quantisation projection, not a measurement. **250** denoising steps (**6** distilled); 2 s horizons, 16-120 sampled trajectories; 224x224 | treats its own runtime as the problem; proposes distillation and quantisation |
| GWM | 5 3D/4D | not reported (full text checked) | **not reported (full text checked via arXiv HTML)** — no timing, no denoising-step count, no GPU. Table A1: MBRL "Horizon: 10", "**Prediction horizon per inference: 1**"; VAE downsamples to **N=2048** Gaussians, **M=512** latent points | none |
| GAF | 5 | not reported (full text checked) | **not reported (full text checked)** — claims "real-time execution on a single GPU during manipulation" **with no number and no GPU named**. **3 diffusion iterations** at inference (50 in training), 128x128, **131,072** Gaussian points | claims real-time, gives no number |
| MRO-GWM | 5 | not reported (full text checked) | **"one batch of 30 items takes 0.65 s on average"** at inference on the validation set, on an **Nvidia A40**; prediction horizon p=4 (also 8, 12); simulator driven at 20 Hz, model variants at **5 Hz** and **10 Hz**, **planning at 2 Hz** | no real-time claim; planning reported at 2 Hz |
| LaDi-WM | 4 policy-coupled | not reported (full text checked) | inference **not reported (full text checked, v1 and v2)**; policy converges at **2 denoising steps**; 4 history frames, 6 imagined frames, 128x128 with 14x14 patches; *training* ~3 days (world model) + ~6 h (policy) on **an NVIDIA 4090** | none |
| **LaWAM** | 4 | **2.3B total; world model ~230M** | **187 ms per action-chunk prediction on an A100**, averaged over 1,000 repeats; chunk spans **1.2 s** of robot motion | yes — "low-latency inference", up to **24x** lower wall-clock than pixel-space WAMs |
| Pixel-space WAMs, *as measured by LaWAM* | 4 | not reported | **LingBot-VA 4482 ms, Motus 3231 ms, Cosmos-Policy 1413 ms**; non-world-model VLA pi-0.5 **220 ms** — all per action chunk, LaWAM's protocol, A100 | these are LaWAM's numbers for other people's systems, not those systems' own claims |
| Efficient-WAM | 4 | **1B** (vs Motus 8B) | **hardware named in the full text**: per chunk **2013 ms (full WAN) -> 430 ms (Efficient-WAM) -> 139 ms (+asymmetric denoising) on an A800**; real-world **98 ms per chunk on an RTX 4090** (**32x** over Motus), "Avg. Lat. per Step **6.1 ms**" over an **H=16** chunk on an Astribot S1; **[2,10]** video/action denoising steps; 384x320 observation, 192x160 predicted future | yes — 98 ms/chunk on an RTX 4090; the abstract's "~100 ms"/30x are the full text's 98 ms/32x |
| **Genie Envisioner / GE-Act** | 4 | action decoder **160M** | **200 ms for a 54-step trajectory on an onboard NVIDIA RTX 4090**; GE-Base video model runs at **5 Hz**, action stream at **30 Hz** (1:6) | yes — "low-latency end-to-end control" |
| Cosmos Policy | 4 | **Cosmos-Predict2-2B** backbone | with planning, "**around 5 seconds to produce one action chunk**" — **the paper names no inference hardware** (64/32/8 H100s are training); direct-policy latency not stated. Chunks 16 (LIBERO) / 32 (RoboCasa) / 50 steps = 2 s at **25 Hz** (ALOHA); 5-10 denoising steps | negative for planning: "substantially lower inference speed when using model-based planning" |
| WorldVLA | 4 | not reported (full text checked) | not reported (full text checked); action chunk K=10 (LIBERO-Long) / K=5; **256 tokens per 256x256 image, 1024 per 512x512** | none |
| DreamVLA | 4 | GPT-2 Medium backbone **~345M** (DiT-B action head size not given) | **not reported (full text checked)**; prediction window is **3 frames** (K=2), 196 latent vectors per image view | none |
| Being-H0.7 | 4 | **3B** | **"3-4 ms/step"** with Universal Asynchronous Chunking — **no hardware named for that figure**; action chunk **T=20**; deployed control rates **20 Hz** (PND Adam-U), **10 Hz** (Unitree G1), **20 Hz** (Franka FR3); **performs no visual rollout at inference by construction** | yes — lowest per-step figure in this table, reported without hardware |
| WMPO | 4 | not reported (full text checked) | not reported (full text checked); K=8 action chunk, 4 conditioning frames, 8 generated frames; OpenSora backbone; *training* on 32 H100s | n/a — no control-time inference |
| DSWAM | 4 | not reported (full text checked) | **198.2 ms (PyTorch) -> 73.8 ms (TensorRT BF16) warmed end-to-end policy latency, 2.69x, on an NVIDIA GeForce RTX 5090** (CUDA 12.9, TensorRT 10.16.1); System 2 sees 5 frames at 1 Hz; action horizon H and robot control rate still not stated | yes — and the full text supplies the number the abstract withheld |
| **V-JEPA 2-AC** | 4 | encoder ~**1B** (ViT-g), predictor ~**300M** | **16 seconds per action**, **single NVIDIA RTX 4090**, 256x256 at 4 fps, receding-horizon replanning after every executed action | the paper frames 16 s as fast *relative to a 10x-sampled variant*; it is the reported figure |
| **AHEAD** | 4 / 6 | **+4.9M** on a frozen **7B** OpenVLA | **~158 ms per action step end-to-end on an H100 80GB at 224x224**: ~20 ms optical flow + ~40 ms world model (5 samples) + ~70 ms VLA; adaptive horizon averages 3-5 steps, K_max 10 | yes — states a **~200 ms** budget and reports 158 ms against it |
| **Khalil & Kwon zero-shot predictive-display benchmark** | 6 | LTX-Video 2B-distilled & 13B, SVD 1.1, Wan VACE 1.3B, Wan I2V 1.3B | **1.31 +/- 0.06 s to 3.85 +/- 0.08 s per frame**; **10.48 s to 30.80 s** per 88-frame rollout; peak GPU memory **3.15-46.68 GB**; **NVIDIA RTX 6000 Ada 48 GB**, source video 15 FPS (66.7 ms/frame), CARLA, 256x160 and 512x320 | **explicitly negative** — "no tested model simultaneously achieves low rollout error, non-divergent per-step error behavior, and real-time inference" |
| Chakraborty et al. delay-compensated video | 6 (geometry, not a world model) | ResNet-18 inpainter + Depth Anything V2 | **13 FPS** overall; depth **58 FPS**, inpainting **36 FPS**; benchmarked on a **2080** (training on two A100s); delays of **250 ms** and **500 ms** compensated | yes — "real-time", 13 FPS, evaluated in the field |
| RynnWorld-Teleop | 6 (name only; actually data generation) | Wan-2.2-TI2V-**5B** base; its own count not reported (full text checked) | **40.0 FPS at 480x832 on a single NVIDIA H100**, "approximately **25 ms**" per frame, split ~5% skeletal action encoding / ~72% causal DiT denoising / ~23% visual decoding; **4-step** flow-matching sampling after distillation; 81-frame clips at 16 FPS | yes — "real-time interactive generation" |
| Prescient teleoperation | 6 (command prediction) | not reported (full text checked); model built from 54 demonstrations | inference time **not reported (full text checked)**; controller runs at **100 Hz**, delayed mocap at 100 Hz, transmission to the robot controller at **50 Hz**; 2 s of observation gives "very accurate predictions for the next 4-5 seconds"; stochastic round-trip delays **200 ms to 2 s** (avg 1.5 s), forward delay **750 +/- 300 ms** | none |
| Residual RL under stochastic delays | 6 (command prediction) | LSTM 256-d x 3 layers; SAC actor/critic [512, 256] | inference **not reported (full text checked)**; **control frequency 250 Hz (dt = 4 ms)**; delays **U(120,160)** / **U(200,240)** / **U(40,240)** ms plus a constant 50 ms action delay, totals **170-210 / 250-290 / 90-290 ms** | none |
| 1X World Model Challenge report | benchmark | Wan-2.2 TI2V-**5B** + LoRA; entry parameter counts not reported (full text checked) | inference/sampling time **not reported (full text checked)**; *training* on **4 nodes x 8 NVIDIA B200** GPUs, ~36 h. Task: predict the **512x512** frame **22 s** ahead from 17 conditioning frames, scored on the **77th frame** | none |
| MiraBench | benchmark | DreamDojo 2B/14B, DreamDojo-GR1 2B/14B, Cosmos-GR1 2B/14B (+14B text), WAN 2.1 14B, WAN 2.2 5B, plus 3 closed models with sizes not given | **not reported (full text checked)** — no latency, no hardware, and no evaluation resolution or rollout length | none |

### What the table says as a whole

*Rewritten 2026-09-16 after the full-text pass. The first version of this section rested on
abstract-level reading for about thirty of the rows; the groupings below now rest on what the full
texts state, and two of the groups did not exist before.*

Of the 46 systems above, **sixteen now report an inference cost with named hardware** — up from
nine when only abstracts had been read. Grouping them by what one "step" of prediction actually
costs:

- **~3-25 ms per control step, latent or decoder-free, on a consumer GPU or on unnamed hardware** —
  Efficient-WAM ("Avg. Lat. per Step 6.1 ms" amortised over an H=16 chunk, **RTX 4090**, and 98 ms
  for the chunk itself), Being-H0.7 (**3-4 ms/step, hardware unnamed**, and no visual rollout at
  all). **This group is new in this pass and it is the finding that most changes the picture.** It
  is the only place in the review where a reported figure sits under this project's 11.1 ms frame,
  and both members get there the same way: no pixels. Two cautions travel with it — the 6.1 ms is
  an *amortisation of one 98 ms chunk over 16 steps*, not a closed-loop re-prediction rate, and
  Being-H0.7's figure has no hardware behind it and therefore cannot be placed against any other
  row.
- **~25-50 ms per generated frame, on a datacentre GPU or TPU, for a whole 360p-720p scene** —
  RynnWorld-Teleop (25 ms, **H100**, 480x832, 4-step distilled sampler), Oasis (~40-50 ms, GPU
  unnamed), GameNGen (50 ms on **a single TPU-v5**, 4 DDIM steps, 320x240), Genie 3 (~42 ms implied
  by 24 FPS, hardware undisclosed). Still the floor the *engineered-for-interactivity* end of the
  field has reached, and every one generates pixels for an entire scene.
- **~70-200 ms per action chunk covering 1-2 s of motion** — DSWAM (**73.8 ms** with TensorRT BF16
  on an **RTX 5090**, 198.2 ms in PyTorch), Efficient-WAM (98 ms, RTX 4090; 139 ms on an A800 in
  sim), AHEAD's world-model component (~40 ms of a 158 ms total, **H100**), LaWAM (187 ms per 1.2 s
  chunk, **A100**), GE-Act (200 ms per 1.8 s trajectory, **RTX 4090**). Viable only because the
  chunk is long and executed open-loop.
- **~100 ms-10 s per frame or per plan for pixel-space video world models** — DIAMOND CS:GO
  (100 ms, RTX 3090, project page), AVDC (**1.51 s per frame and 10.57 s per 8-frame plan on an
  RTX 3080Ti**, 100 denoising steps), the off-the-shelf models in the predictive-display benchmark
  (1.31-3.85 s/frame, RTX 6000 Ada). **Also new in this pass:** Genie reports itself at
  **"around 1FPS"**, and IRASim states flatly that "video generation is not real-time" without a
  number.
- **Tens of seconds per decision for search-based planning over a world model** — V-JEPA 2-AC
  (16 s per action, RTX 4090) and Navigation World Models (**30.3 s on an RTX 6000 Ada** for a
  32-trajectory ranking; its 0.1 s quantised figure is labelled "(est.)" by the authors and is a
  projection, not a measurement).

A separate group the first pass could not see, because it only becomes visible in full texts:

- **Latency reported *without* hardware.** Being-H0.7 (3-4 ms/step), Cosmos Policy (~5 s per
  planned chunk), Embodied.cpp (735.9 ms/step for HY-VLA, 56.85 ms for pi-0.5), Genie (~1 FPS),
  Oasis (20 FPS), Genie 3 (24 FPS). Six systems state a number nobody can place. Embodied.cpp is
  the sharpest case: a paper whose contract is explicitly "latency-first batch-1 inference on
  heterogeneous hardware" names its deployment targets only as a category ("Jetson devices,
  RK-based platforms, x86 edge boxes, and workstation-class systems") and never says which one
  produced Table 3.

Three structural observations follow, all of which are statements about the literature and none of
which is a claim about this system:

1. **The fast numbers are all throughput on datacentre or desktop accelerators, and none of them is
   a headset-class or embedded measurement.** Still true after the full-text pass, and now on a
   larger sample: the closest anything comes to an embedded target is DSWAM's RTX 5090 and
   Efficient-WAM's RTX 4090, both desktop parts. The only embedded target named anywhere in this
   review is the CoRL 2026 workshop's Jetson Orin NX challenge, which exists precisely because
   nobody has published that number. A Quest-class SoC does not appear in this literature at all.
2. **Every system that gets near real time does so by moving the expensive model off the control
   rate**, not by making it fast: dual-rate splits (GE-Act's 5 Hz world model / 30 Hz actions),
   long open-loop action chunks (LaWAM, Efficient-WAM, DSWAM), asynchronous execution (DayDreamer,
   DSWAM, Being-H0.7's Universal Asynchronous Chunking), distillation to few-step or single-pass
   sampling (RynnWorld-Teleop's 4 steps, GameNGen's 4, DIAMOND's 3, GAF's 3, LaDi-WM's 2, NWM's
   250-to-6), decoder removal (Being-H0.7, Dreaming), or predicting only what changed (Delta-IRIS's
   4 tokens per frame, DreamVLA's dynamic regions, Efficient-WAM's 192x160 future against a 384x320
   observation). The full-text pass strengthened this: **the denoising-step counts are almost
   universally small at inference — 2, 3, 4, 5 — and it is the abstracts, not the papers, that
   leave the impression of 50-step samplers.**
3. **The gap between latent and pixel prediction is about three orders of magnitude in
   named-hardware measurements, and it was one order when only abstracts had been read.** 6.1 ms
   per step (Efficient-WAM, RTX 4090, latent, low-resolution future) against 1.51 s per frame
   (AVDC, RTX 3080Ti, pixel, 128x128) is roughly 250x, and against NWM's 30.3 s it is larger still.
   Family 4's repeated claim that the decoder is the expensive part is, on the numbers now in this
   table, an understatement rather than an overstatement.

---

## Searches that found nothing useful

Recorded with the exact query so the next run does not repeat them. "Nothing useful" means either
no relevant hits or only hits already covered above.

- `"world model" teleoperation delay compensation robot arm latent dynamics predict remote scene
  operator VR` — returned VR teleoperation *interface* papers (dual-arm VR frameworks, spherical
  televisualisation, mesh/point-cloud digital twins, a USPTO patent on latency compensation in
  robotic teleoperation) and the two predictive-display papers already covered. **No hit combined a
  learned world model with operator-facing delay compensation.**
- `recurrent state space model RSSM predictive display teleoperation latency compensation robot` —
  returned RSSM tutorials, PlaNet itself, Dreaming, contrastive-RSSM control, and teleoperation
  papers using LSTM/TCN predictors. **No RSSM-based predictive display exists in the reachable
  results.** This was the single most targeted attempt at the bridge and it is empty.
- `world model latency compensation VR head mounted display remote robot 2026 latent rollout
  operator` — returned HMD latency measurement, timewarp/reprojection, display-stabilisation
  patents and quadcopter-piloting latency studies. All head-mounted-display latency work, none of
  it world models. The HMD-latency material belongs to the human-factors and networking files, not
  this one.
- `"model-mediated teleoperation" learned dynamics model neural network environment model delay` —
  model-mediated teleoperation is a real and adjacent literature (the operator interacts with a
  local environment model while the real feedback is delayed), and there is RL and LSTM work inside
  it, but **nothing found uses a world model in the sense of families 1-5 and 7.** The environment models
  in MMT are analytic contact models, not learned generative dynamics. This is the closest
  *conceptual* neighbour to the direction paragraph and the connection appears not to have been
  made in either direction.
- `"world model" predict robot arm stop at obstacle contact anticipation operator display network
  delay milliseconds` — this was a deliberately literal search for the exact capability the
  `aafa443` paragraph describes. It returned EMG/IMU limb-motion prediction for delay compensation,
  contact localisation without torque sensing, predictive shared control with tactile sensing, and
  two of the deployment papers above. **The literal intersection returned nothing.**
- `learned dynamics model on-device mobile SoC Quest standalone headset inference world model robot`
  — returned Meta's own Unity Inference Engine documentation for Quest on-device ML, a portable
  embodied runtime (Embodied.cpp, covered), and generic dynamics-distillation statements.
  **No published world model has been measured on a Quest-class mobile SoC in anything reachable
  here.** For this project the practical consequence is that the on-device operating point is
  entirely unmeasured in the literature, so it would have to be measured rather than looked up.

## Sources that could not be retrieved

Updated 2026-09-16 after a second, full-text-only pass. The three retries the first pass flagged
were attempted again; one succeeded by another route, one was replaced by an equivalent verified
route, and one is still unreachable from here.

- **GWM, ICCV 2025 Open Access** — **retried 2026-09-16 and still 403.** Both the Open Access PDF
  path (`openaccess.thecvf.com/content/ICCV2025/papers/Lu_GWM_..._paper.pdf`) and the HTML landing
  page (`.../html/Lu_GWM_Towards_Scalable_Gaussian_World_Models_for_Robotic_Manipulation_ICCV_2025_paper.html`)
  return HTTP 403 Forbidden to this fetcher. **The retry did succeed by a different route**: the
  arXiv HTML rendering <https://arxiv.org/html/2508.17600v1> (accessed 2026-09-16) serves the full
  text including appendices, and the GWM entry above is now based on it. The outcome is that GWM's
  timing cells stay `not reported` — but now as *"full text checked"*, which is a different and more
  useful statement than before.
- **LaDi-WM's CoRL OpenReview forum page** (`openreview.net/forum?id=o2w2iiMyEU`) — **not retried
  directly**; the interstitial is a property of OpenReview, not of the paper. The alternate route
  asked for was found and verified instead: the CoRL 2025 proceedings entry at
  <https://proceedings.mlr.press/v305/huang25a.html> (PMLR v305, pp. 1726-1743, accessed
  2026-09-16) loads the full author list and abstract, and is now given in the LaDi-WM entry. It
  reports no latency, hardware or parameter count, consistent with the arXiv full text.
- **"A Learning-Driven Visual Servoing Framework for Latency Compensation in Image-Guided
  Teleoperation"** — **still unretrieved after four routes tried on 2026-09-16.** ScienceDirect
  (`S1526149226000639`) returns HTTP 403; the ResearchGate publication page returns HTTP 403; the
  `web.archive.org` copy cannot be fetched by this tool at all; and no author preprint or PMC copy
  surfaced. Search metadata indicates it is a 2026 paper in *Computer Modeling in Engineering &
  Sciences* combining stereo vision, hand-eye calibration and LSTM/TCN predictors in a
  latency-aware visual-servoing loop, evaluated on in-vivo and phantom datasets — **but that is a
  search-engine summary, not a fetched source, so nothing from it is entered above and it has no
  entry.** It remains the single most on-topic unread source for the bridge question, and it is
  blocked on a human with institutional access.
- **Abstract-only rows are now largely resolved.** The first pass recorded that ~30 rows rested on
  abstracts and predicted a follow-up would fill about a third of the `not reported` cells. That
  follow-up ran on 2026-09-16 and reached the full text of **39 of them** — the whole Dreamer
  lineage, TD-MPC2, STORM, Dreaming, IRIS, Delta-IRIS, iVideoGPT, Genie, GameNGen, DIAMOND, Cosmos,
  IRASim, UniPi, RoboDreamer, AVDC, NWM, GWM, GAF, MRO-GWM, LaDi-WM, Efficient-WAM, Cosmos Policy,
  WorldVLA, DreamVLA, Being-H0.7, WMPO, DSWAM, Prescient teleoperation, Residual RL,
  RynnWorld-Teleop, the Motubrain async study, Embodied.cpp, GPC, MiraBench and the 1X report.
  Every cell those fetches changed is marked with the URL and the accessed date on the entry.
  **Cells that now read `not reported (full text checked)` mean the number does not exist in the
  paper, not that nobody looked** — that distinction is the whole point of the pass.
- **Routes that worked, recorded so the next pass does not rediscover them.** `arxiv.org/html/<ID>v<N>`
  serves 2024-2026 papers; the version suffix matters and a 404 on `v1` often means trying `v2` or
  `v3`. `ar5iv.labs.arxiv.org/html/<ID>` covers pre-2024 papers (Ha & Schmidhuber, PlaNet, Dreamer,
  DreamerV2, Dreaming, AVDC) where no arXiv HTML exists, and fails with a conversion error on some
  (DreamerV3, where `arxiv.org/html/2301.04104v2` worked instead). `proceedings.mlr.press` is a
  reliable substitute for an OpenReview forum page for any CoRL/ICML paper.
- **Where the number was in the paper, it was rarely in the prose.** Of the fills in this pass, the
  majority came from a table, a table caption, or an appendix: Delta-IRIS's speed claim is only
  correctly readable from Table 1's caption (it defines FPS as *training* throughput, which the
  headline does not), NWM's runtime is a table caption naming the GPU, DSWAM's TensorRT numbers are
  Table 4, Being-H0.7's control rates are Table 2, Residual RL's entire delay specification is
  Table 2 of an appendix, and TD-MPC2's planning population is Table 8. **A pass that reads only
  prose would have missed most of this file's content.**
- No PDFs were downloaded into the tree, per instruction. Everything above was fetched and read in
  place.

## What this file deliberately does not cover

Four sibling files were being written concurrently. To keep the boundaries clean:

- **Classical predictive display** (geometric/kinematic prediction, wireframe overlays, the Bejczy
  lineage) — not here, except where a modern system uses it as a component, as Chakraborty et al. do.
- **Learned predictive display** (LSTM/GAN/optical-flow video prediction for teleoperation) — the
  bridge section above touches it only where a *world model* is the predictor. Prescient
  teleoperation, residual RL under stochastic delays, and the EMG/IMU limb-prediction line are
  named here as boundary markers and belong to that file.
- **Human factors** (cybersickness, correction perceptibility, workload under delay) — not here.
  The one thing this file contributes to it is MiraBench's **optimism bias** finding, which is a
  human-facing failure mode of generative predictors and has no counterpart in a dead-reckoner.
- **Networking** (jitter, playout, loss) — not here at all.

Also absent by scope: autonomous driving world models (GAIA, DriveDreamer and relatives), which
both 2026 surveys cover and which are a large fraction of the video-world-model literature. They
were skipped because the operating point (forward-facing camera, 10-30 Hz, vehicle-scale motion) is
further from a manipulator than anything included here, and coverage was spent on manipulation
instead. That is a deliberate omission, not an oversight.

---

## Where this touches the project — arguments only, nothing settled

Per `docs/literature/CLAUDE.md`, none of this is evidence about this system, and none of it can
close a question that a sweep has to close. What follows is framed as arguments a human might act
on, including arguments about files this survey was not permitted to edit.

**1. The direction paragraph's target capability is unoccupied, and its cheapest form is not a
world model.** No source found predicts the remote scene forward for a delayed human operator using
a learned world model. The nearest occupied positions are (a) generative predictive display, which
one benchmark reports as 20x-58x too slow off the shelf at 15 FPS on a 48 GB workstation GPU, and
(b) AHEAD, which does latent-space latency compensation on a real arm for ~40 ms of world-model
time — but for the robot's own controller, with no human in the loop. Meanwhile Being-H0.7 and
Dreaming both report that the predictive benefit survives deleting the decoder entirely. If a human
wants the smallest thing in this space that could be built here, prior work points at
*future-conditioned training with future-free inference over a low-dimensional state*, not at
generating anything an operator would look at.

**2. `core/Teleop.Core/Prediction/CLAUDE.md`'s `seq-model` row may be under-specified in a way this
literature can sharpen.** The row says only "uses `IInferenceBackend`, never an ONNX library
directly". Nothing in it distinguishes *one forward pass per `Predict` call* from *a sampled rollout
per call*, and the table above shows that distinction is worth two to four orders of magnitude:
V-JEPA 2-AC's 16 s per action is CEM sampling, not a forward pass, while AHEAD's world model is
~40 ms for 5 samples over a 3-5 step horizon. An argument for a human: the planned row is really two
rows. This file cannot edit that table and does not propose a specific wording.

**3. `Contracts/IInferenceBackend.cs` already demands determinism, and nearly every fast system
found here is a diffusion sampler.** Diffusion inference is stochastic by construction; DIAMOND
reports 3 denoising steps, Efficient-WAM reports asymmetric step allocation, RynnWorld-Teleop
distils the sampler to a single pass. Any of those would need its sampling noise drawn from the
injected seeded RNG (invariant 4) for replay to be bit-identical. The contract's existing
determinism clause appears to already cover this; recorded so that whoever writes the ADR does not
discover it late.

**4. Two metric-shaped observations, neither of which is a proposal to change `docs/metrics.md`.**
MiraBench reports that world models exhibit widespread **optimism bias** — predicting success when
the commanded action should fail — and that visual quality does not predict action fidelity. A
dead-reckoner fails symmetrically and visibly; a learned predictor may fail confidently and
plausibly, which is a different thing to measure and is not obviously captured by prediction error
plus correction cost. Separately, the Motubrain asynchronous-deployment study reports
**chunk-boundary discontinuity** that "cannot be corrected through blending alone" when timestamps
are misaligned — i.e. a correction-cost-shaped failure whose cause is alignment, not smoothing.
Both are arguments that a human may want to consider *before* any world-model work starts, and
neither is a redefinition of anything: metrics are not changed in a survey.

**5. The one architectural pattern the literature agrees on, and this repo already has half of.**
Every system in the table that approaches real time separates a slow model from a fast path:
GE-Act's 5 Hz world model driving a 30 Hz action stream, DayDreamer's decoupled actor/learner,
DSWAM's dual system with async execution, Embodied.cpp's "multi-rate execution inside closed-loop
control". This project's `Pipeline/`, `Buffering/` and `Reconciliation/` axes already are a fast
path that decides when a sample is shown and how its correction is absorbed. Prior work reports
that the slow-model-plus-fast-path split is what makes these systems deployable at all; what it
does not report anywhere, for any system, is that split running on a mobile SoC at 90 Hz. That
number does not exist in the literature and would have to be produced here.
