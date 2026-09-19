# human factors — what operators actually perceive

Scope: latency detection and just-noticeable differences in VR and teleoperation, motion-to-photon
thresholds, cybersickness and how it is measured, sensory-conflict theory and its competitors,
Fitts' law under delay, and the perceptual basis (or absence of one) for the three numbers this
project currently treats as perceptual constants: **peak jerk as a nausea proxy**, a **5 mm
position tolerance**, and a **100 ms convergence budget**.

Nothing here is evidence about this system. Prior work reports X under conditions Y; whether X
holds on a Quest rendering a proxy arm over a 300 ms link is a sweep, not a reading. Where a
source's conditions are far from this project's operating point, that is said in `Conditions:` and
should be read as the main content of the entry, not a footnote.

Accessed dates are all 2026-09-15 unless stated.

**Status: in progress.**

---

## 1. How far the operating point is from the literature's

Almost every number below was measured in one of three setups, none of which is this project:

1. **Whole-body physical motion** — ships, cars, aircraft, motion platforms. Vestibular stimulation
   present, low frequency (0.1–0.5 Hz), exposure measured in tens of minutes to hours. This is
   where dose–response models for sickness actually come from.
2. **Head-referenced VR** — an HMD, a world-stationary scene, the participant rotating their head.
   The error signal is scene motion slaved to head motion. This is where latency thresholds come
   from.
3. **Hand-referenced VR** — a virtual hand or tool offset from the real hand. The error signal is
   visual-proprioceptive discrepancy. This is where positional-offset thresholds come from.

This project is a **fourth** case: a seated, physically still operator watching a rendered proxy of
a remote arm that is neither their head nor their hand, corrected by an algorithm. Case 1 supplies
its provocative-frequency band; case 2 supplies its latency thresholds; case 3 is the nearest
analogue for its 5 mm tolerance and is still not the same thing, because a proxy robot arm has no
proprioceptive referent at all. Read every `Conditions:` field as the main content.

---

## 2. Latency detection and just-noticeable differences

The recurring finding is that there is **no single latency threshold**. Threshold is a function of
how fast the observer is moving, it varies by an order of magnitude between observers, and the
quantity actually detected is not delay but the *scene motion* delay produces. The project's own
`velocity-match` reconciler rests on a version of this premise; prior work supports the premise's
shape (threshold depends on motion) while measuring it on head yaw, not on a hand-guided proxy.

> **Jerald, "Scene-Motion- and Latency-Perception Thresholds for Head-Mounted Displays"**, PhD
> dissertation, UNC Chapel Hill, tech report 10-013 (2010).
> <https://www.cs.unc.edu/techreports/10-013.pdf> (accessed 2026-09-15).
> *Proposes:* that users do not detect latency directly but detect latency-induced scene motion,
> and that the latency threshold is an **inverse function of peak head-yaw acceleration** rather
> than a constant or a linear function of it.
> *Claims:* across five psychophysics experiments, the inverse-acceleration model correlates with
> measured latency thresholds better than a linear model. In Experiment 4, pooling subjects across
> six peak-head-yaw-acceleration deciles (60 threshold estimates): mean 75% detection threshold
> **55.6 ms (SD 23.4, range 19.2–154.1 ms)**; mean 50% threshold 38.7 ms (SD 18.4, range −0.2 to
> 103.2 ms); mean latency *difference* threshold 16.9 ms (SD 10.4, range 3.2–60.5 ms). The author
> reads the 3.2 ms minimum as the figure a system must beat to be unconditionally imperceptible.
> *Conditions:* full dissertation text read. A **simulated-by-projector "HMD"** with effectively
> zero latency (CRT projector chosen over DLP/LCD for response time), into which scene motion was
> artificially injected — so the stimulus is a clean synthetic scene velocity, not a real rendering
> pipeline's delay. Task is yaw head rotation against a world-stationary scene; the negative 50%
> threshold means at least one observer reported motion in a perfectly stable scene. Nothing here
> involves a hand-held or hand-guided object, and nothing involves a *moving* object whose motion
> is expected.

> **Jerald, Whitton & Brooks, "Scene-motion thresholds during head yaw for immersive virtual
> environments"**, ACM Transactions on Applied Perception 9(1), 1–23 (2012).
> <https://colab.ws/articles/10.1145%2F2134203.2134207> (accessed 2026-09-15).
> *Proposes:* the journal form of the above, measuring scene-motion thresholds under varied head
> yaw, scene-motion direction, and illumination.
> *Claims:* thresholds are **larger when the scene moves with head yaw** (gain < 1.0) than against
> it (gain > 1.0), and thresholds increase as head motion increases.
> *Conditions:* **abstract only.** `dl.acm.org` returns HTTP 403 to automated fetches, so this was
> verified through the `colab.ws` DOI mirror, which carries title, authors, journal, volume, year
> and an abstract-level summary but no numeric thresholds and no apparatus description. Three
> experiments involving head yaw, two illumination levels and varied head-yaw speed. The
> dissertation above is the fuller readable version of the same programme and is the entry to
> trust for numbers.

> **Stauffert, Niebling & Latoschik, "Latency and Cybersickness: Impact, Causes, and Measures. A
> Review"**, Frontiers in Virtual Reality 1:582204 (2020).
> <https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2020.582204/full>
> (accessed 2026-09-15).
> *Proposes:* a review tying motion-to-photon latency to cybersickness, with a taxonomy of 24
> latency-measurement methodologies and a strong argument that **a mean latency figure is not a
> sufficient description of a system**.
> *Claims:* latency is not a scalar — it exhibits spiking and periodic structure, and *occasional
> latency spikes* provoke cybersickness (Stauffert et al. 2018), as does periodic latency (St.
> Pierre et al. 2015; Kinsella et al. 2016), with the field unresolved on whether spike amplitude
> or spike frequency matters more. Reports detection as low as 3.2 ms for one participant and a
> commonly quoted ~17 ms imperceptibility figure. Reports that **58 of 76 reviewed experiments
> (~76%) used the SSQ**, and that only ~7% of IEEE VR 2020 papers reported their motion-to-photon
> latency at all.
> *Conditions:* full text read. A review, not a new measurement: the 11 primary studies it
> aggregates span 8–164 participants, absolute latencies 5–355 ms, HMDs (Vive, DK2, Rift) and
> driving simulators, with tasks including visual search with head movement and balance
> maintenance. All head-referenced; none is a rendered remote-robot proxy.

> **MacKenzie & Ware, "Lag as a determinant of human performance in interactive systems"**,
> Proc. INTERCHI '93, 488–493. <https://colab.ws/articles/10.1145%2F169059.169431>
> (accessed 2026-09-15).
> *Proposes:* that lag interacts **multiplicatively** with Fitts' index of difficulty rather than
> adding a constant to movement time.
> *Claims:* at 225 ms lag, movement time rose 64% and error rate 214% relative to zero lag; the
> multiplicative model accounted for 94% of variance and beat additive alternatives.
> *Conditions:* **abstract only.** `dl.acm.org` returns HTTP 403 to automated fetches (both the
> landing page and the PDF) and the author-hosted copy at `yorku.ca/mack/CHI93b.html` fails TLS
> certificate verification, so this was verified through the `colab.ws` DOI mirror. Participant
> count, input device, display and the exact lag ladder were **not** read from a primary source —
> only that there were four lag conditions with a 225 ms maximum. It is a 1993 **desktop pointing**
> study with a 2D target-acquisition task, not VR and not a 3D proxy.

> **Waltemate, Senna, Hülsmann, Rohde, Kopp, Ernst & Botsch, "The impact of latency on perceptual
> judgments and motor performance in closed-loop interaction in virtual reality"**, Proc. ACM VRST
> '16. <https://colab.ws/articles/10.1145%2F2993369.2993381> (accessed 2026-09-15).
> *Proposes:* separating the latency at which motor performance degrades from the latency at which
> people *notice* delay, and from the latency at which sense of agency and body ownership break, in
> one closed-loop VR task.
> *Claims:* motor performance and simultaneity perception degrade noticeably **above ~75 ms**;
> sense of agency and body ownership degrade only **above ~125 ms**, with further deterioration
> above ~300 ms, and neither collapses completely even at the maximum delay tested. Participants
> "perceptually infer the presence of delays more from their motor error in the task than from the
> actual level of delay."
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror (`dl.acm.org` returns HTTP 403).
> A **CAVE**, not an HMD; the stimulus is the participant's own **virtual mirror image**, so the
> displayed object is a self-avatar with a full proprioceptive referent. Delays 45–350 ms,
> psychophysical procedure. Participant count not read.

> **Rahimian, Plumert & Kearney, "The Effect of Visuomotor Latency on Steering Behavior in Virtual
> Reality"**, Frontiers in Virtual Reality 2:727858 (2021).
> <https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2021.727858/full>
> (accessed 2026-09-15).
> *Proposes:* an A-B-A design to separate the *immediate* cost of latency from the operator's
> **adaptation** to it.
> *Claims:* average lateral error during the latency-training phase was almost three times larger
> than in the pre- and post-latency phases, but participants adapted rapidly; self-paced riders did
> worse under latency than speed-controlled ones.
> *Conditions:* full text read. Two experiments, 16 participants each (mean age ~18.6), HTC Vive,
> steering a stationary bike along an illuminated path with 90° turns. Baseline system latency
> ~46 ms; the latency condition added 385 ms for a total of ~431 ms. **Sickness was not measured.**
> The operating point (≈430 ms, continuous steering) is the closest in this file to this project's
> impaired network profiles, and the finding that matters is the adaptation: a latency cost
> measured in the first minutes is not the same number as the same cost measured after training.

### Teleoperation specifically

> **Farajiparvar, Ying & Pandya, "A Brief Survey of Telerobotic Time Delay Mitigation"**,
> Frontiers in Robotics and AI 7:578805 (2020).
> <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.578805/full>
> (accessed 2026-09-15).
> *Proposes:* a survey of delay mitigation in telerobotics, from Ferrell & Sheridan's 1960s
> move-and-wait results through phantom-robot predictive displays to modern intent modelling.
> *Claims:* operators facing transmission delay spontaneously adopt a **move-and-wait strategy** —
> command, stop, wait for feedback, correct — which works but inflates completion time and destroys
> smoothness. For the surgical-teleoperation case it reports 100–250 ms as having minimal
> performance impact, ~400 ms as the acceptable ceiling, and measurable degradation beyond that,
> with completion time rising roughly linearly with delay.
> *Conditions:* full text read, but this is a **survey**: the 100/250/400 ms figures are attributed
> to a robotic-coronary-stenting study, i.e. expert surgeons, a specific instrument, force and
> visual feedback, and a task where errors are consequential. Nothing in it is a Quest HMD or a
> rendered proxy arm. Its value here is the move-and-wait mechanism, which is a *behavioural*
> adaptation: an operator who has started move-and-waiting has changed the input distribution the
> predictor sees, which is the closed-loop problem `Autonomy/CLAUDE.md` already flags.

---

## 3. Cybersickness and how it is measured

`docs/metrics.md` §7 names the SSQ as the subjective instrument for any future human-facing claim.
The SSQ is the field's default and it is also the field's most criticised instrument, so a project
that names it should know what the criticisms are before it ever runs a session.

> **Kennedy, Lane, Berbaum & Lilienthal, "Simulator Sickness Questionnaire: An Enhanced Method for
> Quantifying Simulator Sickness"**, The International Journal of Aviation Psychology 3(3),
> 203–220 (1993).
> <https://cgvr.cs.uni-bremen.de/teaching/vr_literatur/Simulator%20Sickness%20Questionnaire,%20An%20Enhanced%20Method%20for%20Quantifying%20Simulator%20Sickness%20-%20Kennedy,%20Lane,%20Berbaum,%20Lilienthal,%201993.pdf>
> (accessed 2026-09-15).
> *Proposes:* a 16-symptom, 4-point instrument scored into Nausea, Oculomotor and Disorientation
> subscales plus a Total Severity, derived by factor analysis from the older Motion Sickness
> Questionnaire with symptoms that do not discriminate in simulators removed.
> *Claims:* the scoring is deliberately "least dependent on the precision of parameter values
> derived from the sample" — weights are 1 or 0 by whether the varimax loading exceeded .30 — and
> the constants are chosen so that no symptoms scores zero and **the standard deviation of the
> scaled scores is 15 on the calibration sample**.
> *Conditions:* full text read. Derived from **1,119 pre/post questionnaire pairs collected at ten
> US Navy flight simulators**, i.e. military aviators in fixed-base flight simulators around 1989.
> Three caveats stated by the authors themselves, all of which bear on VR use: (i) "fixed-base
> flight simulators differ in a critical respect from ships at sea: namely, when one closes one's
> eyes in a simulator, the stimulus stops"; (ii) the scoring procedure presumes that unwell
> participants are screened out and that **"only postexposure data are scored"**; (iii) "the scales
> do not distinguish among simulators that have no problems, but are rather intended to discriminate
> problem simulators from those with no indicated difficulties." Point (iii) is the one most often
> ignored: the instrument was built as a *screening* tool for identifying bad simulators, not as a
> continuous dependent variable for ranking two conditions that are both fine.

> **Bimberg, Weissker & Kulik, "On the Usage of the Simulator Sickness Questionnaire for Virtual
> Reality Research"**, IEEE VR Abstracts and Workshops (VRW) 2020, DOI
> 10.1109/VRW50115.2020.00098.
> <https://www.uni-weimar.de/en/media/chairs/computer-science-department/vr/research/hci/the-ssq-in-vr-research/>
> (accessed 2026-09-15).
> *Proposes:* an audit of how the SSQ is actually applied in VR papers, with recommendations.
> *Claims:* studies compute and report SSQ results inconsistently (which score, weighted or not,
> pre/post handling), producing large differences between nominally comparable works and making
> cross-study comparison unsound; the instrument's 1990s military-simulator provenance is a poor
> fit for modern VR and simpler instruments are often more appropriate.
> *Conditions:* **abstract/project-page level only.** The IEEE full text was not retrieved; the
> author group's project page confirms title, authors, venue, year and DOI and summarises the
> argument, but the number of papers audited and the size of the differences were not read.

> **Brown, Spronck & Powell, "The simulator sickness questionnaire, and the erroneous zero baseline
> assumption"**, Frontiers in Virtual Reality 3:945800 (2022).
> <https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2022.945800/full>
> (accessed 2026-09-15).
> *Proposes:* that the common practice of scoring only post-exposure SSQ — exactly what Kennedy et
> al. prescribe — assumes participants arrive symptom-free, and that this assumption is false.
> *Claims:* median **pre-exposure** SSQ was 8 (IQR 4–14.5) and **96.8% of participants reported a
> non-zero baseline** before any VR exposure at all; healthy and medically-affected subgroups had
> comparable non-zero baselines. Recommends Bouchard's unweighted scoring over Kennedy's weighted
> scheme as more representative of a general population.
> *Conditions:* full text read. 93 participants aged 18–80 (mean 29 ± 15), 55 self-reported healthy
> and 38 with a medical condition or medication, recruited from a university and social media. The
> measurement is a **baseline questionnaire before any VR** — no VR content, no display, no task —
> so this is a statement about the instrument, not about any VR system.

> **Kim, Park, Choi & Choe, "Virtual reality sickness questionnaire (VRSQ): Motion sickness
> measurement index in a virtual reality environment"**, Applied Ergonomics 69, 66–73 (2018).
> <https://colab.ws/articles/10.1016%2Fj.apergo.2017.12.016> (accessed 2026-09-15).
> *Proposes:* a 9-item VR-specific reduction of the SSQ, retaining two components — **oculomotor
> and disorientation — and dropping the nausea component**.
> *Claims:* the reduced instrument is appropriate for VR; target-selection method and button size
> significantly affected sickness.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror; ScienceDirect was not retrieved.
> Derived from **24 participants performing target-selection tasks** with a VR device, Latin-square
> ordered, SSQ administered after each task. A 24-participant selection-task study is a thin base
> for a general instrument, and dropping nausea is a substantive choice for a project whose own
> headline proxy is named "the nausea proxy" — an instrument that cannot report nausea cannot
> validate one.

> **Kourtesis, Linnell, Amir, Argelaguet & MacPherson, "Cybersickness in Virtual Reality
> Questionnaire (CSQ-VR): A Validation and Comparison against SSQ and VRSQ"** (2023).
> <https://arxiv.org/abs/2301.12591> (accessed 2026-09-15).
> *Proposes:* a further cybersickness instrument in both paper and interactive in-VR 3D forms,
> on the argument that "both [SSQ and VRSQ] suffer from important limitations".
> *Claims:* substantially better internal consistency than SSQ and VRSQ, and better detection of
> the temporary cognitive/psychomotor performance decline that cybersickness causes; **pupil size
> was a significant predictor of cybersickness intensity**.
> *Conditions:* abstract and preprint landing page read. 39 participants, three VR "rides" with
> linear and angular accelerations, cognitive and psychomotor assessment at baseline and after each
> ride. The provocative stimulus is passive transport with imposed acceleration — the opposite of
> this project's stationary seated operator — and the sample is small.

> **Keshavarz & Hecht, "Validating an Efficient Method to Quantify Motion Sickness"**, Human
> Factors 53(4), 415–426 (2011). <https://colab.ws/articles/10.1177%2F0018720811403736>
> (accessed 2026-09-15).
> *Proposes:* the **Fast Motion Sickness scale (FMS)** — a single verbal 0–20 rating given once per
> minute *during* exposure, rather than a questionnaire before and after.
> *Claims:* FMS correlates strongly with the SSQ administered conventionally (r = .785 with SSQ
> total, r = .828 with the nausea subscore); no significant expectancy effects.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror. 126 volunteers across two
> experiments. The property that matters for this project is that FMS yields a **time series**,
> which is the only questionnaire form under which a cumulative-dose model of sickness (§7 below)
> could ever be tested against an extreme-value one.

### Objective measures

> **Stoffregen & Smart, "Postural instability precedes motion sickness"**, Brain Research Bulletin
> 47(5), 437–448 (1998).
> <https://experts.umn.edu/en/publications/postural-instability-precedes-motion-sickness/>
> (accessed 2026-09-15).
> *Proposes:* postural sway measured before symptom onset as an objective precursor of sickness,
> and as evidence for postural-instability theory over sensory conflict.
> *Claims:* increases in postural sway appeared **before** the onset of subjective symptoms; about
> half of participants became sick.
> *Conditions:* **abstract level**, via the institutional repository record. Participants **stood**
> in a physical "moving room" with nearly global oscillating optical flow, a sum-of-sines between
> **0.1 and 0.3 Hz** with an excursion of only **1.8 cm** — a stimulus so slow and small it often
> went unnoticed, yet it made half the room sick. Two points transfer awkwardly and usefully at
> once: the provocative frequency band is again 0.1–0.3 Hz, and the measure requires a *standing*
> observer, which this project's seated operator is not.

> **Dennison, Wisti & D'Zmura, "Use of physiological signals to predict cybersickness"**, Displays
> 44, 42–52 (2016). <https://colab.ws/articles/10.1016%2Fj.displa.2016.07.002>
> (accessed 2026-09-15).
> *Proposes:* physiological estimation of cybersickness from stomach activity (electrogastrography),
> blinking and breathing.
> *Claims:* those signals estimate post-immersion symptom scores with R² up to 0.75; physiological
> measures alone discriminate HMD from monitor viewing at 78% accuracy. **Cybersickness occurred
> only in the HMD condition**, with half of subjects withdrawing after six minutes.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror. **Seated** participants — the one
> posture match in this section — interacting with a virtual environment, monitor versus HMD.
> Participant count not read. The monitor-versus-HMD asymmetry is worth noting before any attempt
> to validate a display-side metric on a desktop replay.

---

## 4. Theories of why people get sick, and why the choice matters here

The three live accounts make **different predictions about a stationary seated operator watching a
proxy**, which is why this is not idle theory for this project.

- **Sensory conflict / subjective vertical mismatch** predicts sickness when sensed and expected
  orientation disagree, and requires a functioning vestibular system in the loop. A seated,
  physically still operator watching a small rendered object that produces no self-motion illusion
  generates very little conflict about the *subjective vertical*.
- **Postural instability** predicts sickness from destabilised postural control, and its evidence
  base is standing observers.
- **Differences in virtual and physical pose (DVP)** predicts sickness from the mismatch between
  where the user's *head* is and where the virtual viewpoint is — a display-side, measurable,
  position-domain quantity. This is the only one of the three that names a signal a headless
  pipeline could actually emit.

> **Oman, "Motion sickness: a synthesis and evaluation of the sensory conflict theory"**, Canadian
> Journal of Physiology and Pharmacology 68(2), 294–303 (1990).
> <https://colab.ws/articles/10.1139%2Fy90-044> (accessed 2026-09-15).
> *Proposes:* a control-engineering (observer-theory) formalisation of the sensory-conflict
> hypothesis, giving the dynamic coupling between a conflict signal and nausea magnitude.
> *Claims:* the conflict formulation accounts for variants like spectacle sickness and flight
> simulator sickness that occur with normal or **absent** physical motion; identifies gaps in the
> neurophysiology, asking what properties a "conflict neuron" would need.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror (`cdnsciencepub.com` returns HTTP
> 403). A theoretical synthesis, not an experiment; no participants, no apparatus. The important
> structural feature for this project is that the model is **dynamic and accumulating** — nausea is
> the output of a filter driven by a conflict signal over time, not a function of an instantaneous
> peak.

> **Riccio & Stoffregen, "An ecological theory of motion sickness and postural instability"**,
> Ecological Psychology 3(3), 195–240 (1991).
> <https://bibbase.org/network/publication/riccio-stoffregen-anecologicaltheoryofmotionsicknessandposturalinstability-1991>
> (accessed 2026-09-15).
> *Proposes:* the main rival to sensory conflict — that "animals become sick in situations in which
> they do not possess (or have not yet learned) strategies that are effective for the maintenance of
> postural stability", so sickness follows from the demands a situation places on the control of
> action, not from stimulation patterns as such.
> *Claims:* motion sickness and postural instability co-occur across otherwise unrelated provocative
> situations.
> *Conditions:* **abstract only**, via a bibliographic aggregator (Taylor & Francis returns HTTP
> 403). A theory paper. Its empirical arm (Stoffregen & Smart 1998, §3) requires a **standing**
> observer, so the theory makes no sharp prediction for this project's seated operator and its
> characteristic measure — sway — is not available.

> **Bos, Bles & Groen, "A theory on visually induced motion sickness"**, Displays 29(2), 47–57
> (2008). <https://research.vu.nl/en/publications/a-theory-on-visually-induced-motion-sickness>
> (accessed 2026-09-15).
> *Proposes:* extending subjective-vertical-mismatch theory to purely visual stimulation, via
> visual-vestibular interaction.
> *Claims:* "people without functioning inner ears do not get sick from motion, **including visual
> motion**" — the vestibular system is in the loop even for visually induced sickness; the framework
> then predicts VIMS from the visual contribution to the subjective vertical.
> *Conditions:* **abstract only**, via the VU Amsterdam repository record. A theory paper. The
> labyrinthine-defective observation is the sharpest constraint in this file on what a purely visual
> stimulus can do: if sickness routes through the vestibular estimate of the vertical, a small
> rendered arm that does not perturb the operator's sense of upright has a weak mechanism available
> to it.

> **Keshavarz, Riecke, Hettinger & Campos, "Vection and visually induced motion sickness: how are
> they related?"**, Frontiers in Psychology 6:472 (2015).
> <https://www.frontiersin.org/journals/psychology/articles/10.3389/fpsyg.2015.00472/full>
> (accessed 2026-09-15).
> *Proposes:* a review of whether illusory self-motion (vection) is necessary and/or sufficient for
> visually induced motion sickness.
> *Claims:* vection is **not sufficient** — many participants have vection without sickness — and is
> plausibly but not certainly **necessary**, since VIMS rarely appears in participants reporting no
> vection, though Ji et al. (2009) found VIMS-like symptoms without reported vection. Correlations
> between vection strength and VIMS strength vary widely across studies. Faster optokinetic-drum
> rotation increased VIMS severity, but speed changes conflict and postural stability at the same
> time, so the manipulation is not clean.
> *Conditions:* full text read. A review of optokinetic-drum, large-screen and simulator studies.
> The relevance here is negative and load-bearing: this project's stimulus — a **small,
> non-self-referential rendered object** in a stable scene — is close to the minimum plausible
> vection stimulus. If vection is even roughly necessary for VIMS, the mechanism by which
> reconciler-induced proxy motion could make anyone sick is unclear and unestablished.

> **Palmisano, Allison & Kim, "Cybersickness in Head-Mounted Displays Is Caused by Differences in
> the User's Virtual and Physical Head Pose"**, Frontiers in Virtual Reality 1:587698 (2020).
> <https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2020.587698/full>
> (accessed 2026-09-15).
> *Proposes:* **DVP** — the time-varying difference between the user's physical head pose and the
> virtual head pose rendered for them — as the proximal cause of cybersickness in HMDs, replacing
> "latency" as the explanatory variable with the *pose discrepancy* latency produces.
> *Claims:* mean unsigned DVP, **peak DVP and the standard deviation of DVP** all had significant
> positive linear relationships with cybersickness severity, for both pitch and yaw head rotation.
> *Conditions:* full text read. Two studies: 30 participants (pitch) and 14 (yaw), Oculus Rift CV1
> at 90 Hz, lag imposed by a frame buffer from 0 to ~200 ms on top of ~4 ms baseline, **continuous
> oscillatory head rotation at 0.5 or 1.0 Hz** driven by instruction, sickness rated on the Fast
> Motion Sickness Scale. DVP is an **angular difference in degrees between physical and virtual
> head orientation** — a head-referenced, zeroth-derivative quantity. It is not the displacement of
> a rendered object relative to its own past, which is what this project's jerk metric measures.
> The forced 0.5–1.0 Hz head oscillation also means the whole stimulus sits an order of magnitude
> above the 0.1–0.5 Hz band that the physical-motion dose literature identifies as provocative.

---

## 5. (a) Is jerk of a displayed proxy a validated proxy for sickness?

**Short answer: no, and the three possible answers separate cleanly.** For *physical whole-body
motion* the question has been studied and the established dose measures do **not** use jerk — they
use band-weighted acceleration integrated over time. For *physical motion and discomfort* (not
sickness) jerk has been studied directly and the one controlled experiment below found the effect
**in the opposite direction** to the assumption. For the *visual, non-vestibular,
physically-stationary* case — this project's case, the jerk of a rendered object seen by a still
observer — the literature searched here contains **no study at all**. Unstudied, not supported and
not contradicted; and adjacent to a contradiction.

The single most useful sentence: **the established motion-sickness dose measures weight
*frequency*, not derivative order.** ISO 2631-1's motion-sickness prediction is a band-limited
weighting centred near 0.2 Hz applied to *acceleration*, root-integrated over exposure time. There
is a peak-sensitive measure in the same standards family — the fourth-power Vibration Dose Value —
but it belongs to the **vibration/health** branch (roughly 0.5–80 Hz) and is invoked by a crest
factor criterion, not to the motion-sickness branch. Nothing in that family differentiates
acceleration a third time.

> **Nooij, "A review on the effects of motion characteristics on motion sickness incidence"**, Max
> Planck Institute for Biological Cybernetics Technical Report No. 197 (2018).
> <https://pure.mpg.de/rest/items/item_3008626_2/component/file_3008627/content>
> (accessed 2026-09-15).
> *Proposes:* a review of which properties of motion predict motion-sickness incidence, organised
> around the ISO 2631-1 prediction method.
> *Claims:* the standard's predictor is the **motion sickness dose value**,
> `MSDV = [∫₀ᵀ a_w(t)² dt]^(1/2)` where `a_w` is the **frequency-weighted acceleration** and `T` the
> exposure duration (units m·s^−1.5), with incidence `MSI = k · MSDV` and `k ≈ 0.33` estimating the
> percentage expected to vomit. Provocativeness peaks near **0.2 Hz**; for vertical oscillation the
> weighting rises ~6 dB/octave below the peak and falls ~12 dB/octave above it, and for fore-aft
> oscillation ~2–3 dB/octave and ~3–4 dB/octave respectively (a broader peak). Horizontal
> oscillation can be about twice as provocative as vertical at the same magnitude, so the standard
> under-predicts for horizontal motion.
> *Conditions:* full text read. **Physical whole-body motion only.** The MSDV parameters come from
> Lawther & Griffin's ship data — more than 20,000 ferry passengers — and from McCauley et al.'s
> laboratory vertical-oscillation work; the car and aircraft sections come from Turner & Griffin's
> coach and short-haul-flight surveys. Exposures are minutes to hours. **The word "jerk" does not
> occur anywhere in the review.** One incidental finding worth this project's attention: in the
> coach studies, when passengers had a good view of the road ahead, "the effect of MSDV was absent"
> — anticipation nulled a dose effect entirely.

> **Killen & Eger, "Whole-body vibration: Overview of standards used to determine health risks"**,
> Centre of Research Expertise for the Prevention of Musculoskeletal Disorders, University of
> Waterloo, position paper, last updated 2016.
> <https://uwaterloo.ca/centre-of-research-expertise-for-the-prevention-of-musculoskeletal-disorders/resources/position-papers/whole-body-vibration-overview-standards-used-determine>
> (accessed 2026-09-15).
> *Proposes:* a practitioner's summary of ISO 2631-1 and related standards.
> *Claims:* the basic evaluation is frequency-weighted **RMS** acceleration, which is "relatively
> insensitive to acceleration peaks or shocks"; when the **crest factor exceeds 9** the standard
> directs the analyst to the **fourth-power Vibration Dose Value** instead, which is deliberately
> more responsive to peaks (8-hour equivalent VDV in m·s^−1.75).
> *Conditions:* a secondary summary, not a primary study; the standards themselves (ISO 2631-1:1997
> and ISO 2631-5) are paywalled and were **not** read — recorded here by number, per the brief. The
> page does not cover motion sickness or its separate frequency weighting. The structure is the
> point: this family of standards *does* have a principled, quantitative rule for when peaks matter
> more than averages, and it is a **crest-factor test that switches the whole estimator to a
> fourth-power integral over the entire exposure**, not a rule that says "read the p99".

> **de Winkel, Irmak, Happee & Shyrokau, "Standards for passenger comfort in automated vehicles:
> Acceleration and jerk"**, Applied Ergonomics 106, 103881 (2023).
> <https://repository.tudelft.nl/file/File_e43765bf-dcb9-4358-9f33-0e76ad3a707c?preview=1>
> (accessed 2026-09-15).
> *Proposes:* an experiment that varies peak acceleration and peak jerk independently and asks
> which predicts discomfort, then tests ISO 2631's vibration and shock models against the data.
> *Claims:* discomfort increases with **peak acceleration** (average "so-so" acceptability point
> 1.23 m/s² longitudinal, 0.98 m/s² lateral). **The effect of jerk was negative on average — higher
> jerk was associated with *less* discomfort** — and triangular pulses, which have jerk plateaus,
> were judged *more* comfortable than sinusoidal ones at matched peak jerk. The authors' own
> hypothesis had been that discomfort rises with jerk. Their stated caveat, verbatim in substance:
> higher jerk also meant shorter pulses, some participants said the briefness made the pulse
> negligible while others said the stronger kick was worse, block-shaped profiles with extreme jerk
> were not tested, so "we cannot exclude the possibility that higher jerk levels aggravate
> discomfort". The **ISO 2631 vibration model performed poorly** here precisely because "discomfort
> was found to correlate strongly with peak acceleration, [and] the frequency weightings distort
> this relation" (model output correlated only r = 0.49 with acceleration); the shock model did
> better (median adjusted R² = 0.61, predictions correlating r = 0.93 with acceleration).
> *Conditions:* full text read. **23 participants in a moving-base driving simulator** — real
> physical acceleration, full vestibular stimulation, seated with seat support. Sinusoidal and
> triangular acceleration pulses, peak acceleration 0.4–2 m/s², peak jerk 0.5–15 m/s³. The measure
> is **discomfort (verbal qualifiers and magnitude estimates) on single short pulses — not motion
> sickness**, which is explicitly a different, low-frequency, long-duration phenomenon in the same
> paper's introduction. Transfer to a rendered proxy is not available; what transfers is that when
> someone finally varied jerk and acceleration independently, jerk did not behave the way the
> intuition says.

> **Ramaseri Chandra, Reza & Pothana, "Exploring the Feasibility of Head-Tracking Data for
> Cybersickness Prediction in Virtual Reality"**, Electronics 14(3), 502 (2025).
> <https://colab.ws/articles/10.3390%2Felectronics14030502> (accessed 2026-09-15).
> *Proposes:* predicting cybersickness from head-tracking kinematics alone, using linear and
> angular velocity, acceleration **and jerk** as features.
> *Claims:* gradient boosting predicted questionnaire cybersickness scores on held-out data with
> normalised differences under 3.08%.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror (MDPI returns HTTP 403 to automated
> fetches). **12 participants**, Oculus Quest 2, questionnaire outcome. Decisive distinction for
> this project: the jerk here is the jerk of the **user's own head**, a measure of what the person
> did, and it enters as one of six correlated features in a black-box model with no per-feature
> attribution in the abstract. It is not the jerk of a displayed object, and it is not evidence that
> jerk causes anything.

### What this adds up to

Prior work supplies, for the physical case, a validated dose model that is **band-weighted
acceleration accumulated over time**. It supplies, for the physical case, one controlled test of
jerk against discomfort whose sign came out negative. It supplies, for the visual case, a body of
theory (§4) in which sickness routes through the vestibular estimate of the vertical and plausibly
requires vection — neither of which a small rendered arm viewed by a still, seated operator obviously
produces. It supplies **nothing** on the jerk of a displayed object as a sickness predictor.

One arithmetic observation, offered as context rather than as a result: the provocative band those
dose models weight is 0.1–0.5 Hz, peaking near 0.2 Hz with steep roll-off above. A correction
spread over a 100 ms budget has essentially all of its energy one to two orders of magnitude above
that band. Whatever a reconciler's displayed-jerk percentile is measuring, the physical-motion dose
literature's weighting function would attenuate that frequency content to near nothing. Establishing
whether that matters is a human-subjects question this project has no plan to answer, and saying so
is more useful than a ranking that assumes the answer.

---

## 6. (b) What positional discrepancy in a rendered proxy is perceptible?

`Teleop.Eval` hardcodes `convergencePositionToleranceMeters: 0.005f` (5 mm) and
`convergenceOrientationToleranceRadians: 0.017f` (~1°) — in `Sweep/SweepCommand.cs`,
`MoveArm/MoveArmCommand.cs` and `ClockSyncCheck/ClockSyncCheckCommand.cs` — and `docs/metrics.md`
§5 defines both `time_to_convergence_ms` and the derived correction rate against "a stated
perceptual threshold". No source is attached to it anywhere.

**The established thresholds are far larger than 5 mm, and they are not a single number.** The
closest measured analogue — a virtual *hand*, which has the strongest possible proprioceptive
referent — gives ~31 mm under the most conservative conditions anyone has run, and ranges to
~190 mm when attention is elsewhere and the offset grows gradually. A rendered remote robot arm has
no proprioceptive referent at all, so if anything the hand numbers are a *lower* bound on what is
detectable about a proxy. The second finding, from redirected walking, is that the *method* of
estimating a detection threshold moves it by a factor of 3.4 on the same phenomenon — so a single
hardcoded constant is the wrong shape of answer regardless of its value.

> **Zenner & Krüger, "Estimating Detection Thresholds for Desktop-Scale Hand Redirection in Virtual
> Reality"**, IEEE VR 2019 (DOI 10.1109/VR.2019.8798143), authors' version.
> <https://umtl.cs.uni-saarland.de/paper_preprints/zenner-krueger-hand-redirection-thresholds-vr-19-pre-print.pdf>
> (accessed 2026-09-15).
> *Proposes:* lower-bound detection thresholds for three hand-redirection dimensions — horizontal
> warp, vertical warp, and gain on the reach.
> *Claims:* the virtual hand can be displaced **up to 4.5° horizontally or vertically** without
> reliable detection (a ~9° undetectable band), and reach gains between **g = 0.88 and g = 1.07**
> go unnoticed (grasping up to 13.75% further or 6.18% less far). The paper converts its own angular
> figure: "Assuming the distance of 40 cm from origin to target tested in our experiment, our
> estimation of 4.5° yields similar thresholds of **≈ 3.1 cm**." It reports comparison figures from
> prior work it discusses: Burns et al. ≈19.1° / ≈19 cm for *gradually growing* offsets inside a
> game; Lee et al. ≈5.2 cm JND when the real and displaced fingertip were rendered **simultaneously**
> as spheres; Abtahi & Follmer ≈49.5° horizontal remapping when **continuous fingertip haptic
> feedback** was present. Conclusion in the authors' words: "human hand-eye coordination is so good
> that even small discrepancies can be detected when the assessment of the hand movement only relies
> on visual feedback".
> *Conditions:* full text read. **12 participants** (6f/6m, mean 28, range 20–61), HTC Vive, **seated
> on a chair**, hand tracked, a finger splint preventing index-finger movement relative to the hand,
> reaching to a green sphere 40 cm from a start position 30 cm below and 30 cm in front of the head.
> 2AFC (pseudo-2AFC) with **fixed** angular offsets, method of constant stimuli, in three scenarios:
> no distraction, an auditory distractor, and a visual 4-digit-readout distractor. Only **one** hand
> was rendered — a virtual human hand model, no real-hand reference. The distraction manipulations
> did *not* substantially raise thresholds, which the authors attribute to all their scenarios being
> conservative. The 5.2 cm and 49.5° figures are **reported by this paper about other work**, not
> verified here from the originals — treat them as secondary.

> **Burns, Razzaque, Panter, Whitton, McCallus & Brooks, "The Hand Is More Easily Fooled than the
> Eye: Users Are More Sensitive to Visual Interpenetration than to Visual-Proprioceptive
> Discrepancy"**, Presence: Teleoperators and Virtual Environments 15(1), 1–15 (2006).
> <https://colab.ws/articles/10.1162%2Fpres.2006.15.1.1> (accessed 2026-09-15).
> *Proposes:* comparing two detection thresholds against each other — visual interpenetration of an
> avatar into an object, versus visual-proprioceptive discrepancy between real and rendered hand.
> *Claims:* "users are much less sensitive to visual-proprioceptive discrepancy than to visual
> interpenetration", i.e. people notice a hand going *through* a surface long before they notice
> their hand being in the wrong place.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror (MIT Press returns HTTP 403). The
> numeric threshold (~19.1° / ~19 cm), the gradual method-of-limits procedure and the game context
> come from Zenner & Krüger's description of this paper, above, not from the original text.
> The qualitative finding is what matters here and it is robust: **geometric plausibility of the
> rendered scene dominates positional accuracy of the rendered limb.** A proxy arm that visibly
> intersects a table is a bigger perceptual event than one that is 2 cm out of place.

> **Grechkin, Thomas, Azmandian, Bolas & Suma, "Revisiting detection thresholds for redirected
> walking: combining translation and curvature gains"**, Proc. ACM SAP '16.
> <https://colab.ws/articles/10.1145%2F2931002.2931018> (accessed 2026-09-15).
> *Proposes:* re-measuring curvature-gain detection thresholds, with and without simultaneous
> translation gain, under two different psychophysical estimation procedures.
> *Claims:* no evidence that curvature-gain thresholds are affected by the presence of translation
> gain — but the headline is methodological: "users can be redirected on a circular arc with radius
> of either **11.6 m or 6.4 m depending on the estimation method** vs. the previously reported value
> of 22 m", and "detection threshold estimates vary significantly with the estimation method".
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror (`dl.acm.org` returns HTTP 403);
> participants, HMD and tracking volume not read. Redirected *walking*, so the stimulus is
> locomotion gain, not object position. Its value here is the warning, which generalises: the same
> perceptual quantity, re-estimated by a different standard procedure in the same subfield, moved by
> **3.4×**. Any single hardcoded perceptual constant — 5 mm included — inherits that uncertainty
> before any question of transfer is even asked.

> Also relevant and recorded in §2: **Jerald (2010)** measured scene-motion thresholds, the
> head-referenced sibling of this question, and found them dependent on head-yaw velocity and
> **asymmetric with direction** (larger when the scene moves *with* the head). That a detection
> threshold for a displayed offset depends on the observer's own concurrent motion, and on the sign
> of the offset, is the structural point this project's single scalar tolerance cannot express.

---

## 7. (c) Correction duration versus correction magnitude

The project spreads corrections over a **100 ms convergence budget** chosen for frame arithmetic.
The question asked here is not whether 100 ms is right but what durations prior work finds
effective. The honest summary is that the literature does **not** support "spread it out" as a
general principle. It supports something narrower and, for this project, more interesting:

- **Apply the correction while the visual system is not looking.** The best-supported and
  largest-magnitude concealment results come from exploiting blinks, saccades and scene changes —
  perceptual gaps of roughly 100–300 ms during which multi-centimetre repositioning goes unnoticed.
  This is a *timing* strategy, not a *duration* strategy: the correction is effectively
  instantaneous, and it is the observer's availability that is being scheduled.
- **Gradual versus fixed offsets, on the same phenomenon, differ by roughly 6×** in the hand case
  (Burns ≈19 cm gradual-growing versus Zenner ≈3.1 cm fixed, §6), which is evidence *for* spreading
  — but the two studies differ in procedure, attention and stimulus type as well as gradualness, and
  Zenner & Krüger attribute the gap to all of those together. Nobody appears to have varied
  correction *duration* alone, with magnitude held fixed, and measured detectability. That is the
  experiment this project's axis actually needs and it does not exist in what was searched.
- Meanwhile the one physical-motion study that varied pulse duration cleanly (de Winkel et al., §5)
  found **shorter** pulses more comfortable at matched peak acceleration — the opposite direction to
  the spreading intuition, in a different modality.

> **Langbehn, Steinicke, Lappe, Welch & Bruder, "In the blink of an eye: leveraging blink-induced
> suppression for imperceptible position and orientation redirection in virtual reality"**, ACM
> Transactions on Graphics 37(4), Article 66 (SIGGRAPH 2018).
> <https://history.siggraph.org/learning/in-the-blink-of-an-eye-leveraging-blink-induced-suppression-for-imperceptible-position-and-orientation-redirection-in-virtual-reality-by-langbehn-steinicke-lappe-welch-and-bruder/>
> (accessed 2026-09-15).
> *Proposes:* applying camera position and orientation corrections **during eye blinks**, when the
> observer is functionally blind, and accumulating them blink by blink.
> *Claims:* users can be translated approximately **4–9 cm** and rotated approximately **2–5°** per
> blink without noticing; combined with conventional redirected walking this improved performance by
> roughly 50%.
> *Conditions:* **abstract/summary level**, via the SIGGRAPH history record (the ACM version returns
> HTTP 403). Detection-threshold studies with commercial eye-tracking HMDs; participant counts,
> exact procedure and the blink-detection latency budget were **not** read. Note what the magnitudes
> are: 4–9 cm is 8–18× this project's 5 mm tolerance, achieved by an *instantaneous* jump placed
> inside a perceptual gap. If concealment-by-timing is available, it dominates concealment-by-easing
> by an order of magnitude — but it needs eye tracking and a blink detector, which lands squarely in
> `unity/Bridge/` and is therefore **blocked on human review** for this project.

> **Sun, Patney, Wei, Shapira, Lu, Asente, Zhu, McGuire, Luebke & Kaufman, "Towards Virtual Reality
> Infinite Walking: Dynamic Saccadic Redirection"**, ACM Transactions on Graphics 37(4) (2018).
> <https://researchconnect.stonybrook.edu/en/publications/towards-virtual-reality-infinite-walking-dynamic-saccadic-redirec/>
> (accessed 2026-09-15).
> *Proposes:* the same idea keyed to **saccadic suppression** rather than blinks, with real-time GPU
> path planning and "subtle gaze direction" methods to *induce* the saccades that create the
> concealment windows.
> *Claims:* "saccades can significantly increase the rotation gains during redirection without
> introducing visual distortions or simulator sickness."
> *Conditions:* **abstract only**, via the institutional repository record; the abstract states no
> numeric gains and the user-study details were not read. Head- and eye-tracking HMD, walking
> locomotion. The transferable idea for a reconciler is the *active* one: the system does not merely
> wait for a concealment window, it arranges for one.

> **Suma, Clark, Krum, Finkelstein, Bolas & Warte, "Leveraging change blindness for redirection in
> virtual environments"**, IEEE VR 2011, 159–166.
> <https://colab.ws/articles/10.1109%2FVR.2011.5759455> (accessed 2026-09-15).
> *Proposes:* changing the environment itself outside the user's attention rather than manipulating
> the mapping between physical and virtual motion, so no visual-vestibular conflict is introduced at
> all.
> *Claims:* across two user studies, **only 1 of 77 participants** definitively noticed that a scene
> change had occurred.
> *Conditions:* **abstract only**, via the `colab.ws` DOI mirror; HMD, tracking volume and the exact
> changes made were not read. Architectural-scale changes to a virtual building during exploration —
> a very different stimulus from a centimetre-scale offset on a tracked object, and the changes were
> to parts of the scene the user was not looking at. The general lesson is the one this project's
> axis has no way to exploit today: **attention allocation dominates the geometry.**

### Where that leaves the 100 ms budget

Prior work gives no basis for preferring 100 ms to 50 ms or 300 ms, because the controlled
duration-versus-detectability experiment has not been run on this stimulus. It does give two
concrete alternatives that were not on the axis's list: schedule corrections into blinks/saccades
(large magnitudes, needs eye tracking, blocked on human review here), and treat the operator's
attention rather than the correction's smoothness as the variable. It also gives a caution about
the reverse direction, from §5: at matched peak acceleration, longer physical pulses were rated
*less* comfortable, so "slower is gentler" is not a safe prior even in the domain where the
intuition originated.
