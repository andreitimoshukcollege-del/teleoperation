# Adaptive playout and delay-based control

The neighbourhood this project's **Buffering** and **Transport** axes sit in. Scope: how a receiver
decides *when* to play out a sample it has already received, and how a sender decides *how fast* to
send when the signal it controls on is delay rather than loss.

## Why this field and not the bulk-transfer one

The traffic this project carries is small, individually timestamped pose datagrams at 90-100 Hz over
UDP, where **a sample that arrives after its playout instant is useless rather than merely delayed**.
That is structurally the VoIP/RTP receiver problem: a fixed packetisation cadence, a per-packet
deadline, and a late packet that counts as a loss. It is *not* the bulk-transfer problem, where a
late byte is still a useful byte and the only cost is completion time.

The practical consequence for reading this list: the 1994-2010 packet-audio playout literature is
the closest match to the Buffering axis by problem structure, even though its packet rate (50 Hz,
20 ms packetisation) is roughly half this project's and its payloads are 20-160 bytes of codec
frames rather than a pose. The congestion-control literature is a *weaker* match — almost all of it
assumes a sender whose rate is elastic, which a fixed-cadence pose stream is not — but it is where
the delay-signal estimation machinery (base-delay filtering, one-way delay gradients, noise
rejection) was actually worked out, and that machinery is reusable independently of the rate
control it was built to drive.

Throughout, entries are marked with how well they transfer:

- **Transfers directly** — same problem structure (per-packet deadline, receiver-side scheduling).
- **Transfers as machinery** — the estimator or the measurement definition is reusable; the control
  loop it drives is not.
- **Does not transfer** — included to prevent reinvention or to mark a dead end.

Per `docs/literature/CLAUDE.md`, nothing here is evidence about this system. Prior work reports X
under conditions Y; only a sweep in `results/` says anything about this codebase.

## Contents

1. [The delay-versus-loss tradeoff curve — the central object](#1-the-delay-versus-loss-tradeoff-curve--the-central-object)
2. [Family A: window/order-statistic playout adjustment (the RFC 3550 line)](#2-family-a-windoworder-statistic-playout-adjustment-the-rfc-3550-line)
3. [Family B: distribution and histogram trackers (percentile targeting)](#3-family-b-distribution-and-histogram-trackers-percentile-targeting)
4. [Family C: forecasting playout (ARMA/GARCH, NLMS and other predictors)](#4-family-c-forecasting-playout-armagarch-nlms-and-other-predictors)
5. [Family D: quality-model-driven playout](#5-family-d-quality-model-driven-playout)
6. [Family E: NetEQ as shipped](#6-family-e-neteq-as-shipped)
7. [How the field measures burst loss, discard and reordering](#7-how-the-field-measures-burst-loss-discard-and-reordering)
8. [Family F: delay-based congestion control](#8-family-f-delay-based-congestion-control)
9. [Family G: real-time media transports and what they assume about late data](#9-family-g-real-time-media-transports-and-what-they-assume-about-late-data)
10. [Cloud gaming and XR streaming latency adaptation](#10-cloud-gaming-and-xr-streaming-latency-adaptation)
11. [The question this list was built to answer](#11-the-question-this-list-was-built-to-answer)
12. [Operating-point convention: target late-loss rate versus delay tail](#12-operating-point-convention-target-late-loss-rate-versus-delay-tail)
13. [Searches that found nothing](#13-searches-that-found-nothing)
14. [Sources that could not be retrieved](#14-sources-that-could-not-be-retrieved)
15. [What this file argues should change elsewhere](#15-what-this-file-argues-should-change-elsewhere)

## 1. The delay-versus-loss tradeoff curve — the central object

Every playout scheme in this field is a point-chooser on one curve: **as you add buffering delay,
the fraction of samples that arrive after their playout instant falls**. The field's name for that
fraction varies — *late loss*, *lateness*, *discard rate* — and the distinction between it and
network loss is the thing the measurement standards below spend most of their text on. A scheme is
not "better" in the scalar sense; it either reaches a lower late-loss at equal delay, or it tracks
the curve's *movement* faster when the path changes. Almost every disagreement in the playout
literature is really a disagreement about which of those two is being measured.

Three things are worth taking from this framing before reading any individual source.

**The curve has a computable optimum, and it is offline.** Moon, Kurose and Towsley's contribution
was less an algorithm than a *bound*: given a recorded delay trace, you can compute the minimum
average playout delay achievable by any algorithm for a given number of late packets, and therefore
say how far a real adaptive algorithm is from the best possible on that trace. That is the shape of
comparison this field settled on — algorithm versus offline-optimal on a fixed trace — and it maps
cleanly onto a deterministic offline replay harness. **What the bound showed, secondhand from
Kansal & Karandikar (§3), who were reading it:** the algorithms of the day sat close to the optimum
at loose loss targets and visibly short of it **in the low-loss region, under 2%**. So the headroom
this framing exposes is not uniform along the curve — it is concentrated exactly where a
latency-sensitive application would want to operate.

**How the curve is actually produced, now that the primaries have been read.** Every paper in
Families A-D generates its curve the same way: **fix the trace, sweep the algorithm's own single
control parameter, and plot the locus of (delay, loss) points that results.** The parameter differs
— Ramjee et al. sweep a maximum buffer size in bytes, Liang et al. sweep a user-specified target
loss rate, Zhang et al. sweep a desired PLR — but the construction is identical, and it is what
makes two algorithms with different knobs comparable at all. The delay axis is almost always a
**mean**, not a percentile, and it is almost always taken **over the played-out packets only**
(§12). Two consequences for reproducing this shape of comparison: a candidate must expose exactly
one monotone knob for the sweep to be meaningful, and the delay statistic is computed over a sample
set that shrinks as you move toward the low-delay end of the curve.

**Shipping code writes the tradeoff down as an explicit cost function.** WebRTC's reorder optimizer
minimises `delay_ms + 100 * ms_per_loss_percent * loss_probability` over candidate delays (see
Family E). That is a literal scalarisation of the curve with a tunable exchange rate between
milliseconds and percentage points of loss. It is the most concrete statement in this whole reading
list of what the field thinks the tradeoff *is*.

**Late loss is not interchangeable with network loss, and the standards bodies insist on it.** See
§7. Any comparison that folds discards into a single "loss" number has destroyed the only signal
that distinguishes a buffer that is too short from a path that is dropping packets.

*Transfers directly.* The structure is identical for a pose stream; only the concealment options
differ (a voice codec can time-scale or interpolate a frame; this project's equivalent question is
what the reconciler does with a missing sample, which is a different axis).

## 2. Family A: window/order-statistic playout adjustment (the RFC 3550 line)

The oldest family. Keep a running estimate of mean delay and of delay variation, set playout to
`mean + k * variation`, and adjust only at talkspurt boundaries so the adjustment is inaudible. The
signal adapted on is an **exponentially weighted inter-arrival jitter estimate** — the cheapest
possible statistic, one multiply-add per packet, no history buffer. Everything later in this file is
a reaction to this family's main weakness: an EWMA of variation reacts slowly to a delay *spike*
and then decays slowly afterwards, so it overshoots on both edges.

> **Schulzrinne, Casner, Frederick & Jacobson, "RTP: A Transport Protocol for Real-Time
> Applications"**, RFC 3550 (2003). <https://www.rfc-editor.org/rfc/rfc3550.html>
> (accessed 2026-09-15).
> *Proposes:* per-packet media timestamps from a monotonic clock, and the interarrival jitter
> estimator `J = J + (|D(i-1,i)| - J)/16`, an exponential average of the difference of relative
> transit times with a fixed gain of 1/16.
> *Claims:* the estimator is a noise-reducing, implementation-independent jitter measure suitable
> for reporting between endpoints. RTP deliberately does **not** specify receiver buffering; it
> states it does not guarantee quality of service and leaves playout to the application.
> *Conditions:* a specification, not a measurement — no evaluation conditions exist. The 1/16 gain
> is asserted, not derived from a trace.
> *Transfers as machinery.* The gain of 1/16 is an arbitrary constant every later paper re-tunes,
> and the "jitter" it reports is a smoothed absolute first difference, which is **not** a percentile
> of the delay distribution and should never be read as one.

> **Ramjee, Kurose, Towsley & Schulzrinne, "Adaptive playout mechanisms for packetized audio
> applications in wide-area networks"**, IEEE INFOCOM '94.
> <https://www.microsoft.com/en-us/research/publication/adaptive-playout-mechanisms-packetized-audio-applications-wide-area-networks/>
> (accessed 2026-09-15).
> **Recovered 2026-09-16** via a co-author-hosted copy — Henning Schulzrinne's Columbia paper
> archive at <http://www.cs.columbia.edu/~hgs/papers/Ramj94_Adaptive.pdf> (accessed 2026-09-16),
> linked from the Microsoft Research landing page above. Text extracted locally and the full paper
> read. Everything below is from the paper.
> *Proposes:* four playout adjustment algorithms, the fourth of which explicitly **detects delay
> spikes** and switches into a spike-following mode rather than continuing to filter. This is the
> origin of "spike detection" as a named technique. All four share the same skeleton — the
> "absolute timing method": the first packet of a talkspurt is played at
> `p_i = t_i + d̂_i + 4·v̂_i`, and every later packet in that talkspurt at `p_j = p_i + t_j - t_i`,
> so playout is constant within a talkspurt and adjusted only at boundaries. **The four algorithms
> differ only in how `d̂_i` is computed; `v̂` is identical throughout**, an EWMA of `|d̂_i - n_i|`.
> Algorithm 1 is the RFC 793 / Van Jacobson linear recursive filter with `α = 0.998002` (the value
> was taken from the existing NeVoT 1.4 implementation, not derived). Algorithm 2 is Mills'
> asymmetric variant — two weighting factors, `0.75` for increasing delay trends and `0.998002` for
> decreasing. Algorithm 3, then shipping in NeVoT 1.6, is `d̂_i = min{n_j}` over the **previous
> talkspurt**. Algorithm 4 detects a spike when `|n_i - n_(i-1)|` exceeds a threshold, then
> "follows" the spike instead of filtering, and detects the spike's end via a slope variable
> derived from `|2n_i - n_(i-1) - n_(i-2)|`. The authors state the spike constants "were chosen
> based on our examination of the behavior of a number of different spikes from a large set of
> audio traces (not reported here)" — i.e. hand-tuned on unpublished data.
> *Claims:* the spike-adapting algorithm achieves a lower rate of lost packets both at a given
> average playout delay and at a given maximum buffer size — but the paper is notably more hedged
> than its abstract. Algorithm 4 is "slightly better" than 1 and 3 on most traces, and much better
> only on the high-jitter ones (on trace 7, ~**300 ms** lower average playout delay at the same
> loss rate). On the two low-jitter traces, **Algorithms 3 and 4 are slightly *worse* than
> Algorithm 1**, which the authors dismiss as under 10 ms. Algorithm 2 "performs quite poorly",
> and the authors draw the moral explicitly: "what is good for one domain need not necessarily be
> good in another domain" — a TCP retransmit-timer estimator is not a playout estimator.
> Algorithm 3 is worse than Algorithm 2 on trace 7, so "taking the minimum of delay values of
> packets in the previous talkspurt may not be a robust delay estimate".
> *Conditions:* **read in full.** Traces collected with **NeVoT**, vat audio packet format,
> unicast UDP, Sun SPARCstation hosts, **160 bytes of audio roughly every 20 ms**. Timestamps were
> logged at **both** source and receiver, so these are genuine one-way delay traces. Four sites:
> **UMass Amherst, INRIA France, UC Irvine, Osaka University**, with the paths characterised by hop
> count and loss: UMASS-OSAKA 26 hops and "very lossy" at **20-30% packet loss**; UMASS-INRIA
> 27 hops at **2-10%**; UMASS-UCI 18 hops at **1-4%**. Hosts "relatively idle", point-to-point.
> Evaluation is a **trace-driven simulator** running all four algorithms over the identical
> received-packet trace. Loss counts **both** late arrivals **and** packets arriving too early for
> a finite circular buffer of `k` packets — i.e. underflow and overflow discards folded into one
> number, which is precisely the conflation RFC 3611 later separates (§7).
> *How the operating point is chosen and the tradeoff reported — and it is not what §12's later
> sources do.* There is no target loss rate anywhere. The curve is **parameterised by maximum
> buffer size**, swept from **160 bytes to 4 KB**, and each (algorithm, buffer size, trace) triple
> yields one point of (average playout delay, % packet loss). "Average playout delay" is defined
> as the mean `d_i` of successfully played-out packets **minus the smallest `d_i` in the entire
> trace**, so the reported quantity is delay above the trace's propagation floor, not absolute
> delay. A second plot gives loss directly against maximum buffer size. The "range of loss rates
> of interest" is stated as **under 10%**, justified in the introduction by "packet loss rates of
> between 1 and 10% can be tolerated, depending on the manner in which voice is coded and missing
> packets are masked".
> *Transfers directly*, with the caveat that spike detection assumes spikes are *rare and
> large* — a router-level transient. A bursty-loss impairment profile produces something
> structurally different (missing samples, not late ones), and this family does not address it.
> Note also that the paper's own results are the first data point in what §4 records as a
> long-running doubt about spike detection: it helps a lot where jitter is spiky, and costs a
> little where it is not.

> **Moon, Kurose & Towsley, "Packet audio playout delay adjustment: performance bounds and
> algorithms"**, Multimedia Systems 6(1):17-28 (1998). DOI `10.1007/s005300050073`; bibliographic
> record verified at <https://api.crossref.org/works/10.1007/s005300050073> and at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1007/s005300050073?fields=title,abstract,year,venue,authors,externalIds>
> (both accessed 2026-09-15).
> *Proposes:* (i) an algorithm computing upper and lower bounds on the minimum achievable average
> playout delay for a given number of late packets on a recorded trace, i.e. an offline optimum to
> measure real algorithms against; (ii) an adaptive algorithm that tracks a statistic of recently
> received packet delays rather than an EWMA of variation.
> *Claims:* the bounds are tight over the loss and delay range of interest, and the delay-tracking
> algorithm outperforms the EWMA family.
> *Conditions:* **still not read directly, after a second retrieval attempt on 2026-09-16.** Every
> route tried is recorded in §14; the decisive one is that CiteSeerX now 301-redirects its whole
> corpus to `web.archive.org`, and every Internet Archive host — `web.archive.org`,
> `scholar.archive.org`, `api.fatcat.wiki` — is unreachable from this environment (the first is
> refused by the fetch tool, the latter two by the network). The paper has **no open-access
> location at all** per OpenAlex and Semantic Scholar (`oa_status: closed`), and the two
> repositories that hold a record — ScholarWorks@UMass and UMass's own technical-report server —
> return HTTP 403 or do not hold this report. Bibliographic detail is confirmed independently from
> Sue Moon's own CV at <https://an.kaist.ac.kr/~sbmoon/resume.pdf> (accessed 2026-09-16):
> "ACM/Springer Multimedia Systems, Vol. 6, pp. 17-28, Jan. 1998".
> **What is now known about it, secondhand from two sources in this file that *were* read in
> full.** This is not a substitute for reading it, and nothing here is a performance number from
> the paper, but it is far better than the blank this entry used to be:
> - **The algorithm.** Kansal & Karandikar (§3), who implemented against it, describe it as a
>   histogram method: "a histogram of the previous `L` packets is plotted and a value higher than
>   the delay of a chosen percentage of packets is used as the predicted value for the next
>   talkspurt", with **`L = 10,000`** in Moon et al. So the operating point is set by a **chosen
>   percentage of the delay distribution** — this is the source of Family B's percentile framing,
>   and the window is enormous by the standards of anything else here.
> - **The bound, and what it showed.** Kansal & Karandikar: the bound is "calculated in [Moon et
>   al.] offline, after recording its packet arrival times", and the simulations "reveal that there
>   is some gap between the playout delay achieved by the existing algorithms and the minimum
>   bound, **especially in regions of low loss, under 2%**". That is the shape of the result: the
>   algorithms of the day were close to optimal at loose loss targets and visibly short of it at
>   tight ones.
> - **The traces**, reproduced as a table by Miranda-Campos & Ramos (§4) and repeated there: six
>   NeVoT traces, 160-byte packets at ~20 ms, captured 1993 and 1995 between UMass, GMD Fokus
>   (Berlin), INRIA, UC Irvine and Osaka University, 580-1348 s each, 23k-57k packets each, with
>   both sender and receiver timestamps. Clock synchronisation is sidestepped by using only the
>   *variable* portion of delay, i.e. each packet's delay minus the trace minimum.
> - **The metric definitions**, which Miranda-Campos & Ramos adopt verbatim "as in (Moon et al.,
>   1998)": average playout delay is averaged **over played-out packets only**, and loss percentage
>   is `(sent - played)/sent`.
> Do not repeat any performance number from this source without reading it.
> *Transfers directly* on structure (the bound-versus-algorithm comparison is exactly the shape an
> offline replay harness can reproduce), but no number from it is usable here.

> **Sreenan, Chen, Agrawal & Narendran, "Delay reduction techniques for playout buffering"**
> ("Concord"), IEEE Transactions on Multimedia 2(2):88-100 (2000). DOI `10.1109/6046.845013`;
> record verified at
> <https://api.crossref.org/works?query.bibliographic=Delay+reduction+techniques+for+playout+buffering+Sreenan+Chen+Agrawal+Narendran&rows=2&select=title,author,container-title,volume,issue,page,issued,DOI>
> (accessed 2026-09-15).
> *Proposes:* Concord — keep a delay history, age it, make short-term predictions, and expose the
> delay/lateness exchange rate as an **application-set parameter** rather than an algorithm
> constant. The application declares how much lateness it will tolerate; the algorithm finds the
> delay.
> *Claims:* substantial reductions in buffering delay and in delay variation at lateness under 1%.
> *Conditions:* **not read** — IEEE Xplore <https://ieeexplore.ieee.org/document/845013/> returned
> an empty body (accessed 2026-09-15). Evaluation is described
> elsewhere as using Internet traffic traces; neither the traces nor the paths were verified here.
> *Transfers directly* as a design pattern: the "application declares a target lateness, the policy
> solves for delay" interface is the same interface WebRTC's reorder optimizer exposes 20 years
> later via `ms_per_loss_percent`.

## 3. Family B: distribution and histogram trackers (percentile targeting)

The reaction to Family A. Instead of a mean and a variation estimate, keep an approximation of the
**delay distribution** and set playout to a chosen quantile of it. The signal adapted on is a
percentile of the delay distribution, and the tuning knob becomes explicitly "what fraction of
packets am I willing to lose to lateness" rather than "how many standard deviations". This is the
family that won in practice: it is what shipped in WebRTC (§6).

The cost is memory and adaptation speed — a histogram with a forget factor adapts on the timescale
of the forget factor, which is why the shipping implementation adds a separate faster path for
early adaptation.

> **Liang, Färber & Girod, "Adaptive playout scheduling and loss concealment for voice
> communication over IP networks"**, IEEE Transactions on Multimedia 5(4):482-493 (2003).
> DOI `10.1109/TMM.2003.819095`; record verified at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/tmm.2003.819095?fields=title,abstract,year,venue,authors>
> (accessed 2026-09-15). Author-hosted PDF at
> <https://web.stanford.edu/~bgirod/pdfs/LiangMM2003.pdf> (accessed 2026-09-15) — the file
> resolves and is 595 KB, but its text layer was not extractable by the fetch tool.
> **Recovered 2026-09-16** — the Stanford PDF (re-accessed 2026-09-16) had its text layer
> extracted locally and the full paper read. Everything below is from the paper. (The author-hosted copy is the published IEEE
> version; its printed pagination runs 532-544, which differs from Crossref's 482-493 — an IEEE
> metadata artefact, not a different document.)
> *Proposes:* receiver-side playout scheduling driven by order statistics of recent delays, plus
> **per-packet** (not per-talkspurt) time-scale modification using WSOLA, so playout delay can be
> changed continuously instead of only at silence boundaries. Concretely: keep the delays of a
> sliding window of the past `W` packets, sort them, and pick the playout deadline from the order
> statistic corresponding to a **user-specified loss rate** `e`. Two details matter. First, the
> empirical order statistics are **extended** with an estimated minimum delay and an estimated
> maximum delay `d_max = d_(W) + βσ` before the quantile is taken, because "due to the heavy-tailed
> nature of network delay, the maximum possible value of the delay cannot be determined from a
> limited sample space" — the raw sample maximum is "too optimistic", so the achievable target
> loss rate would otherwise be floored at roughly `1/(W+1)`. The deadline is then interpolated
> between adjacent order statistics. Second, a **spike mode**: when the present delay exceeds the
> previous one by more than a threshold, discard the first spike packet, set the estimate to the
> last spike delay, and **freeze** the order statistics; on recovery, resume from the saved
> pre-spike state rather than from a filter polluted by the spike.
> *Claims:* at a **5% target late-loss rate**, average buffering delay lower than the
> talkspurt-boundary baselines by 40.0 / 20.7 / 4.3 / 28.0 ms (versus Algorithm 1) and by
> 31.5 / 11.8 / 4.4 / 20.0 ms (versus Algorithm 2) on Traces 1-4. Conversely at a fixed 40 ms
> buffering delay on Trace 1, total loss rate is "more than 10% lower". Separately and more
> interestingly, **burst loss rate falls from 12% to 1% at 40 ms buffering delay on Trace 1**, and
> is 3.9 points lower at 10 ms on Trace 3 — the trace with the mildest jitter and the smallest
> delay/loss gain. Subjective listening tests give typical gains of one MOS point on a 5-point
> scale.
> *Conditions:* **read in full.** Four one-way delay traces of UDP packets, local host at Stanford,
> four geographic remote sites (Trace 3 is Stanford-MIT, described as high-bandwidth with the
> mildest delay variation; the full site table is a figure and was not extractable). **160-byte
> payloads, 20 ms G.711 at 8-bit quantisation, 180 s per trace, 9000 packets per trace**, clocks
> NTP-synchronised. Maximum delay jitter (max minus min delay within a trace) ranges **39-130 ms**
> across the four traces. Evaluation is **offline trace-driven replay**: delay traces and voice
> packets are read from recorded files and each algorithm is executed over the identical input.
> The window size `W` was **tuned per algorithm on the same collected trace data** used for the
> comparison — worth noting, since it means the reported gaps include per-algorithm tuning on the
> evaluation set.
> *How the operating point is chosen and the tradeoff reported — the part §12 needed.* The paper
> does **not** report a point. It reports a **continuous curve** per algorithm, obtained by
> "varying the control parameters of each particular algorithm, e.g., the user-specified loss rate
> determining the playout deadline in Algorithm 3", and states plainly that "the variation of the
> control parameter therefore illustrates the achievable tradeoff". Both axes are named: loss rate
> against **average** buffering delay. There is no delay percentile anywhere in the evaluation.
> The link loss rate is drawn as a horizontal dashed line that lower-bounds total loss, so late
> loss is read off as the vertical distance above it. **A second curve, burst loss rate against
> average buffering delay, is reported alongside the first** — burst loss being defined as the rate
> of packets belonging to a pair of two consecutive losses, reported separately "because they are
> more difficult to conceal and impair sound quality more severely".
> *Transfers as machinery, with a caveat this project should notice:* continuous playout-rate
> adjustment is exactly what a pose stream *can* do more easily than audio — there is no waveform
> to distort, only a resampling of a continuous signal — but the "cost" of doing it is a correction
> the operator sees, which is this project's correction-cost metric and not a concept this
> literature has.

> **Kansal & Karandikar, "Adaptive delay estimation for low jitter audio over Internet"**,
> IEEE GLOBECOM 2001.
> <https://www.microsoft.com/en-us/research/wp-content/uploads/2001/11/kansal_globecom01.pdf>
> (accessed 2026-09-15).
> **Recovered 2026-09-16** — text extracted locally from the same URL (re-accessed 2026-09-16) and
> the full paper read.
> *Proposes:* three things. (i) An **α-adaptive** playout algorithm: rather than fix the EWMA gain
> `α` at NeVoT's 0.998002, run two estimators in parallel with gains `α` and `α ± 0.0001`, measure
> the loss each would have produced over the last `L` talkspurts, and step `α` toward whichever
> did better — an online hill-climb on the gain, re-evaluated every `L`-th talkspurt (`L = 5`).
> (ii) A separate **jitter control procedure** that bounds how much the playout point may move in
> a silence period, so silence-period stretching does not itself become audible. (iii) Modifications
> to DeLeon & Sreenan's NLMS predictor.
> *Claims:* the α-adaptive algorithm reduces playout delay by **up to 100 ms at low packet loss**
> (and up to 200 ms in a stated region) versus the fixed-gain algorithm, "especially in low loss
> regions", and "at least as good" elsewhere; the jitter control procedure reduces playout jitter
> without a serious increase in playout delay.
> *Conditions:* **read in full.** Trace-driven simulation on **the Moon, Kurose & Towsley 1998
> traces** (the paper's reference [5]) — see the Moon entry in §2 and the trace table recorded
> under Miranda-Campos in §4. Comparison is restricted to "losses up to 5%, the acceptable limit
> for voice", attributed to Jayant's 1980 waveform-coded-speech work. Spike detection is added to
> the α-adaptive algorithm when comparing against the spike-detecting baseline, so the comparison
> is like-for-like on that feature.
> *A second, earlier negative result on the NLMS family, which §4 should be read alongside.* This
> paper independently reports that NLMS playout does **not** transfer across trace sets: the
> parameters `µ = 0.0001` and filter order `N = 18` that "worked well for traces used in
> [DeLeon & Sreenan 1999] did not work well for the traces from [Moon et al. 1998] with talkspurts,
> rendering the delay estimates useless", and with talkspurt-based adjustment NLMS is "in fact
> worse" than the plain linear recursive filter — the opposite of the published claim. Together
> with Miranda-Campos & Ramos (§4), that is **two independent failures to reproduce a
> forecasting-playout result on a different trace set**, seven years apart.
> *Transfers as machinery and as a warning.* The α hill-climb is the cheapest idea in this file for
> making a fixed-gain filter self-tuning, and it is allocation-free; the repeated cross-trace
> failure of NLMS is the strongest available argument that a fitted predictor's parameters are a
> property of the trace, not of the problem.

> **Sakir & Feldbauer, "Efficient Quality-Based Playout Buffer Algorithm"**, arXiv:0909.2816
> (2009). <https://arxiv.org/abs/0909.2816> (accessed 2026-09-15).
> *Proposes:* a closed-form playout-delay solution that minimises a sum of the ITU-T E-model's
> simplified delay impairment factor and a loss impairment term, modelling the delay distribution
> as **Pareto** to keep the solution cheap.
> *Claims:* outperforms then-current algorithms at reduced computational complexity.
> *Conditions:* **abstract only was read** (arXiv abstract page). The abstract states "simulation
> results" without naming traces, delay models, packet rates or baselines. A Pareto delay model is
> a heavy-tailed assumption — relevant to note, because a scheme whose optimum is derived under a
> heavy-tailed delay model is making a different bet than one tuned on a histogram.
> *Transfers as machinery:* the idea of choosing playout by minimising a scalar impairment function
> rather than hitting a loss target is the same move WebRTC's reorder optimizer makes.

## 4. Family C: forecasting playout (ARMA/GARCH, NLMS and other predictors)

The third reaction. Rather than tracking a statistic of *past* delays, fit a time-series model and
set playout from a **quantile of the forecast distribution** for the next packet. The signal adapted
on is a model-based forecast — usually a conditional mean plus a conditional variance — which in
principle reacts to a delay spike as it begins rather than after it has been absorbed into a filter.

This family is the most interesting one for this project and the most poorly served by retrievable
sources: the headline papers are behind Elsevier and IEEE paywalls. What *is* readable is a
replication study, and it is a negative result, which is worth more than another positive claim.

> **Zhang, Fay, Kilmartin & Moore, "A Garch-based adaptive playout delay algorithm for VoIP"**,
> Computer Networks 54(17):3108-3122 (2010). DOI `10.1016/j.comnet.2010.06.006`; record verified at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1016/j.comnet.2010.06.006?fields=title,abstract,year,venue,authors>
> (accessed 2026-09-15). Author-hosted PDF at
> <https://www.cl.cam.ac.uk/~awm22/publications/zhang2010garch.pdf> (accessed 2026-09-15) — resolves
> (890 KB) but the text layer was not extractable by the fetch tool.
> **Recovered 2026-09-16** — the Cambridge-hosted PDF (re-accessed 2026-09-16) had its text layer
> extracted locally and the full paper read. Everything below is from the paper.
> *Proposes:* an ARMA model for the conditional mean of packet delay plus a GARCH model for its
> conditional variance, with playout set from the forecast distribution; operates both
> inter-talkspurt and intra-talkspurt; a parameter estimation procedure the authors call "Direct
> GARCH" that is fitted **to hit a target packet loss rate** while minimising the probability of
> *consecutive* losses. The mechanism in full: the extra delay a packet suffers beyond the
> previous packet's jitter is assumed **Laplacian with zero mean**, the GARCH model forecasts that
> distribution's conditional variance, and the buffer `f_w` is the inverse CDF evaluated at
> `w = 1 - v`, where `v` is the desired packet loss rate. `v` is swept over **1-5%**, so `w` runs
> **0.95 to 0.99**. "Standard GARCH" fits parameters by maximum likelihood; **"Direct GARCH"
> replaces the likelihood with a cost function built directly on the quantity of interest**,
> `F = (1/v)(Σ_k k·q(k))²`, where `q(k)` is the distribution of drop inter-arrivals — so the fit
> penalises *clustering* of late losses rather than fitting the delay series well.
> *Claims:* (i) the headline result is **not** a better delay/loss curve but better *calibration*:
> "given a target Packet Loss Rate (PLR) the Direct GARCH algorithm produces parameter estimates
> which result in a PLR closer than other algorithms", with "a very stable PLR Error which is close
> to zero", and the authors call this "the main advantage of the proposed algorithm"; (ii) lower
> consecutive-loss probability than the Concord variants at both a given target PLR and a given
> buffering delay; (iii) on the delay-versus-loss tradeoff itself the margin is thin — Direct GARCH
> is "marginally offering the best performance", and the Standard/Direct GARCH and an MLP neural
> predictor "achieve very similar performance", with the clear separation being against the linear
> recursive filter, not between the non-linear schemes; (iv) best PESQ MOS of the five.
> *Conditions:* **read in full, and they are weaker than the paper's framing suggests.** Three
> traces, all from NUI Galway: to University of Tokyo (21/05/2007, 6 h 49 m), to UNSW Sydney
> (23/05/2007, 10 h 15 m), to Chengdu, China (23/05/2007, 21 h 36 m). Continuous full-duplex
> **G.729B, 20 ms packets, 80-byte payloads**, RTP over UDP, captured with a modified PJSIP.
> Typical jitter ~30 ms (Tokyo), under 30 ms (Sydney), ~25 ms (Chengdu); the authors note all three
> jitter records "displayed self-similarity and burstiness". **The critical caveat: the clocks were
> not synchronised** — "it was not feasible to take traces using terminals whose clocks were
> accurately synchronised, only information concerning inter-packet arrival times was available" —
> so these are *jitter* traces, not one-way delay traces, and they drive a **simulated VoIP network
> model** rather than a replay of measured delay. Baselines are a linear recursive filter, an MLP
> neural predictor, and three Concord variants. Metrics: additional playout delay, late-arrival
> PLR, consecutive PLR, PESQ MOS.
> *How the operating point is chosen and the tradeoff reported.* Explicitly target-PLR, and the
> paper goes one step further than the rest of the field: it treats **"PLR error", the absolute
> difference between the target and the achieved loss rate, as a first-class result with its own
> figure**. That is a different question from where you sit on the curve — it asks whether the knob
> the application turns does what it says. It also reports **consecutive PLR as a function of
> target PLR, and again as a function of buffering delay**, i.e. the clustering of late loss is
> plotted against the operating point rather than summarised as an average.
> *Transfers as machinery if it transfers at all.* A GARCH fit is state and arithmetic in the
> per-sample path; invariant 8 (no allocations in the hot path) and invariant 6 (C# 9,
> netstandard2.1, no NuGet) make a fitted time-series model a real implementation cost here, and
> "Direct GARCH" appears to require an estimation procedure, not just a recursion.

> **Shallwani & Kabal, "An adaptive playout algorithm with delay spike detection for real-time
> VoIP"**, IEEE CCECE 2003, pp. 997-1000. DOI `10.1109/CCECE.2003.1226063`; record verified at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/CCECE.2003.1226063?fields=title,abstract,year,venue,authors>
> and in the authors' lab publication list at <http://www.ece.mcgill.ca/~pkabal/papers/>
> (both accessed 2026-09-15).
> *Proposes:* an NLMS (normalised least-mean-square) adaptive linear predictor of packet delay,
> plus an extension (E-NLMS) that adds an explicit spike-detection mode. The NLMS predictor itself
> is DeLeon & Sreenan's (ICASSP 1999); the spike detector is what this paper adds.
> *Claims:* the spike-detection extension improves on the plain NLMS predictor.
> *Conditions:* **still not read** — abstract elided by Semantic Scholar, IEEE Xplore returns an
> empty body. **But one condition is now known secondhand from a source that was read in full:**
> Miranda-Campos & Ramos (below) state that this work and DeLeon & Sreenan's evaluated on **traces
> generated with the `ping` program**, measuring RTT, between three hosts in the US and one in the
> UK, and that those traces "are not available to everyone". That matters: a ping RTT series is not
> a one-way audio delay series with talkspurt structure, and it is the explanation the replication
> offers for why the result did not carry over. See the replication below before believing the
> claim.

> **Miranda-Campos & Ramos, "On NLMS estimation for VoIP playout delay algorithms — improving delay
> spike detection"**, SIGMAP (ICETE) 2008.
> <https://www.scitepress.org/papers/2008/19407/> (accessed 2026-09-15).
> **Recovered 2026-09-16** — text extracted locally from
> <https://www.scitepress.org/papers/2008/19407/19407.pdf> (re-accessed 2026-09-16) and the full
> paper read.
> *Proposes:* a replication and comparison of the two NLMS playout algorithms above, then a third
> variant, NLMS-mod, which keeps DeLeon & Sreenan's NLMS predictor but **replaces Shallwani &
> Kabal's spike detector with Ramjee et al.'s** (§2), specifically by adjusting Ramjee's `var`
> parameter.
> *Claims:* **a negative result on the published claim** — "the algorithm that uses spike detection
> does not overperfom the first one" as previously claimed. The authors then propose their own
> NLMS-with-spike-detection variant which they report outperforms both, "for the loss rates of
> interest (below 5%-10% depending on the audio codec used)". They offer two candidate
> explanations for the failed replication, and the first is the interesting one: Shallwani & Kabal
> evaluated on **ping traces measuring RTT**, and "the delay process of ping packets may not model
> well a delay process obtained by a real voice conversation". The second is blunter — "this result
> might reflect a bug in the Shallwani's algorithm".
> *Caveat on the source itself:* the paper's Conclusions section states the opposite of its own
> abstract and results section ("the Shallwani's E-NLMS algorithm **does** overperform the
> DeLeon's algorithm"). Read against the abstract, Section 4 and Figure 3, this is a dropped
> negation in the conclusion, not a second finding. Noted because a reader skimming the conclusion
> would take away the reverse of the paper's result.
> *Conditions:* **read in full.** Trace-driven simulation on **the six NeVoT traces of Moon,
> Kurose & Towsley 1998**, chosen explicitly because they are real conversations with both sender
> and receiver timestamps, and because the ping-derived traces of the prior work "are not available
> to everyone and thus we cannot verify the delay behavior in them". The paper **reproduces Moon
> et al.'s trace table**, which is the best record this file has of that trace set (see §2). The
> table below was reconstructed from text extraction, so **the column-to-header alignment is
> inferred** (each column's values followed its own header in the extracted stream, which is what
> the mapping rests on); the sender/receiver pairs and the start times are unambiguous:
>
> | # | Sender | Receiver | Start | Length (s) | Talkspurts | Packets |
> |---|---|---|---|---|---|---|
> | 1 | UMass | GMD Fokus | 08:41pm 6/27/95 | 1348 | 818 | 56979 |
> | 2 | UMass | GMD Fokus | 09:58am 7/21/95 | 1323 | 406 | 24490 |
> | 3 | UMass | GMD Fokus | 11:05am 7/21/95 | 1040 | 536 | 37640 |
> | 4 | INRIA | UMass | 09:20pm 8/26/93 | 580 | 252 | 27814 |
> | 5 | UCI | INRIA | 09:00pm 9/18/93 | 1091 | 540 | 52836 |
> | 6 | UMass | Osaka University | 00:35am 9/24/93 | 649 | 299 | 23293 |
>
> One 160-byte audio packet approximately every 20 ms during speech activity. Only the **variable
> portion** of end-to-end delay is used, obtained by subtracting each trace's minimum delay, "by
> considering the variable portion of the end-to-end delay, synchronization between sender and
> receiver clocks can be avoided". Metrics are taken verbatim "as in (Moon et al., 1998)": a
> per-packet played-out indicator, `D_avg = (1/T) Σ r·(p - t)` over played-out packets only, and
> `l = (L - T)/L × 100`. **Note what that means for cadence comparability (root `CLAUDE.md`'s
> warning): the average delay is computed over the played-out population only, so a scheme that
> drops more packets averages over a different, smaller sample set than one that drops fewer.**
> *Transfers as a warning, not a technique.* The most useful thing in this family's readable
> literature is that a widely repeated improvement failed to replicate. Spike detection is the
> single most-repeated idea in adaptive playout and there is published doubt about it. If this
> project builds a spike-detecting playout policy, the falsifier should be "does it beat the
> non-spike-detecting version on the *same* traces", which is exactly the comparison that failed
> here.

## 5. Family D: quality-model-driven playout

A different objective function. Instead of minimising delay at a loss target, maximise a *predicted
subjective quality score* — almost always the ITU-T E-model — which is a function of both delay and
loss, so the optimum falls out of the model rather than being set by a knob. This family is the
reason the VoIP literature has an operating-point convention at all (§12): the E-model supplies an
exchange rate between milliseconds and loss percentage points.

**Relevance caveat, stated plainly.** The E-model's impairment functions are fitted to *speech*
listening tests. Nothing in it transfers to a VR operator's perception of a robot arm, and this
project has its own human-factors axis (a different file in this directory). What transfers is the
*shape* — optimise a scalarised delay-and-loss objective — not the coefficients.

> **Atzori & Lobina, "Playout buffering in IP telephony: a survey discussing problems and
> approaches"**, IEEE Communications Surveys & Tutorials 8(3):36-46 (2006).
> DOI `10.1109/COMST.2006.253269`; record verified at
> <https://api.crossref.org/works?query.bibliographic=Playout+buffering+in+IP+telephony+a+survey+discussing+problems+and+approaches+Atzori+Lobina&rows=2&select=title,author,container-title,volume,issue,page,issued,DOI>
> and at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/comst.2006.253269?fields=title,abstract,year,venue,authors>
> (both accessed 2026-09-15).
> *Proposes:* a taxonomy — most playout schemes consist of (i) predicting delay statistics for
> future packets and (ii) setting end-to-end delay to limit or avoid late loss; a newer class uses
> a speech-quality model to find the buffer setting maximising expected quality.
> *Claims:* survey, no new algorithm.
> *Conditions:* **still not read after a dedicated second retrieval attempt on 2026-09-16, and it
> is now believed to be unreachable without institutional access.** Routes tried and their
> outcomes: IEEE Xplore — not retrievable; ACM DL — HTTP 403 on every path; OpenAlex and Semantic
> Scholar both report `oa_status: closed` with **no open-access location of any kind**; the
> University of Cagliari's own IRIS repository record (`hdl.handle.net/11584/26155` →
> <https://iris.unica.it/handle/11584/26155>, accessed 2026-09-16) is **metadata only** — "Non ci
> sono file associati a questo prodotto"; CiteSeerX now redirects its entire corpus to
> `web.archive.org`, and every Internet Archive host is unreachable here; CORE, BASE and every
> general search engine reachable by this environment returned HTTP 403 or a bot challenge. The
> sibling MSAN 2005 and IEEE TMM 2006 papers by the same authors are also closed.
> **This remains the single most valuable missing source in this file** — it is still the only
> thing that would let the families above be compared on a common footing rather than pairwise.
> If anyone gets institutional access, read it first.
> *Partial substitute, added 2026-09-16:* Boi, Atzori & Lobina's openly-licensed MobiMedia 2007
> paper, the last entry in this section, is by the same authors on the same family and **was** read in full. It
> supplies this file's only readable primary statement of the quality-maximisation objective and
> its E-model coefficients. It is one worked instance, not a survey, and it does not close this gap.

> **Sakir & Feldbauer**, arXiv:0909.2816 — see Family B above; it belongs to both families, being
> a percentile-style estimator fitted to an E-model objective.

> **Atzori, Lobina & Isola, "Playout buffering in IP telephony: a quality maximization approach"**,
> MSAN 2005, pp. 49-53. DOI `10.1109/MSAN.2005.1489941`; record verified in the same Crossref
> query as the survey above (accessed 2026-09-15).
> *Proposes:* choose the playout point that maximises predicted quality rather than one that hits a
> loss target.
> *Claims / Conditions:* **not read** — bibliographic record only.

> **Boi, Atzori & Lobina, "IP Telephony over Satellite Networks: Control of the Playout Delay to
> Maximize the Perceived Quality"**, ICST MobiMedia 2007. DOI `10.4108/icst.mobimedia2007.1742`;
> open-access PDF at <http://eudl.eu/pdf/10.4108/ICST.MOBIMEDIA2007.1742> (accessed 2026-09-16),
> text extracted locally and read in full.
> **Why this entry exists:** the 2006 survey above could not be retrieved by any route (§14). This
> paper is by the same authors, from the same lab (MCLab, University of Cagliari), on the same
> family, and it is openly licensed — so it is the **readable primary this file has for Family D's
> argument**, in place of the survey. It is not a survey and it does not give the common footing
> the survey would have; it gives one worked instance from the people who wrote the survey.
> *Proposes:* set the playout point for the next talkspurt to the value maximising the ITU-T
> E-model R-factor, given a running estimate of the delay pdf. The paper writes the scalarisation
> out explicitly: `R = 100 - I_s - I_d - I_e,eff + A`, simplified to `I_s = 6.8`, `A = 0`,
> a **delay impairment `I_d = 0.024·d + 0.11·(PD - 177.3)·H(PD - 177.3)`** — linear in delay, with
> a **knee at 177.3 ms** beyond which the slope is roughly five times steeper — and a loss
> impairment logarithmic in the loss rate, `I_e,eff = 11 + 40·ln(1 ± 10E)` (the sign of the inner
> term did not survive text extraction cleanly; the logarithmic form does not depend on it).
> Total delay and total loss are decomposed into codec, network and buffer terms, with
> `E = e_net + (1 - e_net)·e_dejitter`, which keeps network loss and late-loss (dejitter) discard
> as **separate** quantities that compose rather than add — the §7 distinction, respected.
> A satellite-specific term models the "saw tooth" delay pattern imposed by TDMA framing, shifting
> each packet's delay pdf by its index within the frame.
> *Claims:* average R-factor 83.2 for the proposed scheme versus **82.3 for E-NLMS and 78.7 for an
> adaptive linear filter with spike detection**, at average playout delays of 0.498 s, 0.5155 s and
> 0.485 s respectively. The authors note that E-NLMS "is able to obtain good results at the expense
> of delays that are higher ... due to the **oversetting of the delay when spikes are detected**" —
> a third independent observation in this file that spike detection buys its robustness with
> delay. The linear filter's R-factor oscillates talkspurt to talkspurt, "resulting occasionally in
> burst of losses".
> *Conditions:* **read in full**, and the operating point is unusually relevant here. A real
> **GEO satellite** path — Ka-band (20-30 GHz), Hot Bird 6 with a Skyplex TDMA frame structure —
> with both endpoints **NTP-synchronised**. Codecs GSM, iLBC and G.711 µ-law; calls 5-10 minutes;
> **average network delay 354-400 ms with variance 0.0034-0.0040 s²**. The headline comparison is
> one representative 5-minute GSM conversation, R-factor over the first 30 talkspurts with the
> first five excluded from the average (all algorithms start from a 100 ms playout delay, so the
> exclusion is a startup-transient guard — the same artefact the root `CLAUDE.md` warns about when
> a dramatic win turns out to be a warm-up effect). Only a handful of traces, one reported.
> *How the operating point is chosen — and it is a third convention, distinct from both of §12's.*
> Neither a target loss rate nor a delay percentile: "**no thresholds for the loss and/or delay
> have to be considered**: the playout algorithm automatically adapts the buffer size so as to
> maximize the expected quality". The exchange rate between milliseconds and loss points is
> supplied entirely by the E-model's fitted coefficients, and the reported quantity is the R-factor
> with average playout delay alongside it — not a curve.
> *Transfers as a shape only.* The E-model coefficients are fitted to speech listening tests and
> mean nothing for a VR operator (see this section's caveat). What transfers is that **the knee is
> the interesting part**: a scalarised objective with a piecewise-linear delay penalty behaves very
> differently either side of its knee, and where this project's own objective would put that knee
> is a human-factors question, not a networking one.

## 6. Family E: NetEQ as shipped

The most important entry in this file, because it is the only scheme here whose *actual*
implementation can be read line by line rather than inferred from a paper. NetEQ is libwebrtc's
audio jitter buffer and it is a **histogram/quantile tracker (Family B) with a second, separately
optimised reordering path bolted on** — not the EWMA scheme of folklore, and not a forecasting
scheme.

The design worth transferring is the split: **two independent estimators, combined by `max`**. One
answers "how much delay do I need so I rarely run out of packets"; the other answers "how much
delay do I need so reordered packets are not wasted". They have different objective functions —
the first is a quantile, the second is an explicit cost minimisation — and they are computed from
different sample populations.

> **WebRTC NetEQ overview (`modules/audio_coding/neteq/g3doc/index.md`)**.
> <https://webrtc.googlesource.com/src/+/main/modules/audio_coding/neteq/g3doc/index.md>
> (accessed 2026-09-15).
> *Proposes:* the component contract — an adaptive jitter buffer where "the interarrival time
> between packets is analyzed and statistics is updated which is used to derive a new target
> playout delay", with packets "discarded if too late for playout (for example if it was
> reordered)", and the four playout operations (normal, accelerate, preemptive expand, expand).
> *Claims:* none quantitative.
> *Conditions:* design documentation, no evaluation. It is thin: it does not describe the delay
> manager's statistics at all, which is why the source files below matter more than the doc.

> **`delay_manager.cc`**.
> <https://webrtc.googlesource.com/src/+/main/modules/audio_coding/neteq/delay_manager.cc>
> (accessed 2026-09-15).
> *Proposes:* `target_level_ms_` is the underrun optimizer's recommendation, raised to the reorder
> optimizer's recommendation when the latter is enabled:
> `target_level_ms_ = std::max(target_level_ms_, reorder_optimizer_->GetOptimalDelayMs()...)`.
> Start value `kStartDelayMs = 80`. Quantile is carried in Q30 (`1 << 30`), forget factors in Q15
> (`1 << 15`). Both optimizers are fed the same `arrival_delay_ms`, and the reorder optimizer
> additionally gets a `reordered` flag.
> *Conditions:* shipping code, no evaluation attached.

> **`underrun_optimizer.cc`**.
> <https://webrtc.googlesource.com/src/+/main/modules/audio_coding/neteq/underrun_optimizer.cc>
> (accessed 2026-09-15).
> *Proposes:* a 100-bucket histogram of relative arrival delay at `kBucketSizeMs = 20` ms per
> bucket (so 2000 ms of trackable delay), with an exponential forget factor; target delay is
> `(1 + bucket_index) * kBucketSizeMs` where `bucket_index` is the configured **quantile** of the
> histogram. Optionally it *resamples*: within a configured interval it feeds the histogram only
> the **maximum** delay seen in that interval, rather than every sample.
> *Conditions:* shipping code.
> *Two details this project should notice.* First, the resampling option changes the sample
> population the quantile is computed over — the exact failure mode the root `CLAUDE.md` warns
> about when two implementations emit on different cadences. Second, the 20 ms bucket quantisation
> means this scheme literally cannot express a target between 20 ms and 40 ms, which is coarse
> relative to a 90-100 Hz sample interval of ~10-11 ms.

> **`reorder_optimizer.cc`**.
> <https://webrtc.googlesource.com/src/+/main/modules/audio_coding/neteq/reorder_optimizer.cc>
> (accessed 2026-09-15).
> *Proposes:* a second histogram over the relative delay of **reordered** packets (in-order packets
> land in bucket 0), and a delay chosen to minimise
> `delay_ms + 100 * ms_per_loss_percent_ * loss_probability`, where `loss_probability` is the
> probability mass beyond the candidate delay. `ms_per_loss_percent_` is the tunable exchange rate:
> how many milliseconds of added delay are worth one percentage point of avoided late loss.
> *Conditions:* shipping code.
> *Transfers directly and is the single most actionable idea in this file for the Buffering axis.*
> It is an explicit, cheap, online scalarisation of the delay-versus-loss curve, and it separates
> the reordering-driven component of the buffer from the jitter-driven one.

> **`histogram.cc`**.
> <https://webrtc.googlesource.com/src/+/main/modules/audio_coding/neteq/histogram.cc>
> (accessed 2026-09-15).
> *Proposes:* fixed-point exponentially forgetting histogram. Update is
> `bucket = (int64(bucket) * forget_factor_) >> 15` across all buckets, then
> `(32768 - forget_factor_) << 15` added to the observed bucket, with an explicit renormalisation
> so the buckets sum to `1 << 30`. `Quantile()` starts from 1 and subtracts buckets until it
> crosses the threshold. `start_forget_weight` makes the forget factor start near zero and rise to
> the steady-state value as `add_count_` grows:
> `forget_factor_ = (1 << 15) * (1 - start_forget_weight_/(add_count_ + 1))`.
> *Conditions:* shipping code.
> *Transfers directly.* This is a complete, allocation-free, integer-only recipe for a forgetting
> quantile tracker — the "cold start" problem (a quantile tracker is useless until the histogram
> fills) is solved here by a time-varying forget factor, which is a trick worth having whether or
> not the rest of NetEQ is copied.

> **Lyu, "How WebRTC's NetEQ Jitter Buffer Provides Smooth Audio"**, webrtcHacks, 3 June 2025.
> <https://webrtchacks.com/how-webrtcs-neteq-jitter-buffer-provides-smooth-audio/>
> (accessed 2026-09-15).
> *Proposes:* a walkthrough of the above, and the numbers that are not obvious from the source: a
> default forget factor of **0.983**; a worked quantile example ("if the quantile is 0.95, then
> 120 ms is chosen as the target level because that bucket sums up to 0.96"); the reorder
> tradeoff described as "delay_ms + 20 ms * loss_percent"; and the observation that NetEQ compares
> *individual packet delay* against the target rather than managing total buffer occupancy.
> *Claims:* explanatory, not experimental.
> *Conditions:* a blog post describing a code base, with no measurements. Treated here as
> documentation of the implementation, corroborated against the source files above — not as
> evidence of performance.

## 7. How the field measures burst loss, discard and reordering

This section is here because it is the part of the field most directly reusable by this project and
the part least likely to be found by searching for "playout". The IETF spent a decade defining the
measurements that a delay-versus-loss comparison needs, and the definitions are unusually careful
about exactly the distinctions that get blurred.

**The load-bearing distinction: loss versus discard.** A packet that never arrives is *lost*. A
packet that arrives but cannot be used is *discarded*. They have different causes and different
fixes, and averaging them together hides which one you have.

> **Friedman, Caceres & Clark (eds.), "RTP Control Protocol Extended Reports (RTCP XR)"**,
> RFC 3611 (November 2003).
> <https://www.rfc-editor.org/rfc/rfc3611.html> (accessed 2026-09-15).
> *Proposes:* the VoIP metrics block. Reports **jitter buffer nominal delay**, **maximum delay**,
> **absolute maximum delay** and an **adaptive/non-adaptive flag** as first-class reported
> quantities; separates *loss rate* from *discard rate*, where discard covers packets "discarded
> ... due to late or early arrival, under-run or overflow at the receiving jitter buffer". Defines
> **Gmin**, the run of consecutively-received packets that terminates a burst, with a recommended
> value of 16 corresponding to a minimum burst density of 6.25%, and **burst density** as the
> fraction of packets lost or discarded within burst periods.
> *Claims:* a specification.
> *Conditions:* none — but note that the Gmin=16 recommendation is calibrated to 20 ms voice
> packets, i.e. a 320 ms observation run. At 90-100 Hz the same packet count is ~160-180 ms, so the
> *constant* does not transfer even though the definition does.
> *Transfers as machinery, strongly.* Nominal/maximum/absolute-maximum delay plus an adaptive flag
> is a better-designed reporting schema for a playout policy than a single delay number.

> **Clark, Zhang, Zhao & Wu (ed.), "RTP Control Protocol (RTCP) Extended Report (XR) Block for
> Burst/Gap Loss Metric Reporting"**, RFC 6958 (May 2013). <https://www.rfc-editor.org/rfc/rfc6958.html>
> (accessed 2026-09-15).
> *Proposes:* splitting a stream into **bursts** (loss rate high enough to degrade quality,
> "generally over 5 percent") and **gaps**, using the Gmin threshold, and reporting per-period
> statistics rather than one average.
> *Claims:* "the burstiness of packet loss affects user experience, may influence any sender
> strategies to mitigate the problem, and may also have diagnostic value" — i.e. an explicit
> statement that an average loss rate is the wrong instrument for bursty loss.
> *Conditions:* a specification, no measurement.
> *Transfers directly to this project's burst-loss impairment profiles.* It is the field's answer
> to "how do you report burst loss without averaging it away", and it is a definition, so adopting
> it does not require trusting anyone's numbers.

> **Clark, Huang & Wu (ed.), "RTP Control Protocol (RTCP) Extended Report (XR) Block for Burst/Gap
> Discard Metric Reporting"**, RFC 7003 (September 2013). <https://www.rfc-editor.org/rfc/rfc7003.html> (accessed 2026-09-15).
> *Proposes:* the same burst/gap decomposition applied to **discards** rather than losses —
> packets that arrived but were "too early to be played out, too late to be played out, or thrown
> away before playout".
> *Claims:* a specification; the motivation given is that discard clustering diagnoses jitter
> buffer behaviour specifically, as distinct from network loss.
> *Conditions:* none.
> *Transfers directly.* This is the closest thing the field has to a standard definition of
> **bursty late-loss**, which is precisely the quantity a playout policy under a burst-loss profile
> should be judged on.

> **Clark, Zorn & Wu, "RTP Control Protocol (RTCP) Extended Report (XR) Block for Discard Count
> Metric Reporting"**, RFC 7002 (September 2013).
> <https://www.rfc-editor.org/rfc/rfc7002.html> (accessed 2026-09-15).
> *Proposes:* a plain count of packets received correctly but never played out.
> *Claims:* useful for identifying and characterising transport problems; notably the RFC does
> **not** claim it as a jitter-buffer tuning signal, which is a restraint worth noticing.
> *Conditions:* none.

> **Morton, Ciavattone, Ramachandran, Shalunov & Perser, "Packet Reordering Metrics"**, RFC 4737
> (2006). <https://www.rfc-editor.org/rfc/rfc4737.html> (accessed 2026-09-15).
> *Proposes:* a family of reordering metrics: a packet is reordered if its sequence number is below
> the next-expected value; **reordering extent** (how many packets back it fell, i.e. the buffer
> depth needed to repair it); **late-time offset**, `LateTime(s[i]) = DstTime(i) - DstTime(j)` for
> the earliest earlier-arriving packet with a higher sequence number, which is explicitly the
> metric for deciding "can a de-jitter buffer with a finite time limit accommodate this"; and
> **n-reordering** for TCP duplicate-ACK behaviour.
> *Claims:* a specification. Its central argument is that a single threshold is the wrong
> instrument, because "the effects of packet reordering vary with these procedures, a metric that
> quantifies a key aspect of one receiver's behavior could be irrelevant to a different receiver".
> *Conditions:* none.
> *Transfers directly, and it is the metric this project's reordering impairment most obviously
> wants.* **Late-time offset is a delay measured in time, not in packets** — it is directly
> comparable to a playout buffer setting, which reordering-extent-in-packets is not.

> **Bennett, Partridge & Shectman, "Packet reordering is not pathological network behavior"**,
> IEEE/ACM Transactions on Networking 7(6):789-798 (1999). DOI `10.1109/90.811445`; record verified
> at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/90.811445?fields=title,abstract,year,venue,authors>
> (accessed 2026-09-15).
> *Proposes:* the observation that reordering arises from ordinary parallelism inside routers and
> links, not from faults.
> *Claims:* reordering incidence is substantially higher than previously reported, and it is
> normal rather than pathological.
> *Conditions:* **not read** — abstract elided; IEEE Xplore not retrievable. The measurement
> context is 1990s Internet backbone equipment and is very unlikely to transfer quantitatively to
> anything current; the *qualitative* point (reordering is structural, so a playout policy must
> handle it as a design case rather than an anomaly) is what survives.

> **Yajnik, Moon, Kurose & Towsley, "Measurement and modelling of the temporal dependence in packet
> loss"**, IEEE INFOCOM '99, pp. 345-352. DOI `10.1109/INFCOM.1999.749301`; record verified at
> <https://api.crossref.org/works?query.bibliographic=Measurement+and+modelling+of+the+temporal+dependence+in+packet+loss+Yajnik+Moon+Kurose+Towsley&rows=3&select=title,author,container-title,page,issued,DOI>
> (accessed 2026-09-15).
> *Proposes:* Markov-chain models (including the two-state Gilbert family) fitted to measured loss
> traces, to capture temporal *dependence* in loss rather than an i.i.d. rate.
> *Claims / Conditions:* **not read** — bibliographic record only. Included because it is the
> standard reference for "burst loss is modelled as a two-state Markov chain", which is the model
> RFC 8868 names when it allows for correlated loss (§8).

## 8. Family F: delay-based congestion control

**Read this section knowing it is the weaker match.** Every scheme here controls a *sending rate*.
This project's pose stream has a fixed cadence and a fixed payload; there is no rate to control, and
there is no congestion controller in the system today. What transfers is not the control loop but
the **delay-signal estimation**: how to get a usable queuing-delay estimate out of noisy one-way
delay measurements, how to maintain a base-delay reference that does not drift, and how to avoid
mistaking jitter for congestion. Those are the same problems a playout policy has when it decides
whether the path has genuinely got slower or just twitched.

The families differ mainly in **what they take as the congestion signal**: absolute queuing delay
above a base (Vegas, LEDBAT, NADA, SCReAM, Copa), the *gradient* or trend of one-way delay
(GCC), a delivery-rate model with an RTT floor (BBR), or an empirical utility of actually-observed
performance (PCC).

> **Brakmo & Peterson, "TCP Vegas: End to End Congestion Avoidance on a Global Internet"**,
> IEEE JSAC 13(8):1465-1480 (1995).
> <https://experts.arizona.edu/en/publications/tcp-vegas-end-to-end-congestion-avoidance-on-a-global-internet/>
> (accessed 2026-09-15). The author-hosted PDF at
> <https://www.cs.princeton.edu/courses/archive/fall06/cos561/papers/vegas.pdf> resolves (541 KB)
> but its text layer was not extractable.
> *Proposes:* the founding delay-based idea — estimate a `BaseRTT` as the minimum observed RTT,
> compute expected versus actual throughput, and adjust the window to keep the difference between
> two thresholds, so the sender aims to keep a small constant amount of data queued instead of
> filling the buffer.
> *Claims:* 37-71% better throughput than Reno with one-fifth to one-half the losses.
> *Conditions:* the landing page states results come from **both simulations and Internet
> measurements**; the detailed setup was not read. The Internet of 1995 is not a useful operating
> point for anything here. The mechanism, not the number, is what to take.
> *Transfers as machinery.* `BaseRTT` as a min-filter is the ancestor of every base-delay estimator
> below, and its failure mode — the min is wrong if the queue never drains — is the single most
> repeated bug in this whole family.

> **Shalunov, Hazel, Iyengar & Kuehlewind, "Low Extra Delay Background Transport (LEDBAT)"**,
> RFC 6817 (2012). <https://www.rfc-editor.org/rfc/rfc6817.html> (accessed 2026-09-15).
> *Proposes:* control on **one-way delay**, not RTT, against a `TARGET` queuing delay that "MUST be
> 100 milliseconds or less"; base delay estimated as a minimum over a `BASE_HISTORY` of roughly ten
> one-minute minima; explicit treatment of clock skew (100-200 ppm, about 6 ms of error per minute,
> which the one-minute windowing bounds).
> *Claims:* a scavenger transport that yields to interactive traffic.
> *Conditions:* **a specification with no evaluation section.** The RFC itself designates multiple
> areas as requiring experimentation — non-FIFO queues, route changes, parameter tuning, filter
> effectiveness, protocol interaction — and openly documents the **latecomer advantage**: a flow
> that starts while a queue is already standing measures an inflated base delay and then targets
> up to twice the intended queuing delay.
> *Transfers as machinery, and the clock-skew discussion is the most useful part.* One-way delay
> between two clocks is exactly what a receiver-side playout policy measures, and this is the
> clearest published statement of the drift budget. Note that this project's Core takes time from
> an injected `ITimeAuthority` and its replays are deterministic, so skew is a *modelling* choice
> here rather than a measured nuisance — which means a sweep could deliberately include it, and the
> field's numbers say what magnitude to model.

> **Balasubramanian, Ertugay, Havey & Bagnulo, "LEDBAT++: Congestion Control for Background
> Traffic"**, draft-irtf-iccrg-ledbat-plus-plus-01 (25 August 2020); the document series is still
> live (version 06, 29 January 2026, awaiting RFC Editor) per
> <https://datatracker.ietf.org/doc/draft-irtf-iccrg-ledbat-plus-plus/> (accessed 2026-09-15).
> <https://datatracker.ietf.org/doc/html/draft-irtf-iccrg-ledbat-plus-plus-01>
> (accessed 2026-09-15).
> *Proposes:* fixes to five documented LEDBAT failures — latecomer advantage, inter-LEDBAT
> unfairness, **latency drift** (the ten-minute base window lets queuing delay ratchet upward over
> long connections), over-aggression on small-buffer links, and fragility of one-way delay in the
> absence of clock synchronisation. Target queuing delay lowered to **60 ms**.
> *Claims:* deployed at scale as the Windows background transport.
> *Conditions:* informational draft; no experimental section. Deployment is asserted, not measured
> here.
> *Transfers as machinery.* "Latency drift" is the failure mode a long-running adaptive playout
> buffer with a min-based reference would also have, and it is named and diagnosed here.

> **Arun & Balakrishnan, "Copa: Practical Delay-Based Congestion Control for the Internet"**,
> USENIX NSDI '18.
> <https://www.usenix.org/conference/nsdi18/presentation/arun> (accessed 2026-09-15); full text at
> <https://www.usenix.org/system/files/conference/nsdi18/nsdi18-arun.pdf> (accessed 2026-09-15).
> *Proposes:* a target rate `1/(δ·d_q)` from an estimated queuing delay, periodic queue draining so
> the base RTT can actually be measured, a noise-robust queuing-delay estimator, and **TCP-mode
> switching** — detect a buffer-filling competitor and become one, otherwise stay low-delay.
> *Claims:* similar throughput to Cubic at much lower delay and better RTT-fairness; significantly
> fairer and lower-delay than BBR and PCC.
> *Conditions:* evaluated in the **Mahimahi** emulator plus **Pantheon** real-Internet paths, with
> emulated satellite links, a simulated datacenter network, and an explicit *robustness to packet
> loss* experiment using **random** (stochastic) loss. The section headings were read from the PDF;
> the exact RTT ranges, buffer sizes and loss rates were not extracted. **No burst-loss or
> reordering experiment was identified.**
> *Does not transfer as a controller*, but the "periodically drain the queue so your minimum is
> real" trick is directly relevant to any base-delay estimate this project might keep.

> **Cardwell, Swett & Beshay, "BBR Congestion Control"**, draft-ietf-ccwg-bbr-06 (6 July 2026,
> BBRv3); the expired v2 draft is by Cardwell, Cheng, Hassas Yeganeh, Swett & Jacobson.
> <https://datatracker.ietf.org/doc/html/draft-ietf-ccwg-bbr> (accessed 2026-09-15). The earlier
> ICCRG draft (v2) is at
> <https://datatracker.ietf.org/doc/draft-cardwell-iccrg-bbr-congestion-control/>
> (accessed 2026-09-15), expired, replaced by the CCWG draft.
> *Proposes:* an explicit path model — `BBR.max_bw` (windowed max of delivery-rate samples) and
> `BBR.min_rtt` (minimum RTT over the last 10 s) — driving a ProbeBW cycle (UP / DOWN / CRUISE /
> REFILL) plus a periodic **ProbeRTT** that cuts inflight to 50% of BDP for 200 ms every 5 s to
> refresh the RTT floor. BBRv3 adds a short-term model with `BBR.LossThresh = 2%` tolerated loss
> per round during probing and a `0.7` multiplicative decrease on loss.
> *Claims:* the draft states BBR "is robust to packet reordering" and makes no ordering
> assumptions; it also says delivery-rate sampling caps samples at the send rate to filter
> implausible values caused by ack aggregation and compression.
> *Conditions:* **an Internet-Draft, not a measurement paper.** Evidence offered is production
> deployment at scale with open-source TCP and QUIC implementations; the draft concedes BBR "does
> not deal well with persistently application limited traffic" and flags ECN response and ProbeRTT
> interval as needing further experimentation. The ACM Queue article on BBRv1 was **not
> retrievable** (403, see §14), so the v1 deployment numbers are not recorded here.
> *Does not transfer* — a fixed-cadence pose stream is precisely the "persistently application
> limited" traffic the draft says BBR handles badly. Recorded so nobody reaches for it.

> **Dong, Li, Zarchy, Godfrey & Schapira, "PCC: Re-architecting Congestion Control for Consistent
> High Performance"**, USENIX NSDI '15.
> <https://www.usenix.org/conference/nsdi15/technical-sessions/presentation/dong> and
> <https://arxiv.org/abs/1409.7092> (both accessed 2026-09-15).
> *Proposes:* abandon fixed packet-event-to-action mappings; run micro-experiments, measure a
> utility function of the observed result, and move in the direction that empirically helped.
> *Claims:* stable, fair equilibrium and "consistent and often 10x" improvement over TCP.
> *Conditions:* **abstract only.** Both landing pages describe evaluation as "many real-world and
> challenging environments" without naming them; neither names loss models, RTT ranges or
> reordering. This is exactly the kind of headline that must not be repeated without its setup.
> *Does not transfer* directly, but the *method* — define a utility, measure it online, move
> empirically — is the closest thing in the CC literature to what an adaptive playout policy with a
> tunable delay/loss exchange rate is doing.

> **Holmer, Lundin, Carlucci, De Cicco & Mascolo, "A Google Congestion Control Algorithm for
> Real-Time Communication"**, draft-ietf-rmcat-gcc-02 (July 2016).
> <https://datatracker.ietf.org/doc/html/draft-ietf-rmcat-gcc-02> (accessed 2026-09-15).
> **The closest source in this section to this project's traffic class**, because GCC is the
> real-time-media controller.
> *Proposes:* (i) an **arrival-time filter** — a scalar Kalman filter over inter-group delay
> variation with state noise `q = 1e-3` and adaptively estimated measurement noise, with an outlier
> clip at three standard deviations; (ii) an **over-use detector with an adaptive threshold**,
> `del_var_th(i) = del_var_th(i-1) + (t(i)-t(i-1)) * K(i) * (|m(i)| - del_var_th(i-1))` with
> `K_u = 0.01` and `K_d = 0.00018`, clamped to `[6, 600]` ms and frozen when `|m(i)|` exceeds the
> threshold by more than 15 — the asymmetry (raise fast, lower ~55x slower) is the whole trick, and
> exists to stop the delay-based controller starving against loss-based flows; (iii) a rate
> controller with Increase / Decrease / Hold states, decrease to `0.85 * R_hat`; (iv) a separate
> **loss-based controller**: increase 5% below 2% loss, hold between 2% and 10%, decrease by
> `(1 - 0.5p)` above 10%; final rate is the min of the two.
> *Claims:* the 2/10% thresholds encode an assumption that self-inflicted congestion loss escalates
> quickly, so moderate loss should be ignored rather than reacted to.
> *Conditions:* **implementation experience, not an experiment.** The draft reports deployment in
> Chrome since M23 and in Hangouts, and testing on multi-party conferences and "problematic WiFi",
> with no controlled results of its own; it points at a separate peer-reviewed analysis.
> *Transfers as machinery, and the most interesting part is the loss policy.* GCC explicitly
> **does not react to loss below 2%** and treats 2-10% as a hold band. A system that cannot use
> late data has the opposite instinct. That disagreement is worth understanding before any
> loss-reactive policy is built here.

> **Carlucci, De Cicco, Holmer & Mascolo, "Analysis and design of the Google congestion control for
> web real-time communication (WebRTC)"**, ACM MMSys 2016, pp. 1-12. DOI `10.1145/2910017.2910605`;
> record verified at
> <https://api.crossref.org/works?query.bibliographic=Analysis+and+design+of+the+google+congestion+control+for+web+real-time+communication+WebRTC+Carlucci+De+Cicco+Holmer+Mascolo&rows=3&select=title,author,container-title,page,issued,DOI>
> (accessed 2026-09-15). The same Crossref query also returns the extended journal version,
> **"Congestion Control for Web Real-Time Communication"**, IEEE/ACM Transactions on Networking
> (2017), DOI `10.1109/TNET.2017.2703615`.
> **Recovered 2026-09-16.** The lab has migrated from MediaWiki to Drupal and the file moved; the
> live copies are <http://c3lab.poliba.it/sites/default/files/2026-05/Gcc-analysis.pdf> (MMSys
> 2016) and
> <http://c3lab.poliba.it/sites/default/files/2026-05/Congestion_Control_for_Web_Real-Time_Communication.pdf>
> (IEEE/ACM ToN 2017), both reachable from <http://c3lab.poliba.it/publications> (all accessed
> 2026-09-16). Text extracted locally and both read.
> **Correction to what this file previously said.** The earlier, unread entry asserted that this
> paper "moved GCC's delay estimation from the receiver-side Kalman filter to a sender-side
> trendline". **That is wrong and is retracted.** Neither the MMSys 2016 paper nor the ToN 2017
> extension contains the words "trendline" or "linear regression" anywhere. Both describe the
> **Kalman filter** estimator of one-way delay variation compared against an adaptive threshold —
> i.e. the same algorithm as draft-ietf-rmcat-gcc-02 above, with the controlled evaluation the
> draft lacks. The trendline (linear-regression slope) estimator that libwebrtc ships today is a
> later change and **is not described in this paper**; as far as this survey found, it exists only
> in the libwebrtc source, and this file still does not have a peer-reviewed evaluation of it.
> That gap is real, but it is a different gap from the one previously recorded — see §14.
> *Proposes:* GCC as deployed in Chrome and Hangouts: a Kalman filter producing an estimate `m(t)`
> of the queuing-delay gradient from inter-group one-way delay variation, compared against a
> dynamically adapted threshold to raise an overuse signal; a remote rate controller; and a pacer
> with **pacing factor 1.5**. The paper's own framing of its contribution is the *analysis* — why
> a static threshold starves the flow when the bottleneck queue is small or when a loss-based flow
> shares it, and why the threshold must therefore adapt.
> *Claims:* the adaptive threshold is what makes GCC work. In a single GCC flow over a 1 Mbps link
> with `T_q = 150 ms`, the **95th percentile of RTT falls from 187 ms to 130 ms** with the adaptive
> threshold versus a static one, while the median stays "very close to the propagation delay"
> `RTT_min = 50 ms` in both cases. GCC tracks a link capacity stepped 1 → 2.5 → 0.5 → 1 Mbps every
> 50 s. Against short-lived TCP bursts, two GCC flows reach 72% aggregate channel utilisation.
> *Conditions:* **read in full, and they are narrower than the deployment story suggests.** A
> four-machine **emulated** WAN: Linux 3.16.0, real Chromium browsers signalling through apprtc,
> a `v4l2loopback` virtual webcam cyclically replaying the **"Four People"** YUV sequence encoded
> with **VP8** (which caps the source at 2 Mbps unconstrained), so the media is real but fixed for
> reproducibility. **Round-trip propagation delay is fixed at `RTT_min = 50 ms`** (25 ms each
> direction) via NetEm; bottleneck capacity is set with a Token Bucket Filter, typically 1 Mbps
> (2 Mbps in the multi-flow cases); drop-tail bottleneck queue sized as `q_M = T_q · C` with
> `T_q = 300 ms` by default and 150 ms / 350 ms in specific experiments, chosen by the RMCAT
> evaluation criteria. NIC offloads disabled. Cross traffic is **TCP CUBIC**, long-lived and
> short-lived (3 s every 12 s), plus a reverse-path TCP flow.
> **No loss is ever injected and reordering is never mentioned.** Loss appears only as an
> *outcome* metric (`l = bytes lost / bytes sent`), i.e. self-inflicted congestion loss. Scenarios
> are: variable capacity, multiple concurrent GCC flows, GCC versus long-lived TCP, GCC with
> short-lived TCP, and reverse traffic. Ten repetitions per scenario; experiments run 125-300 s.
> *How the operating point is reported — and it is the opposite convention from §12's playout
> literature.* The metric set is channel utilisation `U = R_r/C`, Jain's fairness index, loss
> ratio, and **the 50th and 95th percentiles of queuing delay, defined as `RTT(t) - RTT_min` over
> the RTT samples carried in RTCP feedback**. That is a delay-*tail* convention, reported against
> a measured propagation floor, with loss as a reported outcome rather than a target. The
> real-time-media congestion-control literature and the playout literature therefore do not share
> a reporting convention, which is worth knowing before borrowing a number from either.

> **De Cicco, Carlucci & Mascolo, "Experimental investigation of the Google congestion control for
> real-time flows"**, FhMN @ SIGCOMM 2013. DOI `10.1145/2491172.2491182`; record verified at
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1145/2491172.2491182?fields=title,abstract,year,venue,authors>
> (accessed 2026-09-15). PDF resolves at
> <https://conferences.sigcomm.org/sigcomm/2013/papers/fhmn/p21.pdf> (accessed 2026-09-15, 2.1 MB)
> but its text layer was not extractable.
> **Recovered 2026-09-16** — text extracted locally from the same SIGCOMM-hosted URL
> (re-accessed 2026-09-16) and the full paper read.
> *Proposes:* a controlled testbed evaluation of GCC rather than a new algorithm.
> *Claims:* three findings, and **two of them are negative**. (1) A single GCC flow keeps channel
> utilisation above 0.8 with contained queuing delay, even when available bandwidth follows a
> staircase pattern. (2) **A GCC video flow is starved when it shares the bottleneck with a TCP
> flow and the bottleneck capacity is at or below 1000 kbps.** (3) **When two GCC flows share a
> bottleneck, "the algorithm behavior appears unpredictable and exhibit poor fairness"**, with the
> cause left as open work. This is the paper whose negative results the 2016 redesign above answers
> with an adaptive over-use threshold, so the two should be read in order.
> *Conditions:* **read in full.** Two hosts across an emulated bottleneck, NetEm plus `tc` setting
> a symmetric one-way delay `d` (so `RTT_m = 2d`) and a bandwidth cap. Bottleneck buffers set to
> **60 KB to emulate a typical home gateway**; available bandwidth swept over
> **{500, 1000, 1500, 2000} kbps** ("typical of ADSL uplink speeds and cable connections") and
> minimum RTT over **{30, 50, 80, 120} ms**. Chromium with instrumented WebRTC sources; a
> `v4l2loopback` virtual webcam cyclically replaying the **Foreman** YUV sequence (the 2016 paper
> switches to "Four People"); encoder capped at 2 Mbps unconstrained. Cross traffic is TCP CUBIC.
> Metrics: channel utilisation `U = R/b`, **good utilisation `g = v/b` excluding FEC and
> retransmission bytes** — a distinction the 2016 paper drops — loss ratio, and queuing delay
> `Q(t) = RTT(t) - RTT_m`. **No loss is injected and reordering is not mentioned**, consistent with
> the rest of this family (§11).

> **Zhu, Pan, Ramalho & Mena, "Network-Assisted Dynamic Adaptation (NADA): A Unified Congestion
> Control Scheme for Real-Time Media"**, RFC 8698 (2020). <https://www.rfc-editor.org/rfc/rfc8698.html> (accessed 2026-09-15).
> *Proposes:* relative one-way delay against a long-window baseline minimum, **denoised by a
> 15-sample minimum filter**, combined with loss and ECN ratios converted into equivalent-delay
> penalties via quadratic terms, into a single aggregate congestion signal; two modes (accelerated
> ramp-up, gradual update).
> *Claims:* recovers from throughput drops caused by wireless delay spikes.
> *Conditions:* evaluated in **ns-2 and ns-3** simulation over the RMCAT test cases plus wireless
> scenarios, with real-world testing in Firefox and video-conferencing clients on home and
> enterprise networks. Simulation-dominant.
> *Transfers as machinery:* the 15-sample min filter is a concrete, cheap answer to "how do I stop
> jitter looking like congestion", and the delay/loss-to-common-currency conversion is the same
> scalarisation move as NetEQ's reorder cost function.

> **Johansson & Sarker, "Self-Clocked Rate Adaptation for Multimedia" (SCReAM)**, RFC 8298 (2017).
> <https://www.rfc-editor.org/rfc/rfc8298.html> (accessed 2026-09-15).
> *Proposes:* queuing delay from send/receive timestamps against `QDELAY_TARGET_LO = 0.1 s`, a
> `qdelay_trend` from autocorrelation to detect incipient congestion, and a self-clocked congestion
> window with a send window that adds slack when queuing delay is low. Explicitly assumes media
> that can be **rate-limited or dropped**, and allows discarding encoded frames.
> *Claims:* keeps queues near-empty by exploiting the natural rate variability of video.
> *Conditions:* LTE system simulation plus simulated Internet bottlenecks, and an OpenWebRTC
> implementation against artificial bottlenecks with OpenH264 and VP9. Recommended experiments span
> EDGE/3G/4G/Wi-Fi/DSL; the RFC does not prescribe RTT values.
> *Does not transfer as a controller* — its core assumption, that the media source's bitrate is
> elastic, is false for a fixed-cadence pose stream — but `qdelay_trend` is a second worked example
> of a delay-*gradient* signal alongside GCC's.

> **Zhang & Yang, "Cross: A Delay Based Congestion Control Method for RTP Media"**,
> arXiv:2409.10042 (16 September 2024). <https://arxiv.org/abs/2409.10042> (accessed 2026-09-15).
> *Proposes:* queue-load-based multiplicative-increase/multiplicative-decrease rate control for
> real-time media.
> *Claims:* low queuing delay and high utilisation **under random loss**; roughly 58% reduction in
> video freezing in a field deployment.
> *Conditions:* **abstract only.** A simulation module plus field deployment; the loss model named
> in the abstract is *random*, not bursty, and no reordering evaluation is mentioned.

> **Ray, Smith, Wei, Chu & Seshan, "SQP: Congestion Control for Low-Latency Interactive Video
> Streaming"**, arXiv:2207.11857 (25 July 2022). <https://arxiv.org/abs/2207.11857>
> (accessed 2026-09-15).
> *Proposes:* frame-coupled **paced packet trains** to sample bandwidth, plus an adaptive one-way
> delay measurement for bounded queuing delay — i.e. probing that is synchronised to the media
> cadence rather than independent of it.
> *Claims:* 2-3x higher bandwidth than GoogCC, Sprout and PCC-Vivace, comparable to Copa; in a real
> **A/B test on Google's AR streaming platform**, 27% (LTE) and 15% (Wi-Fi) more sessions with high
> bitrate and low frame delay versus Copa.
> *Conditions:* **abstract only** for the details, but the abstract is unusually specific about the
> operating point: production AR streaming, LTE and Wi-Fi, compared against queue-building traffic
> (Cubic, BBR). Loss model and reordering are not mentioned in the abstract.
> *Transfers as an idea, not code.* "Couple the probe to the frame cadence" is the CC-side analogue
> of a playout policy that adapts on the media clock rather than on wall time.

> **Winstein, Sivaraman & Balakrishnan, "Stochastic Forecasts Achieve High Throughput and Low Delay
> over Cellular Networks"** (Sprout), USENIX NSDI '13.
> <https://www.usenix.org/conference/nsdi13/technical-sessions/presentation/winstein>
> (accessed 2026-09-15).
> *Proposes:* receiver-side inference of path dynamics from packet arrival times, producing a
> **probabilistic forecast** of how many bytes may be sent while bounding the risk of packets being
> delayed too long. The controller is explicitly risk-bounded rather than target-tracking.
> *Claims:* versus Skype, 7.9x lower self-inflicted delay at 2.2x the bitrate; versus Hangouts
> 7.2x at 4.4x; versus FaceTime 8.7x at 1.9x; matched or beat Cubic-over-CoDel.
> *Conditions:* **trace-driven replay of four commercial LTE and 3G networks**, captured circa
> 2012-2013. This is one of the few entries whose operating point is stated crisply, and it is
> cellular — high, highly variable delay with deep buffers, not burst loss.
> *Transfers as machinery and as framing.* "Forecast a distribution, then choose an operating point
> that bounds the probability of being late" is precisely the playout problem restated as a sender
> problem, and it is the cleanest statement in this file of the quantile-of-a-forecast idea that
> Family C gropes at.

> **Meng, Atre, Xu, Sherry & Apostolaki, "Confucius: Achieving Consistent Low Latency with Practical
> Queue Management for Real-Time Communications"**, arXiv:2310.18030 (2023, rev. 2024).
> <https://arxiv.org/abs/2310.18030> (accessed 2026-09-15).
> *Proposes:* in-network queue scheduling that deliberately **slows bandwidth reallocation to match
> the reaction time of end-host congestion control**, so RTC flows do not stall during the several
> RTTs a controller needs to adapt.
> *Claims:* more than 50% reduction in stall duration with on-par performance for competing flows.
> *Conditions:* **abstract only**; traces, scale and testbed not stated on the abstract page.
> *Does not transfer* (it is a middlebox/AQM change, outside this project's control), but the
> framing is worth borrowing: the cost of an adaptation is not just its steady-state error, it is
> the transient while the loop converges — which is the thing this project would measure as
> correction cost.

> **Fouladi, Emmons, Orbay, Wu, Wahby & Winstein, "Salsify: Low-Latency Network Video through
> Tighter Integration between a Video Codec and a Transport Protocol"**, USENIX NSDI '18.
> <https://www.usenix.org/conference/nsdi18/presentation/fouladi> (accessed 2026-09-15).
> *Proposes:* fusing the codec and the transport — a purely functional video codec is used to
> explore several encodings of *each frame*, and the one whose compressed length fits the current
> capacity estimate is sent, so rate control happens per frame rather than as a longer-term bitrate
> target.
> *Claims:* lower video delay than FaceTime, Hangouts, Skype and WebRTC's reference implementation
> (with and without SVC), and higher visual quality over variable paths.
> *Conditions:* **abstract only.** The landing page states the authors "developed a testbed for
> evaluating real-time video systems end-to-end with reproducible video content and network
> conditions" but does not say whether links were emulated or real, nor name traces, loss models or
> delay metrics.
> *Does not transfer as an algorithm* — there is no codec here to co-design with — but it is the
> strongest published statement of a principle this project already embodies: **a controller that
> owns the media pipeline can avoid provoking queueing instead of reacting to it**, and the value is
> in the coupling, not in either half.

> **Yan, Ma, Hill, Raghavan, Wahby, Levis & Winstein, "Pantheon: the training ground for Internet
> congestion-control research"**, USENIX ATC '18.
> <https://www.usenix.org/conference/atc18/presentation/yan-francis> (accessed 2026-09-15); PDF at
> <https://pantheon.stanford.edu/static/pantheon/documents/pantheon-paper.pdf>
> (accessed 2026-09-15, text only partially extractable).
> *Proposes:* a shared benchmark of congestion-control schemes, a common evaluation platform, a
> public result archive, and **calibrated emulators fitted to real Internet paths**.
> *Claims:* schemes "vary dramatically in their relative performance as a function of path
> dynamics" over more than a year of data; the calibrated emulators "closely approximate real-world
> results" and make experiments reproducible.
> *Conditions:* real paths across multiple countries (Brazil, Colombia, US routes were legible in
> the PDF), cellular and wired, plus the emulators. The specific accuracy numbers for the
> emulator calibration **were not extractable** and are not recorded here.
> *Transfers as methodology and is the most relevant entry here to how this project runs
> experiments.* It is prior work making exactly this project's bet — that a deterministic emulator,
> calibrated against reality, is the right substrate for comparing schemes — and it treats the
> calibration itself as a result that has to be demonstrated, not assumed.

## 9. Family G: real-time media transports and what they assume about late data

Short section, one question per entry: **what does this transport do with data that is already too
late to be useful?** The answers differ more than the protocol descriptions suggest, and this is the
axis on which a transport either fits a pose stream or does not.

> **RFC 3550 (RTP)** — see §2. Answer: nothing. RTP timestamps the data and leaves the decision
> entirely to the application. This is why the playout literature exists at all.

> **Perkins & Singer, "Multimedia Congestion Control: Circuit Breakers for Unicast RTP Sessions"**,
> RFC 8083 (2017). <https://www.rfc-editor.org/rfc/rfc8083.html> (accessed 2026-09-15).
> *Proposes:* last-resort shutdown conditions for RTP flows — RTCP timeout (3x the reporting
> interval), media timeout, a congestion breaker that fires when the RTP rate exceeds **ten times**
> the TCP-equivalent throughput `X = s/(Tr*sqrt(2bp/3))`, and a media-usability breaker.
> *Claims:* a specification; the explicit assumption is that RTP senders may have **no congestion
> control at all**, so something must eventually stop a runaway flow.
> *Conditions:* none.
> *Transfers as a safety pattern.* A fixed-cadence sender with no rate control is exactly the case
> this RFC was written for. A 10x-TCP-throughput breaker is a cheap, well-specified guard.

> **Pauly, Kinnear & Schinazi, "An Unreliable Datagram Extension to QUIC"**, RFC 9221 (2022).
> <https://www.rfc-editor.org/rfc/rfc9221.html> (accessed 2026-09-15).
> *Proposes:* `DATAGRAM` frames with no ordering and no retransmission — "not retransmitted upon
> loss detection" — but **still subject to the connection's congestion controller**: a sender "MUST
> either delay sending the frame until the controller allows it or drop the frame without sending
> it". Frames cannot be fragmented; size is bounded by `max_datagram_frame_size`,
> `max_udp_payload_size` and the path MTU.
> *Claims:* a specification.
> *Conditions:* none.
> *Transfers directly and carries a trap.* This is the natural modern substrate for a pose stream —
> until you notice that "delay sending until the controller allows it" converts a congestion
> response into *added latency on a deadline-bound datagram*, which is the one thing this traffic
> cannot absorb. The RFC says nothing about applications that cannot use late data.

> **"Media over QUIC Transport"**, draft-ietf-moq-transport-21 (September 2026).
> <https://datatracker.ietf.org/doc/html/draft-ietf-moq-transport> (accessed 2026-09-15).
> *Proposes:* a publish/subscribe model of Tracks / Groups / Subgroups / Objects over QUIC or
> WebTransport, with two-tier priorities (subscriber priority, then publisher priority), and —
> the relevant part — explicit **`OBJECT_DELIVERY_TIMEOUT` and `SUBGROUP_DELIVERY_TIMEOUT`**, after
> which objects are dropped (datagrams) or streams reset, plus a `TOO_FAR_BEHIND` termination for
> subscribers that fall behind.
> *Claims:* lower latency than TCP-based alternatives by using parallel QUIC streams to avoid
> head-of-line blocking.
> *Conditions:* a draft specification, no measurements.
> *Transfers as vocabulary.* A per-object delivery deadline enforced by the transport is the
> cleanest statement in this section of "late data is worthless"; if this project ever needs to
> describe its own drop-if-late behaviour in standard terms, this is the terminology.

> **Sharabayko, Sharabayko, Dube, Kim & Kim, "The SRT Protocol"**, draft-sharabayko-srt-01
> (7 September 2021; expired, Independent Submission stream — see
> <https://datatracker.ietf.org/doc/draft-sharabayko-srt/>, accessed 2026-09-15). <https://datatracker.ietf.org/doc/html/draft-sharabayko-srt-01>
> (accessed 2026-09-15).
> *Proposes:* Timestamp-Based Packet Delivery (TSBPD) maintaining a **constant end-to-end latency**
> negotiated at handshake (Receiver/Sender TSBPD Delay fields, the larger of the two winning),
> selective ARQ retransmission over UDP, and **Too-Late Packet Drop** — discard packets whose
> delivery deadline has passed rather than delaying the stream.
> *Claims:* recreates the signal's timing characteristics at the receiver.
> *Conditions:* a specification. The draft does not state a recommended latency as a multiple of
> RTT, contrary to common practice folklore; RTT is measured continuously and carried in ACKs.
> *Transfers directly as a design point.* SRT is a **fixed-delay** playout policy with retransmission
> inside the budget — i.e. the baseline this project's Buffering axis already has, plus ARQ. It is
> the clearest existence proof that a fixed playout target is a legitimate production design and not
> merely a straw-man baseline.

## 10. Cloud gaming and XR streaming latency adaptation

The newest and least settled part of the field, and the one closest to this project's application
domain — though almost all of it streams *video to* the user rather than *control from* them.

**Boundary note:** speculative/predictive rendering for cloud gaming (Outatime and its descendants)
is predictive display, which is another file's field. It is noted here only where the mechanism is
buffering rather than prediction.

> **Lee, Chu, Cuervo, Kopf, Degtyarev, Grizan, Wolman & Flinn, "Outatime: Using Speculation to
> Enable Low-Latency Continuous Interaction for Mobile Cloud Gaming"**, ACM MobiSys 2015.
> <https://www.microsoft.com/en-us/research/publication/outatime-using-speculation-to-enable-low-latency-continuous-interaction-for-mobile-cloud-gaming/>
> (accessed 2026-09-15).
> *Proposes:* speculative rendering of possible future frames delivered a full RTT early, with
> misprediction recovery. Listed for completeness and cross-reference only — this is a prediction
> technique, not a playout technique.
> *Claims:* masks up to 120 ms of network latency; users strongly prefer it to a thin client with
> the RTT fully visible.
> *Conditions:* Doom 3 and Fable 3; a user study; the abstract's framing is that cellular, Wi-Fi
> and even wired residential RTTs "can exceed 100 ms". Detailed network conditions not read.

> **Zhao, Wu, Lv, Yang, Zhang, Peng, Liu, Li, Chen, Guo & Xie, "JitBright: towards Low-Latency
> Mobile Cloud Rendering through Jitter Buffer Optimization"**, ACM NOSSDAV 2024, pp. 36-42.
> DOI `10.1145/3651863.3651881`; record verified at
> <https://api.crossref.org/works/10.1145/3651863.3651881> (accessed 2026-09-15).
> *Proposes:* client-side adaptive jitter buffer management for cloud rendering — the authors name
> the two mechanisms as **"adaptive gain" and "proactive keyframe requests"** — the latter to break
> frame dependencies that would otherwise force the buffer to stay deep.
> *Claims:* from the abstract, verbatim in substance: a real-world measurement study of their own
> production cloud-rendering system finds "the primary factor causing increased motion-to-photon
> (MTP) latency is the receive-to-composition (R2C) latency at the client, which is primarily
> caused by the ineffective jitter buffer management strategy"; JitBright then "**increases the
> proportion of sessions that meet MTP latency requirements by 6%-27%**" while improving playout
> smoothness.
> *Conditions:* **abstract only — the full text was not retrieved.** What the abstract does give:
> **large-scale A/B tests involving over 12,000 users** on a production mobile cloud-rendering
> service. It gives no network profile, no delay or loss model, no frame rate, no device class, and
> no definition of the "MTP latency requirement" whose satisfaction rate is the headline number.
> **The 6%-27% figure must not be repeated as if its conditions were known**; the range itself
> implies the result varies a great deal across whatever segments were measured, and which segments
> those are is exactly what is missing.
> Author affiliations, from OpenAlex (accessed 2026-09-16): **Alibaba Group** (Hangzhou) together
> with the Institute of Computing Technology, Chinese Academy of Sciences, Purple Mountain
> Laboratories and UCAS. The paper is recorded as **gold open access**, yet every ACM DL path —
> landing page, `/doi/pdf/`, `/doi/fullHtml/` — returns HTTP 403 to this environment, and Semantic
> Scholar lists no arXiv identifier (DBLP key `conf/nossdav/ZhaoWLYZPL0CGX24`, Corpus ID 268733528),
> so there is no preprint to fall back on. See §14. It remains the single most on-topic recent
> source in this file and the one whose conditions are least known.

> **Casasnovas, Michaelides, Carrascosa-Zamacois & Bellalta, "Experimental Evaluation of Interactive
> Edge/Cloud Virtual Reality Gaming over Wi-Fi using Unity Render Streaming"**, arXiv:2402.00540v2
> (August 2024). <https://arxiv.org/html/2402.00540> (accessed 2026-09-15).
> *Proposes:* a measurement study rather than an algorithm.
> *Claims:* on Wi-Fi 6 (802.11ax, 5 GHz, 80 MHz, RSSI -46 dBm, two spatial streams, 50 Mbps fixed
> bitrate, ~600 Mbps measured capacity), RTT averaged 1.5 ms with 99.99th-percentile 4-5 ms; video
> jitter fell from 20 ms at 30 fps to 7 ms at 90 fps; frame assembly delay fell from ~21 ms at
> 30 fps to ~6 ms at 90 fps. ITU-T VR QoS targets (RTT < 20 ms, jitter < 15 ms, zero loss) were met
> at 60 and 90 fps.
> *Conditions:* a single-AP local Wi-Fi testbed with no WAN impairment. **This is the opposite end
> of the operating range from this project's profiles** and its numbers must not be read as
> applicable to a 300 ms path.
> *Transfers as one useful structural point:* frame assembly delay falls with frame rate because a
> frame occupies fewer packet-times. The analogous claim for a pose stream is that a 90-100 Hz
> cadence makes per-sample assembly delay negligible and pushes essentially all variance into the
> network — which is an assumption this project could check rather than inherit.

> **LiveKit, "Low-latency video and data" (robotics documentation)**.
> <https://docs.livekit.io/robotics/media/performance/low-latency/> (accessed 2026-09-15).
> *Proposes:* two practitioner-level knobs for teleoperation media — **playout delay hints**
> (minimum and maximum playout delay in ms, applied per subscriber) and a **zero jitter buffer**
> mode. The stated rationale is that a teleoperator "needs to see what the robot sees now", so the
> usual buffering tradeoff is weighted differently than for media consumption.
> *Claims:* zero-jitter-buffer gives lowest latency "at the cost of less smooth playback on poor
> networks". No numbers.
> *Conditions:* **product documentation with no measurements at all.** Included for one reason: it
> is direct evidence that the commercial teleoperation stack's answer to this question in 2026 is a
> *manually bounded* playout delay, not an adaptive policy. That is the state of practice this
> project's Buffering axis is measuring against, and it is a low bar.

## 11. The question this list was built to answer

**Which adaptation family has the best-documented behaviour under bursty loss and reordering, and
under what conditions were those results obtained?**

The honest answer is that **no family in this literature is well documented under bursty loss
combined with reordering**, and the most useful thing this reading produced is a precise account of
*why not* and *who comes closest*.

### What the evaluation standards actually specify

> **Sarker, Singh, Zhu & Ramalho, "Test Cases for Evaluating Congestion Control for Interactive
> Real-Time Media"**, RFC 8867 (January 2021). <https://www.rfc-editor.org/rfc/rfc8867.html>
> (accessed 2026-09-15).
> *Proposes:* the standard scenario set for real-time-media congestion control. Reference bottleneck
> capacity 1 Mbps with 0.5x-4.0x ratios; default one-way delay **50 ms** with some cases spanning
> 10-150 ms; default queue 300 ms, tail-drop; recommended maximum end-to-end jitter 30 ms; required
> reporting of end-to-end delay, sending-rate variation, losses at the receiver, convergence time
> and feedback overhead, logged at roughly 200 ms granularity.
> *Claims:* a common basis for comparing schemes.
> *Conditions:* **"Path loss ratio: 0%" is the baseline.** The standard test cases do not specify
> burst loss or reordering patterns.

> **Singh, Ott & Holmer, "Evaluating Congestion Control for Interactive Real-Time Media"**,
> RFC 8868 (January 2021). <https://www.rfc-editor.org/rfc/rfc8868.html> (accessed 2026-09-15).
> *Proposes:* the evaluation guidelines. Propagation delays in four bands — very low (0-1 ms), low
> (50 ms), high (150 ms), **extreme (300 ms)**. Loss to be tested as **independent random loss** at
> 0%, 1%, 5%, 10%, 20%. Queue depths 70 ms (QoS-aware), 300-500 ms (nominal), 1000-2000 ms
> (bufferbloated). Packet delay variation modelled as "Approximately Random Subject to No-Reordering
> Bounded PDV" — a truncated Gaussian, 5 ms standard deviation, clipped at ±3σ, **deliberately
> constructed so that reordering cannot occur**. Video sources are real encoders (Foreman CIF,
> FourPeople 720p); cross-traffic is long-lived CUBIC plus short TCP bursts.
> *Claims:* guidelines, not results. The document notes "more sophisticated loss models could be
> considered", naming Gilbert-Elliott for correlated loss, but does not require it.
> *Conditions:* the document *is* the conditions.
> *This is the most consequential single fact in this file.* The entire body of real-time-media
> congestion-control evaluation — NADA, SCReAM, GCC and everything benchmarked against them — is
> standardised on **independent random loss with reordering explicitly excluded by construction**.
> Its delay bands do reach 300 ms, which matches this project's range; its loss model does not
> match a burst-loss profile at all.

### Ranking the families by quality of documentation in the bursty/reordered regime

1. **NetEQ (Family E) is the best-documented on reordering, by a distance, and the documentation is
   source code rather than a paper.** It is the only scheme in this file with a *separate estimator
   for reordering*, with its own histogram, its own sample population (reordered packets only,
   in-order packets pinned to bucket 0) and its own objective
   (`delay_ms + 100 * ms_per_loss_percent * loss_probability`), combined with the underrun estimate
   by `max`. **Conditions:** production deployment in Chrome and every libwebrtc-based product; no
   controlled experiment, no published delay-versus-late-loss curve, no stated trace set. So it is
   well *specified* under reordering and essentially un-*evaluated* in public.

2. **The RTCP XR burst/gap family (§7) is the best-documented on bursty loss** — but it is
   measurement vocabulary, not an adaptation scheme. RFC 6958 and RFC 7003 give a standard,
   threshold-parameterised decomposition of loss and of *discard* into bursts and gaps, with the
   explicit rationale that an average rate is the wrong instrument. **Conditions:** specifications;
   Gmin=16 is calibrated to 20 ms voice packets and does not transfer numerically to 90-100 Hz.

3. **Sprout has the best-documented real-world operating point of any adaptation scheme here** —
   trace-driven replay of four commercial LTE and 3G networks — but cellular impairment is delay
   variation over deep buffers with link-layer ARQ hiding loss. It is strong evidence about
   *variable delay*, weak evidence about *burst loss*, and silent on reordering.

4. **Copa, Cross and the RMCAT-evaluated schemes (NADA, SCReAM, GCC) are documented under random
   loss only.** Copa's robustness experiment is stochastic loss; Cross's abstract names random loss;
   NADA and SCReAM are evaluated against RFC 8867/8868, which is random loss with reordering
   designed out. GCC's draft contains no controlled evaluation of its own at all.

5. **BBRv3 asserts reordering-robustness in prose** ("robust to packet reordering", makes no
   ordering assumptions) **with no experiment in the draft**, and its own text says it handles
   persistently application-limited traffic badly — which is what a fixed-cadence pose stream is.

6. **The classical playout families (A-D) do not treat reordering as a distinct phenomenon**; a
   reordered packet is simply a late packet, which is a defensible modelling choice for
   talkspurt-boundary adaptation and a poor one for a 10 ms sample interval. Nothing recovered in
   the 2026-09-16 pass changes this: not one of Ramjee 1994, Liang 2003, Kansal 2001,
   Miranda-Campos 2008, GARCH 2010 or Boi 2007 mentions reordering at all. **On the reordering
   half of the question the ranking above is unchanged.**

7. **On burst loss, the ranking above understated the classical playout families, and this is the
   substantive correction from the 2026-09-16 recovery pass.** Three of those papers were read in
   full for the first time, and two of them report clustered loss as a **first-class result with
   its own curve**, not as an aside:
   - **Liang, Färber & Girod 2003** (§3) defines a **burst loss rate** — the rate of packets
     belonging to a run of two consecutive losses — reports it separately "because they are more
     difficult to conceal and impair sound quality more severely", and **plots it against average
     buffering delay alongside the ordinary loss curve**. Its strongest claimed result is a
     clustering result, not a rate result: burst loss from 12% to 1% at 40 ms of buffering on its
     Trace 1, and still 3.9 points lower at 10 ms on the trace where the ordinary delay/loss gain
     was smallest.
   - **Zhang et al. 2010** (§4) goes further: its "Direct GARCH" parameter fit **replaces the
     likelihood with a cost function over the drop inter-arrival distribution**, so the estimator
     is fitted to avoid clustering; and it reports **consecutive PLR as a function of the target
     loss rate, and again as a function of buffering delay** — two more curves of clustering
     against the operating point.
   **The precise thing that changed, stated carefully, because the distinction matters.** These
   papers do not evaluate under an injected correlated *network*-loss model — no Gilbert-Elliott,
   no two-state chain, no burst-loss profile. What they evaluate is the **burstiness of the
   discard process the playout policy itself produces** on real delay traces. That is a different
   object from a bursty path, and it is arguably the more useful one for a receiver-side policy:
   it says the field already knew that *when* you lose samples matters as much as *how many*, and
   it already had the instrument. So the honest revision is that **the classical playout literature
   is better documented on clustered late loss than this file previously recorded, while remaining
   undocumented under correlated network loss and silent on reordering.** The §13 search that found
   nothing evaluating a playout algorithm under a correlated-loss model *and* a reordering model
   together still found nothing.

8. **A related correction on the congestion-control side.** Carlucci et al. 2016 and De Cicco et
   al. 2013 were also read in full for the first time, and they make item 4 above stronger rather
   than weaker: **neither injects loss at all.** In both, loss appears only as an outcome metric
   (self-inflicted congestion loss), and reordering is never mentioned. So the statement "the
   RMCAT-evaluated schemes are documented under random loss only" is, for GCC's own peer-reviewed
   evaluations, better put as **documented under no injected loss at all**.

### What to run first, given the above

The cheapest falsifiable experiment this reading suggests for the Buffering axis is
**NetEQ's two-estimator split**: a forgetting-histogram quantile tracker for underrun, plus a
*separate* reorder-cost minimiser, combined by `max`, versus the existing adaptive policy with a
single estimator. The falsifier is clean — if the split does not beat the single estimator on a
reordering-heavy profile, the extra estimator is dead weight, and that is a publishable negative
result. Holding the predictor and reconciler fixed while varying only the buffering policy is
required (coupled axes), and the baseline rows (`none`, `snap`) must be present.

The second cheapest is **adopting RFC 4737's late-time offset** as a reported quantity on
reordering profiles, since it is a definition rather than an algorithm and costs nothing to be
wrong about.

## 12. Operating-point convention: target late-loss rate versus delay tail

The task asked what the field's convention is for choosing a playout operating point, because this
project reports p50/p95/p99 and reads verdicts off **p99** on the stated assumption that the tail is
what the operator perceives. Stating what prior work does, under its conditions, without drawing a
conclusion about this system:

**The playout literature overwhelmingly selects the operating point by a target late-loss (or
discard) rate, and reports the resulting delay — not the other way round.** The readable evidence:

- **Ramjee et al. (1994)**, read in full on 2026-09-16, is the exception and it is the field's
  founding paper. **It has no target loss rate at all.** Its curves are parameterised by the
  **maximum buffer size**, swept from 160 bytes to 4 KB, each setting yielding one
  (average playout delay, % loss) point; a second plot gives loss directly against buffer size.
  The only loss-rate statement is a tolerance band inherited from the codec literature — "packet
  loss rates of between 1 and 10% can be tolerated" — used to mark the "range of loss rates of
  interest", not to set an operating point. **So the target-late-loss convention is not present at
  the origin of this literature; it arrives later.** Note also that its reported delay is the mean
  over *successfully played-out* packets minus the trace's minimum delay — a queuing delay above
  the propagation floor, computed over a sample set that changes with the algorithm.
- **Concord** (Sreenan et al., 2000) exposes the delay/lateness exchange rate as an
  *application-set* parameter and reports results at lateness below 1%. The application declares the
  loss it will tolerate; the algorithm returns a delay.
- **Liang, Färber & Girod (2003)**, read in full on 2026-09-16, is the clearest statement of the
  convention in its mature form. The playout deadline is taken from an order statistic selected by
  a **user-specified loss rate**, and the authors say the point of this explicitly: "the user can
  specify the acceptable loss rate ... and the algorithm automatically adjusts the delay
  accordingly. Therefore, the tradeoff between buffering delay and late loss can be controlled
  explicitly." **What they report is not a point but a curve**, produced by sweeping each
  algorithm's own control parameter, with loss rate against **average** buffering delay. There is
  no delay percentile anywhere in the paper. Headline comparisons are quoted at a **5% target late
  loss**, and separately at a fixed 40 ms delay.
- **Direct GARCH** (Zhang et al., 2010), read in full on 2026-09-16, is a parameter-estimation
  procedure designed "to implement a desired packet loss rate while minimizing the probability of
  consecutive packet losses". Concretely, the desired loss rate `v` is swept over **1-5%**, giving
  a quantile `w = 1 - v` of **0.95 to 0.99** taken from a Laplacian whose variance the GARCH model
  forecasts. The target is a rate; the secondary objective is about *clustering* of losses, not
  about a delay percentile. **And it adds a question the rest of the field does not ask:** it
  reports **"PLR error", the absolute difference between the target loss rate and the achieved
  one**, as a headline result with its own figure, and the authors call closeness to target "the
  main advantage of the proposed algorithm" — ahead of any position on the delay/loss curve. The
  delay/loss curve itself separates the linear filter from everything else, but barely separates
  GARCH from a neural predictor.
- **NetEQ** sets target delay to a **quantile of the delay histogram** — the webrtcHacks walkthrough
  gives 0.95 as the worked example. A delay quantile and a late-loss rate are duals: choosing the
  0.95 quantile of observed arrival delay is choosing to accept roughly 5% underrun, expressed on
  the delay axis. The knob the operator turns is still "what fraction am I willing to lose".
- **NetEQ's reorder path** does not use a quantile at all; it minimises a weighted sum of delay and
  loss probability, i.e. it picks the operating point from an explicit exchange rate
  (`ms_per_loss_percent`).
- **RFC 3611** reports the buffer as a *triple* — nominal, maximum and absolute-maximum delay —
  plus an adaptive flag, alongside separate loss and discard rates. It does not define a single
  headline delay percentile.
- **The quality-model family** (Atzori & Lobina's third category; Sakir & Feldbauer) does neither:
  it maximises a predicted quality score that is a function of both delay and loss, so the operating
  point falls out of the model. The justification offered in that literature is that a speech
  quality model (the ITU-T E-model) makes milliseconds and loss percentage points commensurable.

**Why the field justifies it that way, as stated in the readable sources.** Two reasons appear. The
first is concealment: a voice decoder can conceal a bounded fraction of missing frames, so a
tolerable loss rate is a property of the codec and is knowable in advance, whereas a tolerable delay
is a property of the conversation and is not. The second is the burstiness argument of RFC 6958 —
that "the burstiness of packet loss affects user experience" beyond the average rate — which is an
argument for reporting the *clustering* of late events, a statistic that neither a mean nor a single
delay percentile captures.

**What the 2026-09-16 recovery pass changed about this section.** The convention above was stated
largely from modern readable sources — shipping NetEQ code, RFCs, and abstracts — and inferred
backwards onto the classical literature. Four of those classical sources have now been read in
full, and the result both confirms and complicates the picture:

- **Confirmed, and more strongly than before.** From 2000 onward the convention is explicit and the
  authors argue for it in their own words: Concord makes lateness an application-set parameter,
  Liang et al. say the user specifies the acceptable loss rate so the tradeoff "can be controlled
  explicitly", and Zhang et al. fit the model directly to a desired loss rate. None of the four
  reports a delay percentile. All four report the operating point on the **loss** axis and the
  consequence on the **average delay** axis.
- **Complicated at the origin.** Ramjee et al. (1994) does not do this. Its sweep parameter is a
  buffer size in bytes, which is an *implementation* limit rather than a quality target, and the
  loss band it cares about is inherited from what a codec can conceal. The convention is therefore
  a development of the field, roughly 1994 → 2000, not an axiom of it. That matters for anyone
  arguing "the field has always done X": it has not.
- **Complicated again at the other end.** The **quality-model family chooses neither**. Boi, Atzori
  & Lobina (§5) state it flatly: "no thresholds for the loss and/or delay have to be considered",
  because the E-model supplies the exchange rate and the optimum falls out. Their delay impairment
  is **piecewise linear with a knee at 177.3 ms**, above which the per-millisecond penalty jumps by
  roughly a factor of five. So within the playout literature there are **three** conventions —
  sweep an implementation limit, target a loss rate, or optimise a scalarised quality model — and
  the third one's behaviour is dominated by where its knee sits.
- **And the neighbouring field uses the opposite one.** Carlucci et al. (2016) and De Cicco et al.
  (2013) — the two peer-reviewed evaluations of the real-time-media congestion controller (§8) —
  report the **50th and 95th percentiles of queuing delay**, measured as `RTT(t) - RTT_min`, with
  loss as an *outcome* metric. That is a delay-tail convention. The playout literature and the
  real-time congestion-control literature, which share authors, venues and a decade, **do not share
  a reporting convention**, and a number lifted from one cannot be compared to a number from the
  other without saying which axis was the independent variable.

**A sample-set warning that fell out of reading the primaries, and that this project should take
more seriously than the convention question.** Both Moon et al.'s metric definitions (as reproduced
verbatim by Miranda-Campos & Ramos, §4) and Ramjee et al.'s define **average playout delay over the
played-out packets only**. A policy that discards more samples therefore averages its delay over a
smaller and differently-selected population than one that discards fewer — so the delay axis of
every one of these curves is computed over a sample set that *moves as you slide along the curve*.
That is precisely the failure the root `CLAUDE.md` warns about ("the percentiles compare populations,
not algorithms"), sitting unremarked inside the founding metric definitions of this field. It is not
a reason to distrust the curves — the loss axis tells you exactly how much the population moved —
but it is a reason never to quote one of these average delays on its own.

**One asymmetry worth recording without editorialising.** A target-loss-rate convention and a
delay-tail convention answer different questions. A delay percentile such as p99 describes the
latency of the samples that *were* played out. A late-loss rate describes the samples that were
*not*. A policy can improve one while worsening the other, and the classical literature's answer —
report the whole delay-versus-loss curve rather than a point on it (§1) — is the only form that is
immune to that. Prior work reports curves; this project reports percentiles. Whether that matters
here is a question for a sweep, not for this file.

## 13. Searches that found nothing

Recorded so the next person does not repeat them. Queries are verbatim.

- `adaptive playout buffer evaluation under Gilbert-Elliott bursty loss and packet reordering VoIP
  comparison` — **no source found that evaluates a playout algorithm under a correlated-loss model
  and a reordering model together.** Returned the Gilbert-Elliott model itself, generic playout
  surveys, and a run of US patents. This is the gap identified in §11 and I could not close it.
- `adaptive playout buffering for teleoperation pose stream 90 Hz robot arm jitter buffer latency`
  — **no academic source found on playout buffering for a pose/command stream.** Results were
  patents on generic jitter buffers, humanoid teleoperation papers concerned with control rather
  than transport, and one piece of vendor documentation (LiveKit, §10). If an adaptive playout
  policy for a 90-100 Hz pose stream has been published and evaluated, this search did not find it.
- `cloud gaming frame pacing adaptive playout buffer "queuing delay" arxiv measurement 2022 latency
  variance` — found relevant work (SQP, PASync, JitBright) but **every ACM-published item was
  paywalled**; no open evaluation of a cloud-gaming playout buffer was retrievable.
- `"Google congestion control" GCC evaluation testbed "trendline" bandwidth estimation experimental
  conditions paper pdf poliba carlucci` — located the papers but **not a readable copy of any of
  them**; the lab-hosted PDF is now a 404. **Resolved 2026-09-16 without a search engine**: the
  lab site migrated from MediaWiki to Drupal, so the file simply moved. Fetching
  <http://c3lab.poliba.it/publications> and following its links reached both the MMSys 2016 and
  ToN 2017 papers. *The general lesson: when a lab-hosted PDF 404s, fetch the lab's publications
  page rather than searching for the file.* Note that the search's premise was also wrong — the
  papers do not contain a trendline estimator (§8).
- `Moon Kurose Towsley playout delay adjustment UMass technical report 98 postscript pdf "packet
  audio"` — **no open-access copy of the 1998 Multimedia Systems paper exists** that this
  environment can read; the UMass technical-report PDF that surfaced is a different report (its
  text *is* extractable with `pdftotext`, and reading it confirmed it is the clock-skew paper).
  **Re-attempted exhaustively on 2026-09-16 and still failed** — the full route list is in §14.
  OpenAlex and Semantic Scholar both confirm the paper has **no open-access location at all**, so
  this is not a search problem and further searching will not help. What did help was reading the
  papers that used its traces: see §2.
- **A whole-category note, 2026-09-16:** searching for an *author-hosted* copy is far less
  productive from this environment than **asking a metadata API where the repository copies are**
  (`api.openalex.org/works/doi:<DOI>` returns every known location and the OA status) and then
  **fetching lab and co-author pages and reading their outbound links**. Both of this pass's
  best recoveries — Ramjee et al. 1994 via Schulzrinne's Columbia archive, and the GCC papers via
  the Politecnico di Bari lab site — came from link-following, not from search.

## 14. Sources that could not be retrieved

The failures cluster by host, and the pattern is worth recording on its own: **publisher landing
pages for ACM, IEEE, Elsevier and Wiley are uniformly unavailable.**

**Update, 2026-09-16 — the "no extractable text layer" class of failure is solved and should not be
recorded again.** The original pass concluded that pre-2010 PDFs "generally resolve but have no
extractable text layer", and treated that as a property of the documents. It is not: it is a
property of the fetch tool's built-in extractor. When that extractor fails it saves the raw PDF to
a local scratch path and prints it, and **`pdftotext` (poppler, present on the Linux box) reads
those files without difficulty** — including 1994-vintage documents. Seven sources previously
recorded here as unreadable were read in full this way: Ramjee et al. 1994, Kansal & Karandikar
2001, Liang et al. 2003, Miranda-Campos & Ramos 2008, Zhang et al. 2010, De Cicco et al. 2013 and
Carlucci et al. 2016. **Anyone continuing this file should try that before recording a text-layer
failure.** (Figures, and tables typeset as figures, remain unrecoverable — Liang et al.'s trace
table is still lost this way, and the equations in older papers lose their subscripts and
occasionally their signs.)

**Two retrieval routes that the task for this pass recommended and that do not work here, recorded
so nobody spends the time again:**

- **The Internet Archive is unreachable in its entirety.** `web.archive.org` is refused by the
  fetch tool itself; `scholar.archive.org` serves an anti-scraping gate; `api.fatcat.wiki` refuses
  the TCP connection outright (`ECONNREFUSED 207.241.225.9:443`). This matters more than it sounds,
  because **CiteSeerX has been retired and now 301-redirects its entire corpus into the Wayback
  Machine** — so every CiteSeerX link in the older literature is, from here, a dead end.
- **General web search is not available for this kind of lookup.** The session's search budget was
  exhausted, and every directly-fetched search engine refused: DuckDuckGo (HTML and lite endpoints)
  serves a CAPTCHA, Mojeek 403, CORE 403, BASE 403 ("Anubis"), Marginalia imposes a bot wait.
  **The routes that did work were metadata APIs plus link-following**: `api.openalex.org` (which,
  unlike Crossref and Semantic Scholar, lists repository locations and open-access status and was
  the single most useful tool in this pass), `api.crossref.org`, `api.semanticscholar.org/graph/v1`,
  and then following the URLs those returned. Two recoveries came from simply **fetching a lab or
  co-author page and reading its links**: the Microsoft Research landing page for Ramjee et al.
  links to Henning Schulzrinne's Columbia archive, and Politecnico di Bari's lab site has moved
  from MediaWiki to Drupal, so the GCC papers are alive at new paths behind
  <http://c3lab.poliba.it/publications>.

| Source | URL attempted | What happened |
|---|---|---|
| **Moon, Kurose & Towsley 1998** (playout bounds) — **STILL FAILS; every route below was tried** | `link.springer.com/article/10.1007/s005300050073` | HTTP 303 to an auth endpoint; the follow-up 302'd again with `error=cookies_not_supported` |
| same, Springer direct PDF | `link.springer.com/content/pdf/10.1007/s005300050073.pdf` | HTTP 303 to `idp.springer.com/authorize` |
| same, CiteSeerX (two URL forms) | `citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a9f2a0f2...`, `.../viewdoc/summary?doi=10.1.1.109.4870` | first `ECONNREFUSED 130.203.135.69:443`, then HTTP 301 into `web.archive.org` — **CiteSeerX is retired and its whole corpus now lives only in the Wayback Machine**, which is unreachable here |
| same, open-access lookup | `api.openalex.org/works/doi:10.1007/s005300050073`, `api.semanticscholar.org/.../DOI:10.1007/s005300050073?fields=openAccessPdf` | both report **no open-access location exists**; `oa_status: closed` |
| same, ScholarWorks@UMass (the OA record OpenAlex points at) | `scholarworks.umass.edu/cs_faculty_pubs/721`, `scholarworks.umass.edu/handle/20.500.14394/10325` (via `hdl.handle.net`) | HTTP 403 on both — appears to be a whole-domain block |
| same, UMass technical-report server | `web.cs.umass.edu/publication/docs/1998/UM-CS-1998-043.pdf` | **resolves, but it is a different paper** — "Estimation and Removal of Clock Skew from Network Delay Measurements" (Moon, Skelly, Towsley, TR 98-43). A search engine asserted this was the playout paper; it is not. The TR index at `web.cs.umass.edu/publication/` is a POST form whose parameters could not be determined without raw HTML access |
| same, author and group pages | `www-net.cs.umass.edu/networks/publications.html`, `gaia.cs.umass.edu/papers/papers.html`, `www-net.cs.umass.edu/kurose/index.html`, `an.kaist.ac.kr/~sbmoon/` | group publication lists resolve but are too long for the fetch tool to return the 1990s entries; Kurose's page errors with "Could not open input file: make_toc.php"; Sue Moon's KAIST page routes publications to Google Scholar. Her CV (`an.kaist.ac.kr/~sbmoon/resume.pdf`) **is** readable and confirms the bibliographic record, but carries no URL or TR number |
| same, aggregators | `core.ac.uk/search`, `base-search.net`, `scholar.archive.org`, `api.fatcat.wiki` | 403, 403 ("Anubis"), anti-scraping gate, `ECONNREFUSED` |
| **→ partial recovery** | — | **its algorithm (`L = 10,000` delay histogram, playout at a chosen percentage), its bound result (gap to optimum widens below 2% loss), its six traces and its metric definitions are now recorded in §2 and §4, secondhand from Kansal & Karandikar 2001 and Miranda-Campos & Ramos 2008, both of which were read in full** |
| **Atzori & Lobina 2006 survey** — **STILL FAILS; highest-value gap in this file** | `ieeexplore.ieee.org`, `dl.acm.org` | empty body / HTTP 403 |
| same, open-access lookup | `api.openalex.org/works/doi:10.1109/COMST.2006.253269`, Semantic Scholar Graph API | **no open-access location of any kind**; `oa_status: closed`; abstract elided by both |
| same, the authors' own institutional repository | `hdl.handle.net/11584/26155` → `iris.unica.it/handle/11584/26155` | record resolves, **metadata only** — "Non ci sono file associati a questo prodotto" |
| same, sibling papers by the same authors | `10.1109/MSAN.2005.1489941`, `10.1109/tmm.2005.864348` (→ `hdl.handle.net/11584/31975`) | both closed, no PDF |
| **→ partial substitute** | `eudl.eu/pdf/10.4108/ICST.MOBIMEDIA2007.1742` | **Boi, Atzori & Lobina's 2007 MobiMedia paper is gold OA, resolves, and was read in full** (§5). Same authors, same family, real GEO-satellite traces at 354-400 ms average delay. It is one instance, not the survey's common footing |
| ~~Liang, Färber & Girod 2003~~ — **RECOVERED 2026-09-16** | `web.stanford.edu/~bgirod/pdfs/LiangMM2003.pdf` | resolves; the fetch tool's extractor fails but `pdftotext` reads it cleanly. **Read in full**, see §3 |
| ~~Zhang et al. 2010 (GARCH)~~ — **RECOVERED 2026-09-16** | `cl.cam.ac.uk/~awm22/publications/zhang2010garch.pdf` | same — `pdftotext` reads it. **Read in full**, see §4 |
| same, publisher | `sciencedirect.com/science/article/abs/pii/S1389128610001726` | HTTP 403 (irrelevant now — the Cambridge copy is readable) |
| Sreenan et al. 2000 (Concord) — **still fails** | `ieeexplore.ieee.org/document/845013/` | empty body. Not re-attempted in depth this pass; Concord's design is described secondhand and consistently by Zhang et al. 2010, which was read |
| ~~Kansal & Karandikar 2001~~ — **RECOVERED 2026-09-16** | `microsoft.com/.../kansal_globecom01.pdf` | `pdftotext` reads it. **Read in full**, see §3 |
| ~~Miranda-Campos & Ramos 2008~~ — **RECOVERED 2026-09-16** | `scitepress.org/papers/2008/19407/19407.pdf` | `pdftotext` reads it. **Read in full**, see §4 — and it reproduces Moon et al. 1998's trace table |
| ~~Carlucci et al. 2016 (GCC analysis)~~ — **RECOVERED 2026-09-16** | old: `c3lab.poliba.it/images/6/65/Gcc-analysis.pdf` (404, dead MediaWiki path); **live: `c3lab.poliba.it/sites/default/files/2026-05/Gcc-analysis.pdf`** | the lab migrated to Drupal and the files moved; both the MMSys 2016 paper and the ToN 2017 journal version are linked from `c3lab.poliba.it/publications`. **Both read in full**, see §8. Reading it **retracted a wrong claim** this file previously made about the paper proposing a trendline estimator |
| same, ACM | `dl.acm.org/doi/10.1145/3232755.3232783` and others | HTTP 403 (irrelevant now) |
| ~~De Cicco et al. 2013 (GCC experiments)~~ — **RECOVERED 2026-09-16** | `conferences.sigcomm.org/sigcomm/2013/papers/fhmn/p21.pdf` | `pdftotext` reads it. **Read in full**, see §8 |
| **GCC's trendline estimator — a newly identified gap** | — | the libwebrtc delay signal in current use is a linear-regression **trendline**, but neither Carlucci et al. 2016 nor the ToN 2017 extension describes it (neither contains the words "trendline" or "linear regression"), and draft-ietf-rmcat-gcc-02 documents the Kalman filter. **This file has no peer-reviewed evaluation of the estimator libwebrtc actually ships.** The only primary source located is the libwebrtc source tree itself, which was not read in this pass |
| Cardwell et al., BBRv1 (ACM Queue) | `queue.acm.org/detail.cfm?id=3022184` | HTTP 403 |
| **JitBright (NOSSDAV 2024)** — **STILL FAILS except for its abstract** | `dl.acm.org/doi/10.1145/3651863.3651881`, `dl.acm.org/doi/pdf/...`, `dl.acm.org/doi/fullHtml/...` | HTTP 403 on all three, **despite OpenAlex recording the paper as gold open access** |
| same, preprint search | `api.semanticscholar.org/.../DOI:10.1145/3651863.3651881?fields=externalIds` | **no arXiv identifier exists** (only DBLP `conf/nossdav/ZhaoWLYZPL0CGX24` and Corpus ID 268733528), so there is no preprint to fall back on. Authors are Alibaba Group plus ICT/CAS; no author-hosted copy was reachable without a search engine |
| **→ partial recovery** | Semantic Scholar and OpenAlex abstracts | **the abstract is now readable and recorded in §10**, including the "6%-27%" headline and the 12,000-user A/B test. Its conditions remain entirely unknown |
| PASync (NOSSDAV 2026) | `dl.acm.org/doi/pdf/10.1145/3798065.3798067` | HTTP 403 |
| Kelkkanen, synchronous remote rendering for VR | `onlinelibrary.wiley.com/doi/10.1155/2021/6676644` | HTTP 403 despite being open access |
| Joint playout+FEC adjustment (IEEE INFOCOM 2003, paper 16_03) | `infocom2003.ieee-infocom.org/papers/16_03.PDF` | HTTP 418 |
| Copa (full conditions) | `usenix.org/system/files/conference/nsdi18/nsdi18-arun.pdf` | PDF partially extractable — section structure read, numeric conditions not. **Not re-attempted in the 2026-09-16 pass** (that pass was scoped to the classical playout gap); the `pdftotext` route above would very likely close this row and should be tried first |
| Pantheon (emulator calibration numbers) | `pantheon.stanford.edu/.../pantheon-paper.pdf` | PDF partially extractable — paths read, accuracy figures not. **Same: not re-attempted, and likely recoverable by the same route** |
| Vegas (full conditions) | `cs.princeton.edu/courses/archive/fall06/cos561/papers/vegas.pdf` | recorded in §8 as resolving without extractable text. **Same: likely recoverable by `pdftotext`, not re-attempted** |
| dblp records | `dblp.org/rec/...` | blocked by the site's "Anubis" bot protection |
| Semantic Scholar paper pages | `semanticscholar.org/paper/...` | consistently returned an empty body; **the Graph API at `api.semanticscholar.org` works** and was used instead, though it rate-limits (HTTP 429) under bursts and elides publisher abstracts |
| ITU-T G.114 | `itu.int/rec/T-REC-G.114/en` | landing page readable (version 05/2003 confirmed) but the recommendation text, and therefore the one-way-delay limits, are not on it. RFC 6817 refers to a 150 ms voice-quality threshold from G.114; that attribution is secondhand and was not checked against G.114 itself |

One invented-reference near-miss is worth recording as a method note: an arXiv identifier guessed
from memory (`arxiv.org/abs/1802.01329`) resolved to a real paper on the reshaping of Janus rings in
nematic elastomers, nothing to do with congestion control. Every identifier in this file was
obtained from a search result or a metadata API and then fetched, never recalled.

**Two further method notes from the 2026-09-16 pass, both failures of the same kind.**

1. **A search engine's summary asserted a wrong identity and was believed for one step.** Asked for
   the Moon/Kurose/Towsley technical report, a search result stated that
   `web.cs.umass.edu/publication/docs/1998/UM-CS-1998-043.pdf` "is" that paper. It is not — it is
   the clock-skew paper by Moon, Skelly & Towsley. Fetching and reading the first page caught it
   immediately. **The lesson is the one already in this file: a search engine's prose summary is
   not a verification, and the only verification is opening the document and reading its title
   block.**
2. **An OpenAlex author ID was constructed rather than looked up** (`A5017596461`, reached for while
   trying to enumerate Atzori's works). It returned zero results, so nothing false entered the
   file, but it was the same error as the arXiv one and could equally have resolved to a real,
   wrong author. The correct ID was afterwards taken from the paper record's own `authorships`
   field. **Identifier construction is the recurring failure mode of this activity, and it does not
   stop being one just because the identifier is from a metadata service rather than a memory.**

## 15. What this file argues should change elsewhere

Written here rather than in the files concerned, per the survey-run constraint. These are arguments,
not changes, and none of them is settled by literature alone.

1. **`docs/metrics.md` has no definition of late loss / discard, and the field has three.** RFC 3611
   separates loss from discard; RFC 7002 counts discards; RFC 7003 decomposes discards into bursts
   and gaps. If the Buffering axis is going to be compared against anything outside this repo, a
   discard-rate definition is the one metric worth importing, and RFC 7003 supplies it without
   anyone having to trust a published number. That would be a new metric, added in the same change
   per the root `CLAUDE.md` — not a redefinition of an existing one.
2. **Reordering profiles should report RFC 4737 late-time offset, not just a reordering rate.**
   Late-time offset is measured in *time* and is therefore directly comparable to a playout buffer
   setting; reordering extent in packets is not.
3. **`Buffering/CLAUDE.md` should carry NetEQ's two-estimator split as a named planned row**, with
   the reorder cost function written out, because it is the one design in this file that is both
   fully specified and directly applicable.
4. **The "adaptive beats fixed by 47-61%" result is not comparable to anything in this literature**
   and should not be described as if it were. Nothing here reports that metric, on those profiles,
   at that packet rate. SRT (§9) is the nearest *design* comparison — a production fixed-delay
   playout with ARQ inside the budget — which makes fixed delay a legitimate production baseline
   rather than a straw man, and that is the useful thing to say about it.
5. **If a congestion controller is ever added, GCC is the family to start from and BBR is not** —
   GCC because it is the real-time-media controller and its loss policy (ignore below 2%) is an
   explicit, testable disagreement with a deadline-bound stream's instincts; BBR because its own
   draft says it handles persistently application-limited traffic badly. That would be an ADR, not
   a sweep.

6. **Added 2026-09-16, from reading the primaries rather than the abstracts.** Two arguments, both
   about measurement rather than algorithms.
   - **The field's own delay statistic is averaged over a moving sample set**, because average
     playout delay is computed over played-out packets only (§12). If this project ever reports a
     delay figure alongside a discard rate, the two must be read together or not at all — and this
     is an argument *for* the percentile-plus-baseline discipline the repo already has, not
     against it.
   - **"Does the knob do what it says" is a reportable result, and nothing in this repo reports
     it.** Zhang et al. (§4) make target-versus-achieved loss-rate error a headline figure, ahead
     of their position on the delay/loss curve, and it is the one place their scheme clearly beats
     the alternatives. Any adaptive buffering policy here that takes a target as configuration has
     the same question available to it, and answering it costs one extra column. That would be a
     new metric needing a definition in `docs/metrics.md` in the same change — not a redefinition
     of an existing one.
