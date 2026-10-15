# Qwen-RobotNav Technical Report: A Scalable Navigation Model Designed for an Agentic Navigation System

Qwen Team 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/af71bb5905c23267ca3b04cfb8fcc497527d700de8978e59d3a8380dbeed564c.jpg)


https://qwen.ai/blog?id=qwen-robotnav 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/ddb75abd6d341a2b00c014a797bc001b312f7a33fad837171d278b13a833bc45.jpg)


https://github.com/QwenLM/Qwen-RobotNav 

## Abstract

Agentic navigation systems require a base navigation model whose observation strategy can be externally reconfigured at inference time, because instruction following, object search, target tracking, and autonomous driving share the same perception-planning backbone yet demand fundamentally different strategies for consuming the visual stream. We present Qwen-RobotNav, a scalable navigation model built on Qwen3-VL that addresses it through a parameterised interface with two complementary dimensions: multiple task modes that select the navigation behaviour, and controllable observation parameters (e.g., token budget, per-camera weights) that govern how visual history is encoded. With training-time randomization over all parameters, Qwen-RobotNav is robust to any inference-time configuration requiring zero architectural modification to the Qwen3-VL backbone. We train Qwen-RobotNav on 15.6M samples; co-training with vision-language data prevents the collapse into reactive action-sequence mappers observed in trajectory-only training. The parameterised interface also makes Qwen-RobotNav a natural building block for agentic systems: for long-horizon scenarios, an upper-level planner decomposes goals into sub-tasks and dynamically switches Qwen-RobotNav’s task mode and context strategy mid-episode, composing complex behaviours from repeated calls to the same model. Extensive experiments show that Qwen-RobotNav sets new state-of-the-art results across major navigation benchmarks, achieving 76.5% success rate on VLN-CE RxR, 90.0% tracking rate on EVT-Bench, and 91.4 PDMS on NAVSIM. Beyond these standalone results, an agentic navigation system built with Qwen-RobotNav set a new state of the art on Embodied Question Answering, improving over the best prior method by 10.8% on HM-EQA and 15.4% on EXPRESS-Bench while requiring 77% fewer navigation steps. The model exhibits favourable scaling from 2B to 8B parameters, with joint multi-task training developing a shared spatial-planning substrate that transfers across task families, and demonstrates strong zero-shot generalisation to real-world robots across diverse environments. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/a6d647f39f4b1fb8e06522ff8acfb9623d1b201a7c199c2f23389e0220f593a6.jpg)



Diverse Tasks, Embodiment and Sources 67% 32% Synthetic Videos Real World Videos Generated Videos


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/04a164e11b3ee7c643f39d20870fd8809afe50da5f7d55c6010f1c2fffd34ac8.jpg)



》 Qwen-RobotNav


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/4e1c0240452b343aacc007047172c537fe6b4fd92d9234e3d7c4b8e0cc467227.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/68205d418e8a94842aaa975eade84888af6412fd4e972796a5879a45a152c9b1.jpg)



“Go to (5, 2).”


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/e9a7d2ec3c50a50a2cb34cb59ed2ce58d9244e5b360a4536372bf6d4479af3a7.jpg)



“Follow the man in black.”


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/4e2a40ea47c3476173a6a6a5c0e12d91a36d5f6925ddc539da7df9d458932296.jpg)



“Go to the living room, turn left Go to the Ivag toom,chen"


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/cd772414a08fdc647235d15bb269d7adae132350ecc7459f588694be053f93f1.jpg)



Plan over long horizon using Plan over long horizon using


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/5e9ac224ddb098b4eca69e486ce5e7c3a315d6eea2e9220b31d7ab25a349e702.jpg)



A:"Yes,Ifind it on the sofa."


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/3ca1ca76be548a8df77b1a27dbd38a8e6dd35799d19eb57072cf7ee56bd90b79.jpg)



Driving safely.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/59096ed9ef901439e744ea0c5a342b2319a9287d8eaf332dca87f37e051680bd.jpg)



Data and Model scaling behavior on instruction following.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/1864a186567c22f003e8589a4a360e640c210f557097cc96bdb9e21d17e9029f.jpg)



Figure 1: Benchmark summary. Across instruction following, object search, target tracking, embodied question answering, and autonomous driving, Qwen-RobotNav-4B and Qwen-RobotNav-8B achieve state-of-the-art or competitive performance against specialist and navigation foundation model baselines. Trophy icons mark the best result in each benchmark group.


## 1 Introduction

Embodied navigation spans a remarkably diverse family of tasks, including instruction following (Anderson et al., 2018; Krantz et al., 2020; Ku et al., 2020; Yin et al., 2025a), point-goal navigation (Savva et al., 2019; Wijmans et al., 2020), object searching (Batra et al., 2020; Yokoyama et al., 2024b;a; Yin et al., 2025b), target tracking (Zhong et al., 2024; Wang et al., 2025b; Liu et al., 2025), and autonomous driving (Caesar et al., 2020; Sun et al., 2020), each coupling perception with purposeful movement through open-world environments. Recent unified navigation models (Zhang et al., 2024a; 2025b;a; AMAP CV Lab, 2025; Cheng et al., 2025; Wang et al., 2026; Cai et al., 2025; Zhou et al., 2025a; Zhong et al., 2025; Yin et al., 2026) and navigation-oriented evaluation suites (Lin et al., 2025; Zhao et al., 2025) have demonstrated that a single architecture can handle multiple task families. 

However, complex real-world scenarios, such as Embodied Question Answering (Das et al., 2018; Majumdar et al., 2024; Ren et al., 2024; Yang et al., 2025b; Zhang et al., 2026), demand more: the navigation model must serve as a core module within larger agentic systems, where an outer planner dynamically orchestrates navigation capabilities to accomplish long-horizon, multi-step goals. This requires not only multi-task capability, but also a controllable interface whose observation strategy can be reconfigured by an external agent at inference time. Currently, no existing model meets this requirement: NavFoM (Zhang et al., 2025a) uniformly sub-samples frames; ABot-N0 (AMAP CV Lab, 2025) retains only a sliding window, each embedding a single assumption about which observations matter that cannot be adjusted at deployment time. The root cause is that different navigation tasks demand fundamentally different temporal context and memory strategies. Instruction following requires retaining observations spanning dozens of prior steps to re-reference distant landmarks (An et al., 2024; Wang et al., 2024; Wei et al., 2025b;a), whereas target tracking is governed by the most recent few frames and treats stale history as noise (Zhong et al., 2024; Wang et al., 2025b; Liu et al., 2025). Object searching further complicates the picture by shifting its own requirements within a single episode, from broad global memory during exploration to tight recency-focused attention during approach (Yokoyama et al., 2024a; Yin et al., 2024; Kuang et al., 2024; Cao et al., 2024). No single fixed context strategy can serve this full spectrum, let alone adapt mid-episode. 

We introduce Qwen-RobotNav, a scalable navigation model built on Qwen3-VL (Yang et al., 2025a) that reframes the central challenge of multi-task navigation as observation context modelling rather than architecture design. Qwen-RobotNav formulates all tasks as unified waypoint trajectory prediction and exposes a parameterised interface with two complementary configuration dimensions. First, multiple task modes (VLN, PointNav, ObjNav, Tracking) allow an upper-level planner to select the navigation behaviour appropriate for each sub-goal. Second, controllable observation parameters, including visual token budget, temporal decay, and per-camera importance weighting, govern how the model consumes the observation stream, enabling dynamic adjustment of the context strategy at inference time without task-specific retraining. Training-time randomisation over all parameters ensures that Qwen-RobotNav generalises to any inference-time configuration without task-specific tuning: the model never trains at a fixed setting, so it cannot overfit to one regime. Camera identity, temporal order, and embodiment type are communicated entirely through natural-language tags and prompt preambles, requiring zero architectural modification to the Qwen3-VL backbone; supporting a new platform requires only a new prompt template, not new parameters. 

The parameterised interface makes Qwen-RobotNav a natural building block for agentic navigation systems. For long-horizon scenarios such as Embodied Question Answering, where prior systems rely on explicit reasoning, exploration, and memory modules (Zhou et al., 2024b;a; Long et al., 2024; Ren et al., 2024; Saxena et al., 2024; Yang et al., 2025b; Zhang et al., 2026), we deploy Qwen-RobotNav within a two-tier hierarchical system. An upper-tier planner (Qwen3.6-Plus) decomposes complex goals into sub-goals and dispatches configurable navigation calls, each specifying a task mode, a sub-goal instruction, and an observation configuration; Qwen-RobotNav serves as the reactive executor, predicting waypoint trajectories at high frequency. The planner can dynamically reconfigure Qwen-RobotNav’s task mode and context strategy mid-episode, and the two tiers communicate exclusively through natural language, keeping the system modular and extensible. To support long-horizon reasoning, the system maintains a two-level memory: compact single-episode memory summarising each navigation rollout, and a persistent cross-episode memory that accumulates durable conclusions such as searched regions, candidate object locations, and rejected hypotheses, enabling effective context compression over extended episodes. 

To train Qwen-RobotNav, we curate 15.6M samples comprising navigation trajectory planning data (85%) spanning five task families (instruction following, point-goal navigation, object searching, target tracking, and autonomous driving) across diverse embodiments, and navigation-related vision-language reasoning data (15%). Co-training with vision-language data preserves the language understanding that underpins Qwen-RobotNav’s natural-language camera and temporal tags and mitigates the degradation of open-world perception that can occur when vision-language models are adapted only to action prediction (Zhou et al., 2024a; Zhang et al., 2024a; 2025b; Cheng et al., 2025). 

Without any task-specific fine-tuning, Qwen-RobotNav achieves state-of-the-art results across diverse navigation benchmarks. On VLN-CE, Qwen-RobotNav-8B attains 72.1% SR on R2R and 76.5% SR on RxR Val-Unseen, surpassing NavFoM by 10.4% and 12.1% SR respectively. On EVT-Bench, Qwen-RobotNav achieves the highest tracking rate (90.0% TR) among all evaluated methods. Strong cross-embodiment generalisation is also observed on HM3Dv2 object-goal navigation (75.6% SR) and NAVSIM autonomous driving (91.4 PDMS). The model exhibits favourable scaling properties: performance improves from 2B to 8B parameters, with particularly pronounced gains on long-horizon reasoning tasks. Equipped with the agentic system, Qwen-RobotNav sets new state-of-the-art results on three EQA benchmarks (HM-EQA, MT-EQA, and EXPRESS-Bench), and further demonstrates zero-shot transfer to real-world robots across diverse environments. 

## 2 Navigation Model

Task Formulation. We consider a general mobile navigation setting (Zhang et al., 2025a; AMAP CV Lab, 2025; Zhou et al., 2025a) in which an embodied agent receives a textual instruction L and a sequence of $\mathbf { I } _ { 1 : T } ^ { 1 : N } \in \mathbb { R } ^ { H \times W \times 3 }$ 1:T the navigation policy π must predict a waypoint trajectory 

$$
\mathcal {W} = \left\{\left(x _ {k}, y _ {k}, \theta_ {k}\right) \right\} _ {k = 1} ^ {K}, \tag {1}
$$

where $K { = } 8$ waypoints each encode a 2D position $\left( x _ { k } , y _ { k } \right)$ and heading $\theta _ { k }$ . A central challenge is that T grows online as navigation proceeds, N varies across robot platforms, and the total visual token count scales as $\mathcal { O } ( T \cdot N )$ , rapidly exceeding the context budget of any LLM backbone without a principled compression strategy. Moreover, different tasks impose fundamentally different demands on how observations should be modelled: target tracking requires only a short recency window of high-resolution frames to maintain lock on a moving object, whereas object-goal navigation must retain long-horizon episode history to recall previously explored regions and avoid redundant revisits. A single, task-agnostic observation encoding therefore cannot serve all navigation tasks well, motivating the task-adaptive strategy described in Section 2.2. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/07d567f8251182e9630a5968cff84a5e3059437dc38ec4421cb8c603c0c66d89.jpg)



Figure 2: Qwen-RobotNav architecture. Top: In the agentic navigation system, an upper planner LLM decomposes long-horizon goals into sub-goals and controls Qwen-RobotNav through task-adaptive context parameters such as token budget $\begin{array} { r } { \breve { B , } } \end{array}$ temporal decay $\gamma ,$ camera weights $w _ { c } ,$ and frame sampling mode. Bottom: Qwen-RobotNav receives multi-view RGB observations, an embodied prompt, and a navigation instruction; allocates visual tokens across cameras and timesteps; inserts natural-language temporal and viewpoint tags; and feeds the resulting visual-language sequence into the Qwen3-VL backbone. A lightweight MLP action head maps trajectory hidden states to eight waypoint predictions (x, y, θ).


## 2.1 Model Architecture

Qwen-RobotNav inherits the Qwen3-VL architecture (Yang et al., 2025a) and augments it with a lightweight action head for trajectory regression. The overall system consists of three components: 

• Vision Encoder. The vision encoder, inherited from Qwen3-VL, is built on a SigLIP-2 (Tschannen et al., 2025) Vision Transformer with native dynamic-resolution support via 2D-RoPE (Chen et al., 2025a), enabling the token allocation strategy (Section 2.2) to freely rescale each camera view to its allocated pixel budget. A two-layer MLP patch merger compresses $s _ { m } ^ { 2 }$ adjacent spatial tokens into a single vector aligned to the LLM hidden dimension, and the DeepStack mechanism (Meng et al., 2024) injects visual tokens from multiple ViT layers into early LLM layers, preserving multi-level visual representations critical for spatial reasoning. 

• Language Backbone. The LLM, also inherited from Qwen3-VL, processes the concatenation of visual tokens (organised by the strategy described in Sections 2.2 and 2.3) and language tokens from L. Through large-scale vision-language pretraining, the backbone has already acquired rich world knowledge, spatial understanding, and cross-modal reasoning capabilities that transfer directly to navigation. 

• Action Head. A lightweight 4-layer MLP maps the LLM’s final hidden state to K=8 waypoints, each with 3 degrees of freedom $( x _ { k } , y _ { k } , \theta _ { k } )$ , yielding a 24-dimensional output (Section 2.5). By keeping the action head minimal, the bulk of spatial reasoning remains within the LLM, allowing the model to benefit from the pretrained language backbone’s world knowledge when planning trajectories in unseen environments. 

## 2.2 Task-Adaptive Observation Encoding

Navigation is inherently a sequential decision-making process under partial observability: the agent cannot see the full environment at once and must accumulate spatial knowledge from a stream of egocentric observations. How much history to retain, and at what fidelity, depends fundamentally on the task’s decision structure. Tasks that require plan verification, such as instruction following, demand global episode memory so the agent can re-reference previously observed landmarks against the instruction; tasks driven by reactive pursuit, such as target tracking, depend almost entirely on the most recent frames to maintain a tight perception loop around the moving target. Fixed context strategies, such as uniform sampling (Zhang et al., 2024a; 2025b;a) or sliding windows (AMAP CV Lab, 2025), commit to a single regime and cannot adapt across tasks without retraining. Qwen-RobotNav instead provides a unified, parameterised interface with four orthogonal control axes: 


Algorithm 1 Task-Adaptive Observation Encoding


Require: Observations $I_{1:T}^{1:N}$ , instruction L, config $\Phi=(B,\gamma,\{w_{c}\},m,b_{\min},b_{\max})$ Ensure: Token sequence fed to the LLM backbone

1: $I'\leftarrow SAMPLEFRAMES(I_{1:T}^{1:N},m,T')$ $\triangleright T'\leq T$ frames

2: $\omega_{t}\leftarrow\exp(\gamma\cdot t/(T'-1)),t=0,\ldots,T'-1$ 3: $W[t,c]\leftarrow\omega_{t}\cdot w_{c}$ for all $(t,c)$ $\triangleright Eq.3$ 4: $A\leftarrow CONSTRAINEDALLOC(W,B,b_{\min},b_{\max})$ 5: for each frame $(t,c)$ do

6: $\hat{I}_{t,c}'\leftarrow RESIZE(I_{t,c}',A[t,c]\times(p\cdot s_{m})^{2})$ 7: end for

8: $V\leftarrow VISIONENCODE(\hat{I}')$ $\triangleright ViT+patch merger$ 9: $S\leftarrow INTERLEAVE(V,viewpoint/temporal tags)$ $\triangleright Sec.2.3$ 10: return Concat(S, Tokenize(L))

11: function CONSTRAINEDALLOC(W,B, $b_{min}$ , $b_{max}$ )

12: $A\leftarrow b_{min}\cdot1_{T'\times N};U\leftarrow all cells$ 13: repeat

14: Distribute B- $\sum A$ to U proportionally to W

15: Clamp cells exceeding $b_{max}$ ; remove from U

16: until no cell exceeds $b_{max}$ 17: return A

18: end function 

<table><tr><td>Parameter</td><td>Symbol</td><td>Range</td><td>Effect</td></tr><tr><td>Visual token budget</td><td><eq>B</eq></td><td>2048–4096</td><td>Total tokens across all cameras and timesteps</td></tr><tr><td>Temporal decay</td><td><eq>\gamma</eq></td><td>[1,3]</td><td>Recency bias toward the latest frames</td></tr><tr><td>Camera weights</td><td><eq>w_c</eq></td><td>per-robot defaults</td><td>Per-camera importance multiplier</td></tr><tr><td>Frame sample mode</td><td>-</td><td>random / latest</td><td>History coverage vs. recency window</td></tr></table>

Token Allocation Algorithm. Given T timesteps and N cameras, we first sub-sample or select $T ^ { \prime } { \le } T$ frames according to the frame sample mode, and then compute a temporal weight for each retained frame: 

$$
\omega_ {t} = \left\{ \begin{array}{l l} 1, & T ^ {\prime} = 1, \\ \exp \left(\gamma \cdot \frac {t}{T ^ {\prime} - 1}\right), & T ^ {\prime} > 1, \end{array} \quad t = 0, \dots , T ^ {\prime} - 1, \right. \tag {2}
$$

so that $\gamma { = } 0$ recovers uniform weighting and larger γ allocates disproportionately more tokens to recent frames (at γ=2 the most recent frame receives ≈ 7.4× the budget of the oldest), as visualised in Figure 3(a). A joint weight matrix is then formed as: 

$$
\mathbf {W} [ t, c ] = \omega_ {t} \cdot w _ {c}, \quad t \in \{0, \dots , T ^ {\prime} - 1 \}, c \in \{1, \dots , N \}, \tag {3}
$$

where wc is a per-camera importance weight that reflects the asymmetric information density across viewpoints: the forward-facing camera, which captures the richest actionable cues such as obstacles, navigable paths, and goal landmarks, is assigned the highest weight, while the rear camera receives the lowest as it primarily provides contextual redundancy (e.g. $w _ { c } { = } [ 2 . 0 , 1 . 0 , 0 . 5 , 1 . 0 ]$ ] for front, right, back, left). During training, wc is randomly sampled from its own range (e.g. the front camera from U [1.5, 2.5]); see Section 2.6 for full randomisation details. 

For a given token budget B and the frame $\left( t , c \right)$ at time step t and camera c, token allocation proceeds in three stages: (1) each cell $\left( t , c \right)$ receives a minimum floor of $b _ { \mathrm { m i n } }$ tokens; (2) the remaining budget $B - T ^ { \prime } N b _ { \mathrm { m i n } } \breve { \quad }$ is distributed proportionally to W; (3) any cell exceeding the per-image ceiling $b _ { \mathrm { m a x } }$ releases its surplus, which is redistributed iteratively until stable. We require $\begin{array} { r } { \dot { T } ^ { \prime } N b _ { \mathrm { m i n } } \le B \le { T ^ { \prime } } N b _ { \mathrm { m a x } } ; } \end{array}$ if a sampled configuration violates this constraint, B is clipped to the feasible interval before allocation. The allocated token count for each image then determines its pixel resolution through the patch size and spatial merge size of the vision encoder, and the image is rescaled to this pixel budget while preserving the original aspect ratio. Note that this token allocation is an empirically proven heuristic to expose a unified interface to control the token context of the model, which is also used in our agentic system (Section 3). We believe this strategy could be further improved by a more principled token allocation algorithm. The full procedure is summarised in Algorithm 1. Figure $3 ( \mathrm { c } , \mathrm { d } , \mathrm { e } )$ illustrates how the constrained allocation distributes tokens across timesteps and cameras under three representative budget levels. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/aa6d3df62396fe808b9c2c833ef37ff08554d3833ab32de8a2c7852c840ecc11.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/5b2ea5e055545d791a5724287d1938537258e194a929badd89233908cade497d.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/f53aa06dcdfe2d8bc7c9c99d462a7eb99ce6386633b3f5a311b2e7cc9a53814f.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/ca10b061957b954cb46d1658ab6ad92edf8611bfe5b95286facc373acf31a786.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/632e42de9ae1e7041203196286804d706cf6a96026d577eac510d5a63a13d64b.jpg)



Figure 3: Visualization of task-adaptive observation encoding. (a) Normalized temporal weights $\omega _ { t } ^ { - } = \exp ( \gamma \cdot t / ( T ^ { \prime } { - } 1 ) )$ for varying decay factors $\gamma$ when $T ^ { \prime } { > } 1 ;$ annotations show the newest-to-oldest weight ratio. (b) Resulting per-timestep token budget (summed across all cameras) under a fixed total budget B=3072 with camera weights $\stackrel { \cdot } { w } _ { c } = \left[ 2 . 0 , 1 . 0 , 0 . 5 , 1 . 0 \right]$ for front, right, back, and left views. The dashed line marks the uniform baseline B/T′. (c, d, e) Token allocation matrices across timesteps and cameras for three budget levels (B=1024, 3072, 4096) at fixed $\gamma { = } 2 . 0 ,$ , with per-image constraints $\dot { b } _ { \mathrm { m i n } } { = } 4$ and $b _ { \mathrm { m a x } } { = } 2 5 6$ . Higher γ concentrates tokens on the most recent frames, while larger B increases overall fidelity until the per-image ceiling $b _ { \mathrm { m a x } }$ saturates the highest-weight cells.


Training-Time Randomisation. At training time, all observation hyperparameters are independently randomised per sample: $\gamma \sim \mathcal { U } [ 1 , 3 ] , B \sim \breve { \mathcal { U } } [ 2 0 4 8 , 4 0 9 6 ]$ , each camera weight $w _ { c }$ sampled from its own range $( \mathrm { e . g . }$ . the front camera from $\mathcal { U } [ 1 . 5 , 2 . 5 ] )$ , and the per-image token floor and ceiling $b _ { \operatorname* { m i n } } \sim \mathcal { U } _ { \mathbb { Z } } [ 1 , 8 ]$ $b _ { \mathrm { m a x } } \sim \tilde { \mathcal { U } } _ { \mathbb { Z } } [ 1 2 8 , 2 5 6 ]$ . This ensures the model is never trained at a fixed configuration and supports robust generalisation across configurations within the randomised training range, with moderate extrapolation beyond it, without task-specific tuning. 

## 2.3 Viewpoint and Temporal Identification

After encoding, visual tokens from different cameras and timesteps are indistinguishable to the LLM: the backbone has no built-in mechanism to associate a token with “front camera, step $1 2 ^ { \prime \prime }$ versus “left camera, step $3 ^ { \prime \prime }$ . Without explicit identity signals, the model cannot learn camera-specific or temporally-ordered representations, limiting its ability to reason about spatial layout or episode history. 

Qwen-RobotNav resolves this by interleaving natural-language viewpoint and timestep tags with the visual tokens before they enter the LLM. Concretely, each timestep group is introduced by a Time step t header, followed by each camera’s name and its corresponding <image> token. For example, a two-step, six-camera input is serialised as: 

Camera identity and temporal order are thus communicated entirely through ordinary vocabulary tokens already present in Qwen3-VL, requiring zero architectural modification. The model’s existing open-world language grounding handles viewpoint semantics naturally: words like “Front”, “Left”, and “Back Right” carry spatial meaning that the pretrained LLM has already internalised from its pretraining corpus. An alternative representation is to specify each camera by its numeric azimuth (e.g. “right 90 degrees”); 

however, we find that descriptive names yield slightly better performance, likely because they carry richer semantic associations that the language model can leverage for spatial reasoning. 

## 2.4 Embodiment-Aware Prompt Design

Since Qwen-RobotNav operates across fundamentally different physical platforms, it must distinguish between embodiments such as indoor mobile robots and autonomous vehicles. Rather than encoding embodiment information through learned embeddings or separate model heads, Qwen-RobotNav communicates the agent’s physical identity through a natural-language preamble in the system prompt. For example, an indoor robot sample begins with “Imagine you are a robot programmed for navigation tasks”, whereas an autonomous driving sample begins with “Imagine you are a car programmed for autonomous driving”. 

This embodiment preamble acts as a task prior: by identifying itself as a “robot” or a $\chi _ { \mathrm { d r } ^ { \prime \prime } }$ , the model recruits different subsets of its pretrained world knowledge (e.g., indoor spatial layouts versus traffic rules and road geometry) to inform trajectory prediction. The text-based approach is also inherently extensible: supporting a new platform, such as a drone, a wheeled robot, or a quadruped robot, requires only defining a new prompt template, with no architectural changes or additional parameters. 

## 2.5 Action Planning

The action planning module maps the LLM’s final hidden state $\mathbf { E } ^ { A } \in \mathbb { R } ^ { d }$ to a waypoint trajectory. We use a lightweight 4-layer MLP head with hidden dimension 512 and GELU activations. The 24-dimensional output encodes $K { \bar { = } } 8$ waypoints each with 3 DOF $\left( x _ { k } , y _ { k } , \theta _ { k } \right)$ . Waypoints are normalised to [−1, 1] during training using per-dataset scale factors computed as the 99th percentile of each coordinate across all training trajectories, and training minimises the MSE loss between predicted and ground-truth trajectories. At inference, the predicted waypoints are de-normalised using the same scale factors. 

## 2.6 Training Strategy

Training Objective. Qwen-RobotNav is trained with a composite loss that jointly optimises trajectory regression and vision-language alignment: 

$$
\mathcal {L} = \mathcal {L} _ {\text { traj }} + \lambda   \mathcal {L} _ {\mathrm{VL}}, \tag {4}
$$

where $\mathcal { L } _ { \mathrm { t r a j } } = \| \hat { \mathcal { W } } - \mathcal { W } ^ { * } \| _ { 2 } ^ { 2 }$ is the MSE loss between predicted and ground-truth waypoints (active only on navigation trajectory samples), and ${ \mathcal { L } } _ { \mathrm { V I } }$ is the standard next-token prediction loss on navigation-related vision-language reasoning samples. The two objectives share the same forward pass; λ is set to 1.0 in all experiments. 

Configuration Randomisation. A central principle of Qwen-RobotNav’s training is that no observation configuration is fixed. At each training step, all continuous control parameters of the task-adaptive interface $( B , \dot { \gamma } , w _ { c } , b _ { \mathrm { m i n } } , \dot { b } _ { \mathrm { m a x } } ;$ ranges detailed in Section 2.2) are independently sampled per sample, and the frame sample mode alternates between random and latest with equal probability. 

By exposing the model to a broad combinatorial space of these parameters during training, Qwen-RobotNav generalises across configurations within the randomised training range without task-specific tuning. In particular, training with both random (global coverage) and latest (recency window) frame sampling in equal proportion supports zero-shot context-strategy switching at deployment time, a property exploited by the agentic system described in Section 3. 

Co-training. The training corpus mixes 85% navigation trajectory planning data with 15% navigationrelated vision-language reasoning data (Section 4). Datasets are sampled at the batch level using perdataset rates defined in a dataset registry, ensuring balanced exposure across all navigation task types (instruction following, point-goal navigation, object searching, target tracking, and autonomous driving) within every training epoch. The vision-language component prevents catastrophic forgetting of Qwen3- VL’s open-world perception capabilities: models trained on navigation trajectory data alone tend to collapse toward reactive action sequences and lose general-purpose spatial reasoning, while co-training preserves the rich language grounding that underpins Qwen-RobotNav’s zero-shot generalisation to unseen environments and instruction styles. 

Optimisation Details. Qwen-RobotNav is initialised from the pretrained Qwen3-VL checkpoint and fine-tuned end-to-end. We use the AdamW optimiser (Loshchilov &Hutter, 2019) with $\beta _ { 1 } { = } 0 . \dot { 9 } , \beta _ { 2 } { = } 0 . 9 5 .$ and weight decay $1 0 ^ { - 2 }$ . A cosine learning rate schedule is applied with a linear warm-up over the first 3% of training steps; the peak learning rate is $2 \times 1 0 ^ { - 5 }$ for the vision encoder and LLM backbone and 

$1 \times 1 0 ^ { - 4 }$ for the action head. Gradient norms are clipped to 1.0. The 8B model is trained with a global batch size of 256 for a total of 2,816 H100 GPU hours. 

## 3 Qwen-RobotNav for Agentic Navigation

## 3.1 Overview

The navigation model described in Section 2 is designed not only for standalone benchmark evaluation, but also for deployment as a navigation module inside general-purpose embodied agents (Zhou et al., 2024b; Long et al., 2024). In this setting, the upper-level agent should be able to decide not only what navigation sub-goal to execute, but also which navigation task mode, which observation configuration, and when to query auxiliary visual evidence should be used for the current phase of the task. For example, a longhorizon search episode may alternate between instruction following, object-goal search, point-to-point movement, and target tracking, while also changing how much visual history should be retained. 

We therefore expose Qwen-RobotNav as an agent-ready navigation model through a lightweight tool interface. At each navigation step, the upper-level planner provides a sub-goal instruction $\mathcal { L } _ { i } ,$ selects a task mode $\tau _ { i } ,$ and specifies an observation configuration $\Phi _ { i }$ . The task mode $\tau _ { i }$ describes the navigation behavior to be invoked, such as VLN, PointNav, ObjNav, or Tracking. The configuration $\Phi _ { i } = \^ { \smile } ( B , \gamma , \{ w _ { c } \} , m , b _ { \mathrm { m i n } } , b _ { \mathrm { m a x } } )$ follows the notation in Section 2.2: B controls the total visual-token budget, γ controls the recency bias, $\{ w _ { c } \}$ specifies camera weights, m specifies the frame sample mode, and $\breve { b } _ { \mathrm { m i n } } , b _ { \mathrm { m a x } }$ define the per-image allocation constraints. Given the selected task mode and configuration, Qwen-RobotNav predicts a waypoint trajectory $\mathcal { W } _ { i }$ in the same form as Eq. (1). 

Besides navigation calls, the planner may issue auxiliary vision-tool calls over the current observations or stored key frames. In our interface, these tools include object detection, scene understanding, and semantic grounding. They provide additional evidence for deciding the next sub-goal or verifying candidate observations, but they do not predict waypoints and do not replace Qwen-RobotNav as the core navigation executor. 

The resulting system uses Qwen-RobotNav as the core navigation executor. The upper-level planner is responsible for decomposing long-horizon goals, selecting task modes and context configurations, querying auxiliary visual evidence when needed, and reasoning over the evidence returned by previous navigation calls. A lightweight harness connects the two sides: it converts planner decisions into Qwen-RobotNav calls, and converts completed navigation rollouts into compact trajectory evidence for subsequent planning. This keeps the agentic layer simple while allowing it to access the controllable interfaces already built into Qwen-RobotNav. 

## 3.2 Agent-Facing Qwen-RobotNav Interface

The key agent-facing property of Qwen-RobotNav is that its navigation behavior is externally configurable at inference time. This configurability comes from two complementary navigation interfaces. 

First, Qwen-RobotNav supports multiple navigation task modes. In VLN mode, the model follows naturallanguage route instructions and grounds them in visual observations. In PointNav mode, it moves toward a specified spatial target or waypoint-like goal. In ObjNav mode, it searches for an object category or instance using accumulated visual evidence. In Tracking mode, it prioritizes recent observations to maintain lock on a moving or recently observed target. These modes are not separate navigation policies; they are different task interfaces to the same Qwen-RobotNav model. 

Second, each navigation call can specify an observation configuration Φ. The model section has already described the full task-adaptive observation encoding algorithm in Section 2.2, so here we only emphasize how it is used by the planner. The planner can increase $\breve { B }$ and use a weaker recency bias when the current sub-goal needs broader episode history, such as object search or revisiting a previously explored region. It can instead use a smaller B, larger $\gamma ,$ and latest frame sampling when the sub-goal is local and reactive, such as approaching a visible object or tracking a moving target. In practice, $w _ { c } , b _ { \mathrm { m i n } } ,$ and $b _ { \mathrm { m a x } }$ are usually kept at platform-level defaults, while $B , \gamma ,$ , and m are the main controls exposed to the planner. 

A navigation call can therefore be written abstractly as: 

$$
\mathcal {W} _ {i} = \text { nav\_qwennav } (\mathcal {L} _ {i}, \tau_ {i}, \Phi_ {i}), \tag {5}
$$

where $\mathcal { L } _ { i }$ is the current sub-goal, $\tau _ { i }$ is the selected task mode, and $\Phi _ { i }$ is the observation configuration. This interface gives the planner access to Qwen-RobotNav’s internal context controls without requiring architectural changes, task-specific fine-tuning, or a separate routing model. The same model weights can be used across different task phases; only the tool-call arguments change. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/0751b1da56dad41cdaba5846c85f3b41dae7a2d13d2327d2b53364c18964643a.jpg)



Figure 4: Qwen-RobotNav for agentic navigation. An upper-level planner decomposes a long-horizon task into sub-goals and dispatches either auxiliary vision-tool calls or Qwen-RobotNav navigation calls. Each navigation call is parameterized by a sub-goal instruction $\mathcal { L } _ { i } ,$ a task mode $\tau _ { i } ,$ and an observation configuration $\Phi _ { i }$ . Qwen-RobotNav uses the selected task mode and configuration to predict waypoints $\mathcal { W } _ { i }$ , which are executed in the environment. The harness converts the resulting rollout into sourceindexed key frames, trajectory summaries, and evidence-notebook updates, providing compact context for the next planning step.


For example, when searching for an object in a large indoor scene, the planner may call Qwen-RobotNav in ObjNav mode with a larger token budget and history-covering frame sampling. After a candidate object is observed, the planner can switch to Tracking or local PointNav behavior with a more recency-focused configuration. This illustrates the main role of the agent-facing interface: it allows the upper-level planner to compose long-horizon behavior from repeated, configurable calls to the same navigation foundation model. 

## 3.3 Navigation Harness: Trajectory Evidence and Context Compression

The interface above controls how Qwen-RobotNav is called. The remaining question is how the result of each navigation call is returned to the planner. A raw Qwen-RobotNav rollout contains dense egocentric observations, sampled history frames, low-level control traces, and a predicted waypoint trajectory. Passing this full stream back into the planner dialogue would quickly exhaust the context budget. On the other hand, returning only a success flag would discard the evidence needed for future planning. We therefore use a lightweight trajectory-to-evidence interface. 

For each navigation call, the harness converts the executed rollout into a compact evidence record: 

{
    subgoal: "Search the kitchen area for a mug",
    task_mode: ObjNav,
    config: $\Phi_{i}$ (main controls: $B, \gamma, m$ ),
    progress: "entered kitchen, checked countertop and dining table",
    salient: ["sink", "countertop", "round table", "no mug observed"],
    outcome: "target not found",
    key_frames: [#18, #31]
} 

This record is not intended to replace visual evidence. Instead, it acts as a source-indexed summary of what happened during the navigation segment. The textual fields tell the planner what was attempted, where the agent moved, what was observed, and how the sub-goal ended. The key-frame identifiers point back to stored visual observations that can be retrieved later if the planner needs to re-inspect a scene or verify a candidate target. 

The harness maintains a compact evidence notebook for long-horizon reasoning. The notebook stores durable conclusions such as searched regions, candidate object locations, rejected hypotheses, landmark cues, and layout assumptions. Unlike raw dialogue history, notebook entries are designed to survive context compression. Later entries may revise earlier beliefs, but the update history remains auditable. A typical entry might be: 

[step 47] Kitchen entered and searched; countertop and dining table checked. No mug observed. 

Corridor shelf remains a possible candidate region from key frame #12. 

Together, trajectory evidence and the evidence notebook form a two-level memory. By default, the planner reasons over compact textual records and notebook entries. When text is insufficient, it can retrieve source images through visual recall using the stored key-frame identifiers. This keeps the planner context concise while preserving access to detailed visual evidence. 

The harness can also expose auxiliary vision-tool calls, such as object detection, scene understanding, and semantic grounding. These tools support the planner’s situational awareness, but they do not replace Qwen-RobotNav as the core navigation model. Their role is to provide additional evidence channels around the same planner–Qwen-RobotNav loop: the planner selects a task mode and configuration, Qwen-RobotNav executes the navigation segment, and the harness returns compact trajectory evidence for the next decision. 

In summary, the agentic layer does not introduce a separate navigation policy. Its main function is to make Qwen-RobotNav’s existing task modes and observation controls accessible to an upper-level planner, and to make completed rollouts usable for long-horizon reasoning through trajectory evidence, notebook memory, and context compression. 

## 4 Data

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/8c6bb5bf0047041a7a4271345367b26a6cb98fbcd14fb3ae96df4a9b92d5b374.jpg)



Figure 5: Training data distribution. Left: Per-dataset sample counts across all navigation trajectory and vision-language sources. Right: Aggregated distribution over task categories, totalling 15.6M training samples.


## 4.1 Navigation Trajectory Planning

A key design principle of Qwen-RobotNav is to train on a deliberately broad spectrum of navigation tasks rather than specialising in any single paradigm. We structure the trajectory planning corpus around four capability dimensions that broaden the operational demands placed on the model: grounding language or geometric guidance into executable motion, exploring partially observed spaces to locate targets (Zheng et al., 2025; 2024; Peng et al., 2025), interacting with dynamic agents whose future states must be anticipated in real time, and planning under high-speed, multi-agent, safety-critical dynamics in openworld environments. These dimensions roughly span increasing environmental uncertainty, temporal dependence, and embodiment diversity, but they are not mutually exclusive in practice. Rather, more complex navigation regimes often reuse simpler competencies while introducing additional perception, planning, and control requirements. Jointly training across this spectrum encourages the model to develop a shared spatial-planning substrate that generalises beyond any single task family. The full training corpus comprises 15.6M samples; Figure 5 summarises the per-dataset and per-category distributions. 

Guided by these dimensions, we select five concrete task families as the instantiation of our training corpus: 

• Instruction following and point-goal navigation jointly ground guided navigation by pairing rich procedural language with compact directional or coordinate-based commands, thereby spanning the full granularity spectrum of task specification. 

• Object-goal navigation realises the exploration dimension, requiring the agent to build implicit spatial maps and hypothesise target locations without step-by-step instructions. 

• Active target tracking introduces dynamic interaction, demanding real-time motion anticipation and re-identification of moving persons in crowded scenes. 

• Autonomous driving realises cross-embodiment planning under high-speed, multi-agent traffic dynamics with stringent safety constraints, an operational regime that shares the same underlying spatial-planning principles as indoor navigation yet demands substantially different perception range, action granularity, and decision latency. 

These five tasks are chosen because they are mutually non-redundant: each exercises a distinct combination of perception, planning, and control that the others leave undertrained, while together they provide sufficient coverage for these capability dimensions to reinforce one another during joint optimisation. 

## 4.1.1 Instruction Following

Instruction following represents the language-rich form of guided navigation. The agent must interpret natural language instructions alongside egocentric visual observations and plan waypoint trajectories to reach the described destination while adhering to the procedural milestones specified in the instruction. This supervision trains fine-grained language-to-control grounding: spatial relations, landmark references, and temporal ordering in the instruction must be translated into executable motion. We construct 5.63M instruction-following training samples from two VLN-CE benchmarks (Krantz et al., 2020; Xia et al., 2026), building on Matterport3D (Chang et al., 2017) scenes. 

VLN-CE R2R (1,491K). The Room-to-Room benchmark (Krantz et al., 2020) provides approximately 10K continuous navigation clips in indoor environments, each paired with a concise goal-oriented instruction. We unroll each ground-truth trajectory with teacher forcing and record RGB observations in both singlecamera (front only) and multi-camera (front, left, right, and rear) configurations at every discrete step, yielding 1,491K training samples after data balancing across view configurations, instruction refinement, and image quality enhancement. 

VLN-CE RxR (4,140K). The Room-Across-Room benchmark (Ku et al., 2020) significantly expands R2R in both scale and complexity, featuring longer paths, richer spatial relationships, and multilingual instructions with dense landmark references. Following the same teacher-forcing protocol, we extract 4,140K training samples from approximately 20K continuous clips across view configurations and augmentation variants. 

All instruction-following samples from both sources undergo the image quality enhancement and instruction refinement pipelines described in Section 4.2.1: simulator renders are transformed into photorealistic images via Qwen-Image-Edit (Wu et al., 2025) style transfer, and each instruction is paraphrased into multiple linguistically diverse variants using an LLM (Yang et al., 2025a); the sample counts above include all resulting augmentation variants. 

## 4.1.2 Point Goal Navigation

Point-goal navigation provides the language-sparse counterpart to instruction following. The agent must reach a specified target given only its current visual observation and a compact task specification, without detailed natural language guidance. Whereas instruction-following data emphasise procedural language grounding, point-goal data isolate geometric path planning, local obstacle avoidance, and smooth goal approach from minimal coordinate or command inputs. We generate 984K point-goal navigation training samples from Matterport3D (Chang et al., 2017) and HM3D (Ramakrishnan et al., 2021) scenes using the Habitat simulator (Savva et al., 2019), and organise them into a curriculum from primitive motion grounding to long-horizon path search. 

Coordinate-based point goals (922K). The agent receives the target position as a numeric coordinate in its egocentric frame together with its current pose, distance, and bearing angle to the goal. We allocate this subset according to difficulty rather than sampling uniformly. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/19927c1041f70d7e77eeed82482067d688ad7000d1655e9666abbf2973e5b99d.jpg)



Figure 6: Visualization of the three coordinate-based point-goal navigation categories. Direct Approach (348K) targets near-straight paths along canonical egocentric directions. Short Range (174K) introduces local obstacle avoidance within cluttered indoor geometry. Long Range (400K) requires multi-room path search over extended horizons, navigating around walls and furniture.


Direct-approach point goals (348K). Targets are placed along canonical egocentric directions or on an integer grid within $[ - 5 , 5 ] \times [ - 5 , 5 ]$ m, and paths whose geodesic-to-euclidean ratio exceeds 1.15 are rejected. This group corresponds to simplified point-of-interest navigation in which the target can be reached by a near-straight trajectory, focusing supervision on basic coordinate grounding, heading control, and smooth approach behaviour. 

• Short-range point goals (174K; geodesic distance 0.5–6 m). These episodes introduce cluttered indoor geometry and local obstacle avoidance while keeping the destination within a limited spatial neighbourhood. Because they are relatively local and the visual evidence is often sufficient to infer a feasible detour, a moderate sample budget is sufficient. 

• Long-range point goals (400K; geodesic distance 6–10 m). These episodes receive the largest allocation because they frequently lack direct line of sight to the target and require the agent to search over alternative routes, navigate around walls and furniture, and plan across rooms over extended horizons. 

This distribution implements a curriculum in which the model first learns reliable coordinate-to-motion grounding, then local collision avoidance, and finally the more difficult path-searching behaviour needed for non-myopic navigation. 

Command-based motion primitives (62K). Instead of explicit coordinates, the agent receives a textual motion primitive. Parameterised commands specify both action and magnitude (e.g., “Move forward 2.0 meters”, “Turn left 90 degrees”), while bare commands provide only the direction without a numeric target (e.g., “Move forward”, “Turn left”), requiring the model to infer an appropriate displacement from visual context. This subset is intentionally small because such commands primarily anchor basic action semantics, such as moving forward by a specified distance or rotating by a specified angle; extensive repetition yields limited additional path-planning diversity compared with coordinate-based long-range episodes. 

Point-goal navigation (PointNav) also serves as a controlled baseline for studying navigation competence because the goal specification is simple and the required behaviour can be evaluated without the confounding effect of complex language. This simplicity makes PointNav particularly suitable for controlled observation-configuration and motion-dynamics augmentation; the corresponding front-viewonly training protocol and simulator perturbations are described in Section 4.2.1. 

Each training sample contains a uniformly sampled set of history frames, the task specification (coordinates or command), and an 8-waypoint future trajectory. Within 1.5 m of the goal, deceleration trajectories with linearly decreasing step sizes are generated to teach smooth stopping behaviour. Forward steps are subsampled at a 45% inclusion rate to rebalance the action distribution, while turns and stop actions are always retained. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/a68d42a6231819139cf158a6ed4f4fbc89190bc6487306cd54f65b0fddce5011.jpg)



Figure 7: Object-goal navigation data generation pipeline. (1) The top-down occupancy map is binarised into a navigability mask. (2) Morphological dilation and erosion expand navigable regions and remove thin noise. (3) Skeletonisation extracts the medial-axis graph of the navigable space. (4) An exploration trajectory is generated by traversing the skeleton with backtracking at dead ends. (5) A VLM annotates the goal object at the terminal viewpoint, producing open-vocabulary goal specifications embedded in diverse instruction templates.


## 4.1.3 Object-Goal Navigation

Object-goal navigation addresses goal-directed exploration under partial observability. The agent receives a semantic object category or open-vocabulary object description, rather than a complete route, and must search the environment until an instance of the target is reached. This setting exercises capabilities that are difficult to induce from route-following data alone: maintaining an implicit memory of explored regions, using scene priors to hypothesise likely object locations, and revising the route when the target is absent from the current field of view. 

We construct 2,000K object-goal navigation training samples from Matterport3D (Chang et al., 2017) and HM3D (Ramakrishnan et al., 2021) reconstructed scenes, including open-vocabulary object annotations from HM3D-OVON (Yokoyama et al., 2024b). 

Skeleton-based exploration trajectories. A key challenge is that standard shortest-path followers produce straight-to-goal trajectories that do not reflect realistic search behaviour: in practice, an agent exploring for an unseen object must traverse corridors, inspect rooms, and backtrack from dead ends before locating the target. We therefore generate trajectories using a skeleton-based exploration strategy over the scene’s navigable space. Given the simulator’s top-down occupancy map, we first extract the largest connected component of the navigable area and then apply morphological skeletonisation (Zhang &Suen, 1984) to reduce it to a one-pixel-wide medial-axis graph. Short spurious branches are pruned to suppress redundant bifurcation points. On this skeleton graph, a goal location is sampled at random, and the agent starts from a random navigable position. The path is planned by traversing the skeleton: at each junction the agent randomly selects a branch; upon reaching a dead end it backtracks and explores an alternative branch, continuing until the branch containing the goal is reached. Because the skeleton lies along the medial axis of navigable regions, the resulting paths maintain a safe distance from walls and obstacles. The raw skeleton path is then smoothed via cubic-spline interpolation: waypoints sampled at regular intervals along the skeleton are fitted with a parametric cubic spline, and the final trajectory points are sampled from this spline at a fixed step size of 0.25 m, producing smooth, physically plausible motion. 

VLM-based goal annotation. Each trajectory requires an associated object goal. Rather than relying on a fixed category taxonomy, we adopt a VLM-in-the-loop annotation strategy that produces openvocabulary goals by construction. At the terminal point of a generated trajectory, the agent is oriented towards a random direction and the resulting egocentric image is submitted to a vision-language model, which is asked to identify a salient, reachable object in the scene. If the VLM confirms the presence of a suitable target, the trajectory is retained and the object name returned by the VLM is used as the goal specification; otherwise the trajectory is discarded. Because the VLM is free to name any visible object without a predefined label set, the resulting goal vocabulary is inherently open and covers a long tail of everyday objects beyond the fixed categories of standard ObjectNav benchmarks. The goal is then embedded into diverse natural-language instruction templates (e.g., “navigate to the {goal_object}”, “find and reach the {goal_object}”) to provide linguistic variation during training. Each training sample contains uniformly sampled history frames, the task specification, and an 8-waypoint future trajectory, following the same format used by the other navigation task families. The same observation-configuration and image-quality augmentations described in Section 4.2.1 are applied, including camera height and field-of-view randomisation. 

## 4.1.4 Target Tracking

Active visual tracking extends the trajectory corpus from static goal reaching to dynamic interaction. The agent must identify a person described by a natural language query (e.g., “Follow the man in the blue t-shirt”), maintain pursuit in crowded indoor environments, and keep an appropriate following distance while avoiding obstacles. Compared with object-goal navigation, this setting introduces time-critical requirements including motion anticipation, occlusion handling, target re-identification, and balancing pursuit with safety constraints. We collect 1,486K training samples from the EVT-Bench dataset (Wang et al., 2025b), which covers diverse indoor scenes populated with hundreds of digital avatars whose appearance and motion are controlled independently. Following the benchmark protocol (Wang et al., 2025b), we focus on the Single Target Tracking (STT) split, where a unique target is specified per episode and no distractors are introduced. 

Each training sample contains the current egocentric observation, a short history of past frames, the textual target description, and the ground-truth future trajectory. We additionally apply the same imagequality and observation-configuration augmentations used in the instruction-following data to improve cross-scene and cross-sensor generalisation. 

## 4.1.5 Autonomous Driving

Autonomous driving is incorporated as a cross-embodiment source of trajectory supervision rather than as a separate top-level capability class (Hu et al., 2023; Li et al., 2025c; Peng et al., 2026; Liang et al., 2026). It requires an agent to reason over complex, dynamic, and safety-critical traffic environments and to predict feasible future trajectories under diverse road geometries, traffic rules, and agent interactions. Unlike indoor embodied navigation, driving scenarios involve higher-speed motion, structured lane topology, long-range perception, and dense multi-agent dynamics. These differences stress the same spatialplanning substrate under a substantially different operating regime, exposing the model to long-horizon planning, interaction-aware motion prediction, and safety-constrained control. To equip the model with transferable planning capabilities across both embodied and vehicle-centric settings, we incorporate large-scale driving data into training and cast autonomous driving as a unified waypoint prediction problem conditioned on multimodal observations. We construct the autonomous driving training set from two complementary sources: nuScenes (78K) (Caesar et al., 2020) and OpenScene (3,138K) (Contributors, 2023). Together, these datasets provide diverse urban scenes, rich sensor observations, map-aware traffic contexts, and a broad spectrum of driving behaviours ranging from routine lane following to complex maneuvers such as turning, merging, yielding, and obstacle avoidance. By integrating them into a common trajectory-planning format, we obtain approximately 3.2M autonomous-driving trajectorysupervision instances. Here, the reported number refers to the total number of annotated supervision variants rather than the number of distinct raw driving trajectories. Specifically, a single underlying trajectory may be instantiated into multiple conditioning variants, depending on whether navigation instructions, ego-state information, and historical ground-truth trajectory priors are provided. This design allows the same driving behavior to be observed under different levels of prior information, thereby encouraging the model to learn trajectory planning that is robust to heterogeneous inputs and transferable across different settings. 

Each supervision instance is centered on predicting the ground-truth future trajectory at the current frame, conditioned on multimodal observations and a configurable set of auxiliary priors. All instances include multi-view camera observations, while different annotation variants may additionally provide navigation instructions, the current ego-vehicle state, and/or a short history of past ground-truth trajectories. Thus, the autonomous-driving data is not treated as a single fixed-input prediction task, but as a family of trajectory-planning tasks with different available context. This unified yet flexible representation enables the model to reason over visual scene context, vehicle motion, temporal dynamics, and high-level route intent when available, while also learning to remain effective when some priors are absent. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/4bd8622ebbea19b5864542d82d3defbdad7f506d69ae2aa0c93d8ac0e0539d95.jpg)



Move to the side of the buffet cabinet and stop, positioning yourself to see the decorative items on top and the cabinet door details. Remain stationary for a few seconds.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/b234aa5dbf92682d51087a1493d0069b0864c892665ab330f2d8d96810ac29c7.jpg)



Follow the woman in the light green top as she moves from the window to the bookshelf, then into the adjacent study room; maintain tracking when she exits and proceeds down the hallway.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/83b51cce4a2ee091630aa1a6603ec9d27d2f8d36ba52ae68928025983be698f1.jpg)



Figure 8: Autogenerated navigation data pipeline. Right: A large language model first generates paired video prompts and navigation instructions; a text-to-video model then synthesises first-person egocentric videos, which are filtered by a vision-language model for quality before a monocular depth-and-pose estimator extracts 2-D trajectories; a final kinematic filter removes physically implausible samples. Left: Two example outputs covering instruction following (top) and target tracking (bottom), each showing a generated video frame alongside the recovered bird’s-eye-view trajectory.


## 4.2 Autogenerated Navigation Data with a Video Generator

While simulator-derived trajectories provide precise ground-truth supervision, they are constrained by the availability of reconstructed 3-D assets and exhibit a persistent visual domain gap relative to real-world observations. To complement the simulator corpus with diverse, photorealistic training data that does not require 3-D scene reconstruction, we introduce a fully automated pipeline that converts textto-video (T2V) generations into navigation trajectory samples. The pipeline covers two core capabilities, instruction following and target tracking, and produces 40K training samples through five sequential stages illustrated in Figure 8. 

• Stage 1: Prompt and instruction generation. A large language model generates paired outputs for each sample: a detailed video prompt describing a first-person navigation scenario, and a corresponding natural-language navigation instruction. Prompts are drawn from a scene-complexity matrix spanning 12 scene categories (e.g., home, office, shopping mall, hospital, park, street) and 7 interaction complexities (e.g., obstacle avoidance, target occlusion and reappearance, crowd navigation, sudden direction changes), yielding 29 distinct combinations that ensure broad environmental and behavioural coverage. 

• Stage 2: Text-to-video synthesis. Each video prompt is fed to a T2V model, which renders a firstperson egocentric navigation video of approximately five seconds. The resulting clips depict the agent moving through the described scene with realistic lighting, textures, and dynamic elements such as pedestrians, without requiring any 3-D assets or physics simulation. 

• Stage 3: VLM-based video quality filtering. A vision-language model evaluates every generated video against capability-specific quality dimensions. For instruction-following videos the assessment covers scene consistency, navigation correctness, goal arrival, stopping behaviour, motion continuity, and collision avoidance; target-tracking videos are additionally evaluated on target visibility, pursuit behaviour, and identity consistency across frames. Only videos that pass all applicable criteria are retained. 

• Stage 4: Trajectory extraction. A monocular depth-and-pose estimation model recovers per-frame camera-to-world poses from each retained video. The estimated poses are converted from camera coordinates into the robot-centric ground-plane frame, producing a 2-D trajectory [ x, y, yaw ] per frame, where x and y denote forward and lateral displacement in metres and yaw is the heading angle in radians, all relative to the first frame. 

• Stage 5: Trajectory quality filtering. A rule-based kinematic filter removes trajectories that exhibit physically implausible characteristics, including near-zero total displacement, excessive positional or 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/f84811b2b567377125dcf6c5ba7cc3920ef72dc9fec7e121895f0503d9ce1ca4.jpg)



Figure 9: Visual comparison between original Habitat simulator renders (top) and Qwen-Image-Edit (Wu et al., 2025) (bottom).


heading jitter, single-step teleportation, abnormal acceleration, and high-frequency noise detected via spectral analysis. Each check uses a tunable threshold, and a trajectory is discarded if it triggers any single criterion. 

The resulting dataset provides trajectory supervision grounded in photorealistic, T2V-generated imagery across a wide variety of real-world-like scenes, complementing the simulator-derived corpus with greater visual diversity and reduced domain gap. 

## 4.2.1 Navigation Data Augmentation

To enhance robustness and data diversity, we apply task-dependent augmentation strategies to simulatorderived navigation data. Instruction refinement is restricted to language-conditioned trajectories, whereas visual refinement, observation-configuration perturbation, and speed variation are applied to simulatorderived trajectory samples when supported by the source data. 

• Instruction refinement. We employ an LLM-based paraphrasing pipeline (Yang et al., 2025a) that first deduplicates instructions by trajectory identifier and then generates multiple paraphrased variants per unique instruction. The generation prompt instructs the model to preserve all spatial directions and relative landmarks, vary landmark nouns with synonyms or hypernyms, diversify sentence structure and action verbs, and correct any grammatical noise, so that each variant reads as if written by a different annotator. Three variants are generated per unique instruction, effectively tripling the linguistic diversity of the corpus. 

• Image quality enhancement. The sim-to-real visual gap remains a major bottleneck for transferring simulation-trained policies to real environments. We build a scalable refinement pipeline based on Qwen-Image-Edit (Wu et al., 2025), which applies prompt-guided diffusion-based style transfer to every rendered observation. Given a source image and a natural-language editing prompt (e.g., “Convert the rendered image into a photorealistic photograph while preserving spatial layout”), the model generates a visually refined version that retains the original geometric layout while exhibiting more realistic textures, lighting, and material appearance. 

• Observation and camera augmentation. Camera height is uniformly sampled in [0.5, 1.5] m, horizontal field of view (HFoV) in [90◦, 120◦], and image aspect ratio between 2:1 and 4:3, so the model learns to navigate under diverse sensor configurations without task-specific retraining. For PointNav, we additionally perturb the robot’s initial heading and egocentric viewpoint, and explicitly construct front-view-only variants in which the model receives only the front camera observation rather than panoramic inputs. This setting matches deployment scenarios where only a forward-facing camera is available. 

• Speed augmentation. We generate trajectory data at multiple speed regimes to expose the model to diverse motion dynamics. The standard-speed variant uses the default Habitat action discretisation 

(0.25 m forward steps, 15◦ turns). The low-speed variant replays the same planned trajectories at finer temporal granularity with randomised sub-step sizes (0.05 to 0.25 m), while PointNav trajectories are further replayed under varied motion scales to improve robustness across robot actuation speeds. 

## 4.3 Vision Language Data

Robust action prediction rests on perceptual and reasoning capabilities that cut across all navigation task families and cannot be acquired reliably from trajectory supervision alone. Recognising objects in cluttered scenes, reading text on signs, reasoning about spatial layouts, comparing multiple views, and interpreting novel visual contexts are prerequisites for navigation in unseen environments. We co-train navigation trajectories with approximately 1.0M general vision-language samples and 873K navigationspecific reasoning samples, organised into two complementary groups that collectively maintain and strengthen the perceptual substrate required for the VLM-to-VLA transfer. 

General vision-language data (∼1.0M). We incorporate training samples spanning eight complementary categories: visual question answering (∼669K; VQAv2 (Goyal et al., 2017), DVQA (Kafle et al., 2018), FigureQA (Kahou et al., 2018), CLEVR (Johnson et al., 2017), etc.), image captioning (∼6K), visual grounding (∼178K; RefCOCO (Kazemzadeh et al., 2014), COCO (Lin et al., 2014), Objects365 (Shao et al., 2019)), instruction following (∼30K), multi-image reasoning and comparison (∼38K), general visual question answering (∼76K), object and landmark recognition (∼16K), and STEM problem solving (∼14K). Rather than serving merely as a regulariser against catastrophic forgetting, these samples preserve and extend the in-the-wild visual understanding that underpins downstream action prediction—object and landmark recognition, spatial reasoning, text parsing, and multi-image analysis. All samples follow a unified conversation format and are co-trained with navigation data, so that this perceptual substrate remains intact as the model acquires embodied competence. 

Navigation-specific reasoning (873K). Navigation reasoning is inherently domain-specific: deciding whether to turn left at a hallway intersection demands spatial understanding that differs qualitatively from general visual question answering. We extend NavGPT-2 (Zhou et al., 2024a) and construct two complementary reasoning formats from existing VLN trajectory data. Free-form QA targets key decision points within each trajectory, the start step, the final step, and transition points where the dominant action category switches between forward motion and turning. Each sample’s 8-step future trajectory is classified into one of twelve fine-grained action classes (six pure turns: slight, medium, and large in both left and right directions: four forward-with-turn combinations, straight-ahead movement, and stop) and cast as a question–answer pair, training the model to articulate spatial reasoning in natural language before committing to actions. Structured multi-perspective reasoning extends this idea to multiview contextual analysis. For each selected sample, the agent’s historical front-view observations (up to eight uniformly sampled frames) and current four-view panoramic images (front, right, back, left) are combined with the navigation instruction, ground-truth action label, and trajectory statistics into a structured prompt. A vision-language model then generates a structured reasoning chain comprising four components: 

• History reasoning. A first-person narrative summarising the journey so far based on historical views. 

• Scene analysis. A description of what is visible in each of the four current views. 

• Instruction progress. An assessment of completed and remaining sub-goals relative to the original instruction. 

• Action reasoning. A concise reasoning chain concluding with the predicted action and a confidence score. 

Each sample is subsequently decomposed into four independent question–answer pairs, yielding separate fine-tuning signals for history comprehension, scene understanding, progress tracking, and action prediction. By requiring the model to reconstruct its trajectory context, analyse the current scene, and assess instruction progress before deriving an action, both formats enforce a systematic reasoning process that serves as an inductive bias for the VLM-to-VLA transfer: the language-mediated reasoning, once internalised, acts as a transferable scaffold that improves both QA accuracy and the quality of predicted action trajectories. 

Discrete multi-round navigation data (362K). To complement the continuous-environment trajectory data, we incorporate approximately 362K training samples derived from discrete VLN datasets by reformulating graph-based navigation trajectories into a multi-round, multi-image conversation format. Following the VLN-MME evaluation framework (Zhao et al., 2025), each navigation step is formulated as a single-step action prediction task cast in a multiple-choice question format: the agent receives four perspective-view images (front, right, back, left) extracted from the panoramic observation rendered by the Matterport3D Simulator (Chang et al., 2017), together with a set of candidate next-viewpoints annotated with visual markers as answer options, and selects the correct action among them. Rather than rendering observations in the Habitat simulator, we use the Matterport3D Simulator, which produces higher-definition photorealistic panoramic views directly from the original 3-D scans. Unlike the continuous-environment action data, which are decomposed into independent single-step samples, this discrete formulation allows each trajectory to be maintained as a complete multi-turn dialogue, preserving the full decision history within the conversation context and enabling the model to leverage prior turns for spatial reasoning without requiring an explicit history encoding module. Through this pipeline, we incorporate additional VLN datasets beyond R2R and RxR: CVDN (Thomason et al., 2020), which introduces dialog-conditioned navigation requiring multi-turn linguistic grounding; SOON (Zhu et al., 2021), which targets situated object-oriented navigation with fine-grained spatial goal specifications; REVERIE (Qi et al., 2020), which pairs remote referring expressions with object localisation; and SRDF (Wang et al., 2025c), which bootstraps large-scale augmented trajectories via a self-refining data flywheel across diverse indoor environments. This approach enables us to significantly scale the number of training trajectories and their linguistic diversity while preserving the multi-turn reasoning capabilities that single-step formulations sacrifice. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/81bc8e691235ef3f84ae5900d5ee6cdf628700db97e6cce9b922b0a37fa692cd.jpg)



Figure 10: Visualization of structured multi-perspective reasoning along a complete navigation trajectory. Given the instruction “Enter the open space, then go through the door on the right. Wait just past the black doormat,” the agent executes eight sequential steps (numbered on the floor plan). At each step, a fourcomponent reasoning chain is produced: History summarises the journey so far, Scene Analysis describes the current multi-view observations, Instruction Progress tracks completed and remaining sub-goals, and Action Reasoning derives the next action. This structured decomposition is used to generate navigationspecific reasoning supervision for VLM-to-VLA transfer.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/686362c4f4b684ca1d93db653c89f307d06976398731c50fad0241dbd5e6f4fb.jpg)



Figure 11: Deployment architecture and latency comparison of Qwen-RobotNav-4B on Unitree Go2. Left: remote-server and edge deployment pipelines, where edge inference runs on Jetson Thor with FP8 quantization and TensorRT acceleration. Right: latency evaluation, including (a) per-frame latency fluctuations and (b) average latency breakdown for both deployments.


## 5 Experiments

## 5.1 Deployment

For real-world deployment, we explore both cloud-based and on-device inference for Qwen-RobotNav. In the cloud setting, the robot uploads compressed observations and text instructions over the network to a remote server for trajectory prediction, and receives the planned actions for execution. This setup supports larger models with strong inference capability and faster server-side inference, but introduces communication overhead and relies on network stability. In the on-device setting, inference runs directly on the robot via an NVIDIA Jetson Thor, eliminating transmission delay and improving robustness, at the cost of being constrained by onboard compute and bandwidth resources. 

Taking network transmission into account, the remote-server deployment still achieves lower average end-to-end latency than on-device inference, with 196 ms (5.1 Hz) versus 204 ms (4.9 Hz). The trade-off is higher sensitivity to network conditions, leading to larger latency variance and occasional spikes. By eliminating network transmission, on-device inference provides more consistent latency and is therefore more robust for latency-sensitive tasks such as tracking. Overall, remote-server inference is faster on average, whereas on-device inference is more stable and reliable in real-world deployment. 

## 5.2 Evaluation

We evaluate Qwen-RobotNav on three standard embodied navigation benchmarks: vision-and-language navigation in continuous environments (VLN-CE), open-vocabulary object-goal navigation (OVON), and active visual tracking (EVT-Bench). Results are compared against two recent navigation foundation model baselines (NavFoM (Zhang et al., 2025a) and ABot-N0 (AMAP CV Lab, 2025)) as well as established specialist methods. 

## 5.2.1 Vision-and-Language Navigation

VLN-CE. Table 1 reports VLN-CE R2R and RxR Val-Unseen results under both monocular and panoramic observation settings. Under the panoramic setting, Qwen-RobotNav-8B achieves 72.1% SR and 66.6% 


Table 1: VLN-CE Val-Unseen results under monocular and panoramic observation settings. Bold: best within each setting; underline: second best.


<table><tr><td rowspan="2">Method</td><td colspan="4">R2R Val-Unseen</td><td colspan="4">RxR Val-Unseen</td></tr><tr><td>NE↓</td><td>OS↑</td><td>SR↑</td><td>SPL↑</td><td>NE↓</td><td>nDTW↑</td><td>SR↑</td><td>SPL↑</td></tr><tr><td colspan="9">Monocular</td></tr><tr><td>NaVid (Zhang et al., 2024a)</td><td>5.72</td><td>49.2</td><td>41.9</td><td>36.5</td><td>5.72</td><td>-</td><td>45.7</td><td>38.2</td></tr><tr><td>Uni-NaVid (Zhang et al., 2025b)</td><td>5.58</td><td>53.3</td><td>47.0</td><td>42.7</td><td>6.24</td><td>-</td><td>48.7</td><td>40.9</td></tr><tr><td>NaVILA (Cheng et al., 2025)</td><td>5.22</td><td>62.5</td><td>54.0</td><td>49.0</td><td>6.77</td><td>58.8</td><td>49.3</td><td>44.0</td></tr><tr><td>StreamVLN (Wei et al., 2025b)</td><td>4.98</td><td>64.2</td><td>56.9</td><td>51.9</td><td>6.22</td><td>61.9</td><td>52.9</td><td>46.0</td></tr><tr><td>DualVLN (Wei et al., 2025a)</td><td>4.05</td><td>70.7</td><td>64.3</td><td>58.5</td><td>4.58</td><td>70.0</td><td>61.4</td><td>51.8</td></tr><tr><td>InternVLA-N1 (Cai et al., 2025)</td><td>4.83</td><td>63.3</td><td>58.2</td><td>54.0</td><td>5.91</td><td>65.3</td><td>53.5</td><td>46.1</td></tr><tr><td>Qwen-RobotNav-4B</td><td>4.22</td><td>73.6</td><td>66.9</td><td>60.5</td><td>4.15</td><td>68.6</td><td>71.3</td><td>61.5</td></tr><tr><td>Qwen-RobotNav-8B</td><td>4.36</td><td>72.7</td><td>65.7</td><td>59.6</td><td>4.16</td><td>69.9</td><td>73.4</td><td>63.5</td></tr><tr><td colspan="9">Panoramic</td></tr><tr><td>NavFoM (Zhang et al., 2025a)</td><td>4.61</td><td>72.1</td><td>61.7</td><td>55.3</td><td>4.74</td><td>65.8</td><td>64.4</td><td>56.2</td></tr><tr><td>ABot-N0 (AMAP CV Lab, 2025)</td><td>3.78</td><td>70.8</td><td>66.4</td><td>63.9</td><td>3.83</td><td>-</td><td>69.3</td><td>60.0</td></tr><tr><td>OmniNav (Hu et al., 2025)</td><td>3.74</td><td>74.6</td><td>69.5</td><td>66.1</td><td>3.77</td><td>-</td><td>73.6</td><td>62.0</td></tr><tr><td>AstraNav-World (Hu et al., 2026)</td><td>3.86</td><td>73.9</td><td>67.9</td><td>65.4</td><td>3.82</td><td>-</td><td>72.9</td><td>61.5</td></tr><tr><td>Qwen-RobotNav-4B</td><td>3.80</td><td>77.2</td><td>69.5</td><td>63.6</td><td>3.80</td><td>71.9</td><td>75.2</td><td>65.0</td></tr><tr><td>Qwen-RobotNav-8B</td><td>3.53</td><td>78.5</td><td>72.1</td><td>66.6</td><td>3.58</td><td>72.5</td><td>76.5</td><td>65.7</td></tr></table>


Table 2: Navigation foundation model comparison on VLNVerse (Lin et al., 2025) test split. TL: Trajectory Length; NE: Navigation Error; SR: Success Rate; OSR: Oracle Success Rate; SPL: Success weighted by Path Length. Bold: best; underline: second best.


<table><tr><td rowspan="2">Method</td><td colspan="5">Fine-grained</td><td colspan="5">Coarse-grained</td></tr><tr><td>TL</td><td>NE↓</td><td>OSR↑</td><td>SR↑</td><td>SPL↑</td><td>TL</td><td>NE↓</td><td>OSR↑</td><td>SR↑</td><td>SPL↑</td></tr><tr><td>InternVLA-N1 (Cai et al., 2025)</td><td>9.23</td><td>5.68</td><td>38.69</td><td>28.95</td><td>25.00</td><td>4.00</td><td>5.71</td><td>23.32</td><td>17.51</td><td>16.54</td></tr><tr><td>Uni-NaVid (Zhang et al., 2025b)</td><td>12.35</td><td>4.11</td><td>67.02</td><td>45.74</td><td>26.91</td><td>10.33</td><td>5.05</td><td>42.70</td><td>29.68</td><td>13.47</td></tr><tr><td>NavFoM (Zhang et al., 2025a)</td><td>8.58</td><td>4.13</td><td>69.68</td><td>51.59</td><td>32.40</td><td>6.52</td><td>4.93</td><td>45.93</td><td>38.02</td><td>23.15</td></tr><tr><td>Qwen-RobotNav-4B</td><td>7.99</td><td>3.05</td><td>70.36</td><td>62.61</td><td>56.22</td><td>7.39</td><td>4.40</td><td>51.03</td><td>41.25</td><td>37.37</td></tr><tr><td>Qwen-RobotNav-8B</td><td>8.16</td><td>3.00</td><td>72.81</td><td>63.75</td><td>57.93</td><td>7.68</td><td>4.08</td><td>55.97</td><td>46.59</td><td>41.54</td></tr></table>

SPL on R2R, surpassing NavFoM by 10.4% SR and ABot-N0 by 5.7% SR. On the longer-horizon RxR benchmark, Qwen-RobotNav-8B reaches 76.5% SR and 72.5 nDTW, outperforming NavFoM by 12.1% SR and ABot-N0 by 7.2% SR. Under the monocular setting, Qwen-RobotNav remains competitive despite the significantly reduced field of view. Qwen-RobotNav-4B achieves 66.9% SR and 60.5% SPL on R2R, surpassing the strongest monocular baseline DualVLN by 2.6% SR and 2.0% SPL. On RxR, Qwen-RobotNav-8B reaches 73.4% SR and 63.5% SPL, outperforming DualVLN by 12.0% SR and 11.7% SPL, demonstrating particularly strong gains on long-horizon instructions. The consistent improvements across both observation settings validate the effectiveness of task-adaptive token allocation and the generality of Qwen-RobotNav: it delivers state-of-the-art performance regardless of the input modality, whether operating with a single forward-facing camera or a full panoramic observation. 

VLNVerse. Table 2 reports results on VLNVerse (Lin et al., 2025), a large-scale benchmark that unifies previously fragmented VLN tasks into a single evaluation framework and replaces teleporting “ghost” agents with full-kinematics embodied locomotion driven by a physics engine. We evaluate under both fine-grained and coarse-grained instruction settings, thereby probing whether an agent can follow detailed step-by-step guidance as well as interpret high-level goal descriptions. Qwen-RobotNav-8B achieves 63.75% SR and 57.93% SPL on the fine-grained split, surpassing NavFoM by 12.2% SR and 25.5% SPL, and Uni-NaVid by 18.0% SR and 31.0% SPL. On the more challenging coarse-grained split, where instructions omit procedural detail and require the agent to infer intermediate waypoints, Qwen-RobotNav-8B reaches 46.59% SR and 41.54% SPL, outperforming NavFoM by 8.6% SR and 18.4% SPL. Notably, the SPL margins are substantially larger than the SR margins in both settings, indicating that Qwen-RobotNav not only reaches the goal more often but follows markedly more efficient paths. The 4B variant also surpasses all baselines, confirming that the performance advantage is not merely a consequence of model scale. 

VLN-PE. Table 3 further evaluates on VLN-PE (Wang et al., 2025a) using the flash controller, which decouples high-level navigation planning from low-level locomotion control and thereby provides a more direct assessment of spatial reasoning quality. Qwen-RobotNav-8B achieves 65.50% SR and 61.19% 


Table 3: VLN-PE results with flash controller (R2R Val-Unseen) (Wang et al., 2025a). TL: Trajectory Length; NE: Navigation Error; FR: Fall Rate; OS: Oracle Success Rate; SR: Success Rate; SPL: Success weighted by Path Length. Bold: best; underline: second best.


<table><tr><td>Method</td><td>TL</td><td>NE↓</td><td>FR↓</td><td>OS↑</td><td>SR↑</td><td>SPL↑</td></tr><tr><td>Seq2Seq (Krantz et al., 2020)</td><td>19.24</td><td>8.27</td><td>0.22</td><td>43.05</td><td>15.74</td><td>9.70</td></tr><tr><td>CMA (Krantz et al., 2020)</td><td>40.21</td><td>31.24</td><td>0.22</td><td>45.06</td><td>20.94</td><td>14.06</td></tr><tr><td>RDP (Wang et al., 2025a)</td><td>15.12</td><td>6.98</td><td>0.30</td><td>42.54</td><td>24.94</td><td>17.54</td></tr><tr><td>InternVLA-N1 (Cai et al., 2025)</td><td>10.11</td><td>4.13</td><td>0.45</td><td>67.63</td><td>60.36</td><td>54.93</td></tr><tr><td>Qwen-RobotNav-4B</td><td>10.39</td><td>4.24</td><td>3.83</td><td>72.70</td><td>60.28</td><td>55.24</td></tr><tr><td>Qwen-RobotNav-8B</td><td>9.17</td><td>3.73</td><td>4.05</td><td>72.99</td><td>65.50</td><td>61.19</td></tr></table>


Table 4: Closed-vocabulary object-goal navigation on MP3D and HM3D. †: uses depth or odometry. ‡: results on HM3D v1 as reported in original papers; Qwen-RobotNav is evaluated on the harder HM3D v2 benchmark. Bold: best; underline: second best.


<table><tr><td rowspan="2">Method</td><td colspan="2">MP3D</td><td colspan="2">HM3D</td></tr><tr><td>SR↑</td><td>SPL↑</td><td>SR↑</td><td>SPL↑</td></tr><tr><td><eq>VLFM^{\dagger\ddagger}</eq> (Yokoyama et al., 2024a)</td><td>36.4</td><td>17.5</td><td>52.5</td><td>30.4</td></tr><tr><td><eq>OpenFMNav^{\dagger\ddagger}</eq> (Kuang et al., 2024)</td><td>37.2</td><td>15.7</td><td>52.5</td><td>24.1</td></tr><tr><td><eq>SG-Nav^{\dagger\ddagger}</eq> (Yin et al., 2024)</td><td>40.2</td><td>16.0</td><td>54.0</td><td>24.9</td></tr><tr><td><eq>TriHelper^{\dagger\ddagger}</eq> (Zhang et al., 2024b)</td><td>-</td><td>-</td><td>56.5</td><td>25.3</td></tr><tr><td><eq>WMNav^{\dagger\ddagger}</eq> (Nie et al., 2025)</td><td>45.4</td><td>17.2</td><td>58.1</td><td>31.2</td></tr><tr><td><eq>CogNav^{\dagger\ddagger}</eq> (Cao et al., 2024)</td><td>46.6</td><td>16.1</td><td>72.5</td><td>26.2</td></tr><tr><td><eq>Uni-NaVid^{\ddagger}</eq> (Zhang et al., 2025b)</td><td>-</td><td>-</td><td>73.7</td><td>37.1</td></tr><tr><td>Qwen-RobotNav-4B</td><td>52.2</td><td>16.0</td><td>75.6</td><td>30.6</td></tr><tr><td>Qwen-RobotNav-8B</td><td>48.8</td><td>17.7</td><td>71.2</td><td>33.0</td></tr></table>

SPL, surpassing InternVLA-N1 by 5.1% SR and 6.3% SPL, while also attaining the lowest navigation error (3.73 m) and the highest oracle success rate (72.99%) among all methods. Qwen-RobotNav-4B performs comparably to InternVLA-N1 in SR (60.28% vs. 60.36%) but achieves higher oracle success (72.70% vs. 67.63%), suggesting stronger goal-proximate planning even at smaller model capacity. 

## 5.2.2 Object-Goal Navigation

MP3D & HM3D. Table 4 reports closed-vocabulary ObjectNav results on MP3D and HM3D. Notably, most prior methods in this space rely on depth sensors or odometry (marked with †), whereas Qwen-RobotNav operates from RGB observations alone. Since prior works only report results on HM3D v1 (Semantics v0.1, used in the Habitat 2022 challenge), we list their published numbers as baselines. Qwen-RobotNav is evaluated on HM3D v2 (Semantics v0.2, used in the Habitat 2023 challenge), which features updated semantic annotations and a larger scene pool (216 vs. 120 scenes). Despite this disadvantage, Qwen-RobotNav-4B achieves 52.2% SR on MP3D, surpassing all depth-based baselines including CogNav (46.6%) and WMNav (45.4%). On HM3D v2, Qwen-RobotNav-4B reaches 75.6% SR with a distance-togoal of only 1.72 m, surpassing even the v1 results of the vision-only Uni-NaVid (73.7%) and establishing a new state of the art on this benchmark. The 8B variant achieves the best SPL on MP3D (17.7%) and competitive SPL on HM3D v2 (33.0%), indicating more efficient paths at larger model scale. 

HM3D-OVON. Table 5 reports open-vocabulary results on HM3D-OVON, where the agent must locate objects described by free-form category names rather than a fixed label set. Qwen-RobotNav-4B achieves 57.7% / 60.1% / 53.1% SR on the Seen / Synonyms / Unseen splits, attaining the best success rate on two of three splits and ranking a close second on Unseen (53.1% vs. ABot-N0’s 54.0%). Importantly, Qwen-RobotNav uses only a single forward-facing camera in this evaluation, whereas ABot-N0 consumes panoramic multi-view observations that provide 360◦ scene coverage and substantially reduce the need for active exploration—yet Qwen-RobotNav still surpasses it on both Seen and Synonyms by 2.4% and 4.7% SR respectively. The 8B variant also outperforms ABot-N0 on these two splits while achieving higher SPL (28.5% / 28.8%), reflecting more efficient goal approach at larger model capacity. The lower SPL of Qwen-RobotNav relative to NavFoM and ABot-N0 reflects a reach-first exploration behaviour: the skeleton-based training trajectories encourage thorough room-by-room search, which improves goal-finding rate but at the cost of longer paths. 


Table 5: Open-vocabulary object-goal navigation on HM3D-OVON (Yokoyama et al., 2024b). †: uses depth or odometry. Bold: best; underline: second best.


<table><tr><td rowspan="2">Method</td><td colspan="2">Seen</td><td colspan="2">Synonyms</td><td colspan="2">Unseen</td></tr><tr><td>SR↑</td><td>SPL↑</td><td>SR↑</td><td>SPL↑</td><td>SR↑</td><td>SPL↑</td></tr><tr><td><eq>VLFM^† (Yokoyama et al., 2024a)</eq></td><td>35.2</td><td>18.6</td><td>32.4</td><td>17.3</td><td>35.2</td><td>19.6</td></tr><tr><td><eq>DAgRL+OD^† (Yokoyama et al., 2024b)</eq></td><td>38.5</td><td>21.1</td><td>39.0</td><td>21.4</td><td>37.1</td><td>19.8</td></tr><tr><td><eq>MTU3D^† (Zhu et al., 2025)</eq></td><td>55.0</td><td>23.6</td><td>45.0</td><td>14.7</td><td>40.8</td><td>12.1</td></tr><tr><td>Uni-NaVid (Zhang et al., 2025b)</td><td>41.3</td><td>21.1</td><td>43.9</td><td>21.8</td><td>39.5</td><td>19.8</td></tr><tr><td>NavFoM (Zhang et al., 2025a)</td><td>40.1</td><td>27.1</td><td>45.4</td><td>32.6</td><td>45.2</td><td>31.9</td></tr><tr><td>ABot-N0 (AMAP CV Lab, 2025)</td><td>55.3</td><td>32.1</td><td>55.4</td><td>33.2</td><td>54.0</td><td>30.5</td></tr><tr><td>Qwen-RobotNav-4B</td><td>57.7</td><td>24.4</td><td>60.1</td><td>25.1</td><td>53.1</td><td>20.9</td></tr><tr><td>Qwen-RobotNav-8B</td><td>56.1</td><td>28.5</td><td>57.8</td><td>28.8</td><td>51.2</td><td>24.0</td></tr></table>


Table 6: Active visual tracking on EVT-Bench (Wang et al., 2025b) (Single Target split, single-view). SR: Success Rate; TR: Tracking Rate; CR: Collision Rate. †: uses GroundingDINO; ‡: uses SoM+GPT-4o. Bold: best; underline: second best.


<table><tr><td>Method</td><td>TR↑</td><td>CR↓</td><td>SR↑</td></tr><tr><td>IBVS<eq>^{\dagger}</eq> (Gupta et al., 2016)</td><td>56.2</td><td>3.75</td><td>42.9</td></tr><tr><td>PoliFormer<eq>^{\dagger}</eq> (Zeng et al., 2024)</td><td>15.5</td><td>40.1</td><td>4.67</td></tr><tr><td>EVT (Zhong et al., 2024)</td><td>39.1</td><td>42.5</td><td>24.4</td></tr><tr><td>EVT<eq>^{\ddagger}</eq> (Zhong et al., 2024)</td><td>49.9</td><td>40.5</td><td>32.5</td></tr><tr><td>Uni-NaVid (Zhang et al., 2025b)</td><td>39.5</td><td>41.9</td><td>25.7</td></tr><tr><td>TrackVLA (Wang et al., 2025b)</td><td>78.6</td><td>1.65</td><td>85.1</td></tr><tr><td>TrackVLA++ (Liu et al., 2025)</td><td>81.0</td><td>2.10</td><td>86.0</td></tr><tr><td>NavFoM (Zhang et al., 2025a)</td><td>80.5</td><td>-</td><td>85.0</td></tr><tr><td>ABot-N0 (AMAP CV Lab, 2025)</td><td>87.6</td><td>8.54</td><td>86.9</td></tr><tr><td>Qwen-RobotNav-4B</td><td>90.0</td><td>6.40</td><td>77.4</td></tr><tr><td>Qwen-RobotNav-8B</td><td>89.7</td><td>5.70</td><td>78.6</td></tr></table>

## 5.2.3 Active Visual Tracking

Table 6 reports single-target tracking performance on EVT-Bench. Qwen-RobotNav achieves the highest tracking rates among all methods (90.0% TR for 4B and 89.7% TR for 8B), surpassing ABot-N0 (87.6%) by 2.4%, NavFoM (80.5%) by 9.5%, and the dedicated tracker TrackVLA++ (81.0%) by 9.0%. Qwen-RobotNav-8B also achieves the lowest collision rate among generalist models (5.70% CR), substantially below ABot-N0 (8.54%). However, the success rate of Qwen-RobotNav (77.4% / 78.6%) trails that of ABot-N0 (86.9%) and TrackVLA++ (86.0%), which are specifically optimised for tracking tasks. We hypothesise that the broader multi-task training of Qwen-RobotNav introduces a trade-off where the model maintains tighter following behaviour (superior TR) while being more conservative in declaring episode success. 

## 5.3 Embodied Question Answering

Table 7 summarises embodied question answering results on HM-EQA, MT-HM3D, and EXPRESS-Bench. Our Qwen-RobotNav-based variants outperform prior methods across all three benchmarks. In particular, Qwen3.6-Plus+Qwen-RobotNav achieves the best overall performance, reaching 76.7 SR on HM-EQA, 54.4 SR on MT-HM3D, and 79.27 LLM Score on EXPRESS-Bench, consistently surpassing recent strong baselines such as FAST-EQA and Memory-EQA. Compared with FAST-EQA, Qwen3.6- Plus+Qwen-RobotNav improves absolute performance by 7.5 points on HM-EQA, 3.9 points on MT-HM3D, and 10.57 points on EXPRESS-Bench. It also reduces normalized equivalent steps on HM-EQA and MT-EQA and improves EXPRESS-Bench $E _ { p a t h }$ by 4.71 points, indicating that the learned executor supports more targeted physical exploration. These results suggest that the multimodal reasoning and navigation capabilities of Qwen-RobotNav transfer effectively to embodied question answering, enabling both stronger scene exploration and more accurate answer prediction. Furthermore, the substantial improvement in navigation efficiency achieved by Qwen-RobotNav enables our system to reach target locations using significantly fewer equivalent steps. This further demonstrates its viability as an effective physical-world tool for general-purpose multimodal agents. 


Table 7: Performance comparison on embodied question answering benchmarks. Results are compared against prior state-of-the-art methods on HM-EQA, MT-HM3D, and EXPRESS-Bench. ∗: result reported by prior work. †: results are on the full A-EQA split. Bold: best; underline: second best.


<table><tr><td rowspan="2">Method</td><td colspan="2">HM-EQA</td><td colspan="2">MT-EQA</td><td colspan="2">EXPRESS-Bench</td></tr><tr><td>Acc. (↑)</td><td>Steps (↓)</td><td>Acc. (↑)</td><td>Steps (↓)</td><td>LLM Score (↑)</td><td><eq>E_{path}</eq>(↑)</td></tr><tr><td>Explore-EQA (Ren et al., 2024)</td><td>58.4</td><td>0.52</td><td>36.2*</td><td>0.64</td><td>-</td><td>-</td></tr><tr><td>Graph-EQA (Saxena et al., 2024)</td><td>63.5</td><td>0.20</td><td>45.6*</td><td>0.45</td><td>-</td><td>-</td></tr><tr><td>Memory-EQA (Li et al., 2026a)</td><td>61.4</td><td>0.40</td><td>43.1</td><td>0.41</td><td>-</td><td>-</td></tr><tr><td>Fine-EQA (Jiang et al., 2025)</td><td>56.0</td><td>0.54</td><td>-</td><td>-</td><td>63.95</td><td>25.58</td></tr><tr><td>3D-Mem (Yang et al., 2025b)</td><td>50.4</td><td>0.63</td><td>-</td><td>-</td><td>-</td><td>-</td></tr><tr><td>FAST-EQA (Zhang et al., 2026)</td><td>69.2</td><td>0.65</td><td>50.5</td><td>0.52</td><td>68.7</td><td>29.25</td></tr><tr><td>Qwen3.5-Plus+QwenNav-8B</td><td>74.1</td><td>0.17</td><td>52.1</td><td>0.22</td><td>77.66</td><td>31.73</td></tr><tr><td>Qwen3.6-Plus+QwenNav-8B</td><td>76.7</td><td>0.15</td><td>54.4</td><td>0.19</td><td>79.27</td><td>33.96</td></tr></table>


Table 8: Performance comparison on NAVSIM navtest using closed-loop metrics. NC: Navigation Compliance; DAC: Drivable Area Compliance; TTC: Time-to-Collision; Comf.: Comfort; EP: Ego Progress; PDMS: PDM Score. †: uses additional LiDAR information. ‡: our model evaluated without historical ego-status prior. Bold: best; underline: second best.


<table><tr><td>Method</td><td>NC↑</td><td>DAC↑</td><td>TTC↑</td><td>Comf.↑</td><td>EP↑</td><td>PDMS↑</td></tr><tr><td>Human</td><td>100</td><td>100</td><td>100</td><td>99.9</td><td>87.5</td><td>94.8</td></tr><tr><td>Constant Velocity</td><td>68.0</td><td>57.8</td><td>50.0</td><td>100</td><td>19.4</td><td>20.6</td></tr><tr><td>Ego Status MLP</td><td>93.0</td><td>77.3</td><td>83.6</td><td>100</td><td>62.8</td><td>65.6</td></tr><tr><td>UniAD (Hu et al., 2023)</td><td>97.8</td><td>91.9</td><td>92.9</td><td>100</td><td>78.8</td><td>83.4</td></tr><tr><td>TransFuser<eq>^{\dagger}</eq> (Chitta et al., 2022)</td><td>97.7</td><td>92.8</td><td>92.8</td><td>100</td><td>79.2</td><td>84.0</td></tr><tr><td>PARA-Drive (Weng et al., 2024)</td><td>97.9</td><td>92.4</td><td>93.0</td><td>99.8</td><td>79.3</td><td>84.0</td></tr><tr><td>LAW (Li et al., 2024)</td><td>96.4</td><td>95.4</td><td>88.7</td><td>99.9</td><td>81.7</td><td>84.6</td></tr><tr><td>DRAMA<eq>^{\dagger}</eq> (Yuan et al., 2024)</td><td>98.0</td><td>93.1</td><td>94.8</td><td>100</td><td>80.1</td><td>85.5</td></tr><tr><td>Hydra-MDP++ (Li et al., 2025a)</td><td>97.6</td><td>96.0</td><td>93.1</td><td>100</td><td>80.4</td><td>86.6</td></tr><tr><td>DiffusionDrive<eq>^{\dagger}</eq></td><td>98.2</td><td>96.2</td><td>94.7</td><td>100</td><td>82.2</td><td>88.1</td></tr><tr><td>WoTE<eq>^{\dagger}</eq> (Li et al., 2025b)</td><td>98.5</td><td>96.8</td><td>94.9</td><td>99.9</td><td>81.9</td><td>88.3</td></tr><tr><td>Hydra-NeXt<eq>^{\dagger}</eq> (Li et al., 2025d)</td><td>98.1</td><td>97.7</td><td>94.6</td><td>100</td><td>81.8</td><td>88.6</td></tr><tr><td>VÄDv2 (Jiang et al., 2024)</td><td>98.3</td><td>97.4</td><td>95.7</td><td>100</td><td>82.3</td><td>89.3</td></tr><tr><td>GoalFlow<eq>^{\dagger}</eq> (Xing et al., 2025)</td><td>98.4</td><td>98.3</td><td>94.6</td><td>100</td><td>85.0</td><td>90.3</td></tr><tr><td>DrivingGPT (Chen et al., 2025b)</td><td>98.9</td><td>90.7</td><td>94.9</td><td>95.6</td><td>79.7</td><td>82.4</td></tr><tr><td>NavFoM (Zhang et al., 2025a)</td><td>97.7</td><td>93.5</td><td>92.3</td><td>100</td><td>79.6</td><td>84.3</td></tr><tr><td>AutoVLA (Zhou et al., 2025b)</td><td>98.4</td><td>95.6</td><td>98.0</td><td>99.9</td><td>81.9</td><td>89.1</td></tr><tr><td>ReCogDrive (Li et al., 2025c)</td><td>97.9</td><td>97.3</td><td>94.9</td><td>100</td><td>87.3</td><td>90.8</td></tr><tr><td>ReflectDrive (Li et al., 2026b)</td><td>97.7</td><td>99.3</td><td>93.5</td><td>100</td><td>86.9</td><td>91.1</td></tr><tr><td>Qwen-RobotNav-4B<eq>^{\ddagger}</eq></td><td>96.4</td><td>90.9</td><td>89.0</td><td>99.9</td><td>75.2</td><td>79.5</td></tr><tr><td>Qwen-RobotNav-8B<eq>^{\ddagger}</eq></td><td>95.9</td><td>91.3</td><td>88.4</td><td>100</td><td>75.5</td><td>79.5</td></tr><tr><td>Qwen-RobotNav-4B</td><td>99.8</td><td>97.5</td><td>98.5</td><td>99.9</td><td>84.4</td><td>91.4</td></tr><tr><td>Qwen-RobotNav-8B</td><td>99.8</td><td>96.9</td><td>98.2</td><td>99.9</td><td>84.2</td><td>90.9</td></tr></table>

## 5.4 Autonomous Driving

NAVSIM. Table 8 reports trajectory planning results with closed-loop metrics on NAVSIM navtest. During evaluation, we provide the ground-truth trajectories of the previous three frames in the prompt as historical priors, allowing the model to condition its prediction on short-term ego-motion context. Qwen-RobotNav achieves strong performance among vision-language driving models. In particular, Qwen-RobotNav-4B reaches 91.4 PDMS, outperforming NavFoM by 7.1 points, AutoVLA by 2.3 points, ReCogDrive by 0.6 points, and ReflectDrive by 0.3 points. It also achieves highly competitive safetyrelated scores, with 99.8 NC and 98.5 TTC, while maintaining strong DAC and EP performance. Qwen-RobotNav-8B obtains similarly strong results, reaching 90.9 PDMS with 99.8 NC and 98.2 TTC. Compared with the variants evaluated without historical ego-status priors, both models obtain a substantial gain of more than 11 points in PDMS, highlighting the importance of short-term trajectory history for closed-loop driving prediction. 

As shown in Figure 12, the qualitative NAVSIM visualization further illustrates the closed-loop behavior of Qwen-RobotNav in a left-turn scenario. The model leverages multi-view observations together with ego-state and historical trajectory information to predict a smooth turning trajectory. Across different timesteps, the predicted path adapts to the changing scene context, follows the curvature of the annular road, and gradually transitions from entering the turn to aligning with the outgoing lane. This demonstrates that Qwen-RobotNav can maintain temporally coherent planning behavior under multi-view driving observations. 


Turn Left ...... (ego state infomation and history trajectories)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/bb06d66f87299efd084a6ed18ad59324307643b3be162cdf4ce2d53ac12d95f0.jpg)



Figure 12: Qualitative closed-loop planning visualization on NAVSIM. We visualize a representative left-turn case in an annular road scene. For each timestep, the figure shows the multi-view camera observations, the predicted future trajectory overlaid on the front-view image, and the corresponding BEV scene. Qwen-RobotNav produces temporally consistent curved trajectories from Step 1 to Step 30, progressively completing the left turn while remaining aligned with the drivable lane.



Table 9: AlpaSim closed-loop evaluation on the PhysicalAI-AV NuRec dataset. Results are evaluated on 920 scenarios using at-fault closed-loop metrics, where close encounters are counted only when the ego vehicle is deemed responsible, excluding rear-end close encounters. Bold: best; underline: second best.


<table><tr><td>Method</td><td>Close Encounter Rate↓ (%)</td><td>Off-Road Rate↓ (%)</td><td>AlpaSim Score↑</td></tr><tr><td>Alpamayo-R1-0.5B</td><td>9.0</td><td>19.0</td><td>0.35</td></tr><tr><td>Alpamayo-R1-10B</td><td>4.0</td><td>16.0</td><td>0.72</td></tr><tr><td>Qwen-RobotNav-4B</td><td>22.0</td><td>34.0</td><td>0.15</td></tr><tr><td>Qwen-RobotNav-8B</td><td>22.0</td><td>27.0</td><td>0.17</td></tr></table>

AlpaSim. Table 9 presents closed-loop evaluation on the PhysicalAI-AV NuRec dataset using AlpaSim. This benchmark evaluates driving performance with at-fault metrics, where close encounters are counted 

Turn Right ...... (ego state infomation and history trajectories) 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/65d5cba106d68df73dd1380e92ba2efc70781aa3b48c353f497ec0b2576a418b.jpg)


Go Straight ...... (ego state infomation and history trajectories) 

Figure 13: Qualitative zero-shot closed-loop simulation visualization on AlpaSim. We show two representative cases from the PhysicalAI-AV NuRec dataset. In the first right-turn case, Qwen-RobotNav slows down and proceeds straight after approaching an intersection, then performs a right turn while keeping away from the road boundary, and finally continues forward along the outgoing lane. In the second straight-driving case, the ego vehicle starts from a stopped state after the traffic light turns green and safely passes through a complex intersection while avoiding surrounding traffic participants. 

only when the ego vehicle is deemed responsible. We include AlpaSim as an additional autonomous driving benchmark to examine closed-loop safety and long-horizon driving robustness beyond NAVSIM. 

Importantly, Qwen-RobotNav is evaluated in a zero-shot setting on AlpaSim: the model is directly rolled out in the simulator without AlpaSim-specific training or closed-loop adaptation on the PhysicalAI-AV NuRec scenarios. Therefore, this benchmark mainly measures the out-of-domain transfer ability of a general vision-language navigation model under long-horizon autonomous driving simulation. Although Qwen-RobotNav still lags behind the Alpamayo-R1 models that are designed for this evaluation setting, it achieves non-trivial zero-shot closed-loop performance. Moreover, increasing the model scale from 4B to 8B improves the off-road rate from 34.0% to 27.0% and the AlpaSim score from 0.15 to 0.17, suggesting that larger backbones provide better closed-loop stability and scene-level generalization under this challenging transfer setting. 

Figure 13 provides qualitative examples of Qwen-RobotNav in zero-shot AlpaSim closed-loop simulation. In the right-turn scenario, the ego vehicle first approaches the intersection cautiously and proceeds straight at a reduced speed, then executes a right turn while avoiding the road boundary, and finally aligns with the outgoing lane to continue driving forward. In the intersection scenario, the vehicle correctly reacts to the green traffic light, starts from a stopped state, and proceeds through a complex multi-agent intersection without colliding with other vehicles. These cases suggest that, despite the remaining quantitative gap on aggregate AlpaSim metrics, Qwen-RobotNav can still exhibit meaningful reactive planning behaviors when transferred zero-shot to challenging closed-loop driving scenes. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/4a7c586b6a5743622aa6d1ce139ba14d78e90f02ac13a1fc796fe03bdbdcc4ee.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/d510d07f2eac0e7c852480ecd77bda28cb7cf6a920156609cb9bb52a1d10300c.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/825b3bea1ac2147a9c1e9d96a2a53f54d4a0d10e3170e64569c30bf002260bf6.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/a6a294828d61e2de460fe1644075d98ade93c33ee0de15ee381c9dbf7685a637.jpg)



Figure 14: Data scaling behavior of Qwen-RobotNav. Performance on representative navigation benchmarks as a function of the training data fraction. Increasing the amount of navigation training data yields clear gains on most tasks, with especially strong improvements on long-horizon tasks such as VLN-CE RxR, while short-horizon tracking saturates earlier and exhibits mild non-monotonicity.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/716c76a74688daeb2e8a40540ddc493128d337efbba97f9214a6bfb28e9e2656.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/bb4e524b9e701fe73d4d233cac3589cce3c08852df81d022396d96e62fa263f6.jpg)



OSR (%) SR (%) NDTW (%) SPL (%)


Figure 15: Ablation on token budget B and temporal decay γ. We evaluate Qwen-RobotNav-4B on 500 VLN-CE R2R Val-Unseen episodes under varying configurations. Left: Sweeping the token budget from 2048 to 4608 at fixed $\gamma { = } 2 . 0 \dot { . }$ . Right: Sweeping the temporal decay from 0.5 to 3.5 at fixed $B { = } 3 0 7 \breve { 2 } \cdot$ . 

## 5.5 Ablation Study

Effect of training data scale. Figure 14 shows how Qwen-RobotNav performance scales with the fraction of navigation training data used during training. Across representative benchmarks, increasing the data fraction from 12.5% to 100% leads to clear overall gains, most notably on instruction-following and driving tasks. The gains are particularly pronounced on long-horizon instruction-following tasks such as VLN-CE RxR, suggesting that broader trajectory coverage improves grounding between linguistic instructions and extended visual histories. On shorter-horizon tasks such as target tracking (EVT-Bench), performance improves rapidly with moderate data and then saturates with mild fluctuations, indicating that these tasks are less sensitive to additional training data once basic tracking behavior has been learned. Importantly, the full-data model remains competitive while preserving the stronger gains obtained on long-horizon and cross-embodiment tasks. 

Effect of token budget and temporal decay. Figure 15 examines the two primary control parameters of the task-adaptive observation encoding: the total visual token budget B and the temporal decay factor γ. We evaluate Qwen-RobotNav-4B on 500 VLN-CE R2R Val-Unseen episodes, reporting SR, SPL, nDTW, and OSR. In the token budget sweep (left panel, $\gamma { = } 2 . 0 )$ , increasing B substantially improves performance over the low-budget setting, though the benefit is not strictly monotonic once the budget becomes large. SR improves from 70.8% at $B { = } 2 \widetilde { 0 } 4 8$ to 74.6% at $B { = } 4 6 0 8$ , while OSR rises from 78.9% and peaks at 82.7% when B=3584 before slightly decreasing at the largest budget. This pattern indicates that retaining more visual tokens generally provides richer spatial context and improves goal-reaching ability, but excessive or poorly allocated visual context can yield diminishing returns. In the gamma sweep (right panel, B=3072), OSR shows a clear overall improvement from 78.8% $( \gamma { = } 0 . 5 )$ to 82.6% (γ=3.5), while SR peaks at 72.5% $( \gamma { = } 3 . 0 )$ before slightly declining. Larger γ concentrates more tokens on recent frames, enhancing the model’s ability to resolve the current scene at the expense of early-history context. For instruction-following tasks such as VLN-CE, where the agent must react to its immediate surroundings while still respecting earlier landmarks, this recency bias is beneficial but exhibits a trade-off: oracle success continues to improve at high γ, whereas strict success and path-quality metrics saturate or fluctuate slightly. 

## 5.6 Real-World Deployment Results

To further validate the practical applicability of Qwen-RobotNav, we deploy it on a quadruped robot in a previously unseen, real-world exhibition hall. This environment is highly out-of-distribution, as the hall encompasses various types of environments such as living rooms, medical rooms, and open corridors, presenting a rich mixture of heterogeneous scenes and objects that are absent from the training distribution. 

As shown in Figure 16, we evaluate a vision-and-language navigation task using pure language instructions without any goal images or coordinates. The robot is commanded to navigate from a living room area to a medical room located 21.78 m away. Throughout the trajectory, the model leverages different visual landmarks, such as furniture, doorways, and signage, to ground language instructions into spatial decisions, successfully traversing multiple visually distinct zones to reach the designated target area. 

We further demonstrate fine-grained language-based control by issuing a reverse language command that instructs the robot to walk backward. Upon receiving this command, the model switches its locomotion mode and accurately executes the reverse trajectory, retracing the entire route until it returns to a position closely matching its initial starting pose. This result highlights three key capabilities: (1) Qwen-RobotNav can faithfully interpret and execute diverse motion primitives, including nonstandard behaviors such as backward locomotion, purely from natural language; (2) the model effectively utilizes visual landmarks encountered during forward navigation to maintain spatial awareness along the return path; and (3) when directed to a previously visited location, the model can accurately return to that exact position, demonstrating precise spatial grounding in a cluttered, previously unseen environment. 

We also evaluate Qwen-RobotNav in an indoor apartment setting, as shown in Figure 17. The key finding is that the model’s behavior can be precisely controlled through natural language commands. For instance, when instructed to stop at the nightstand on the left side of the bed, the robot navigates into the bedroom and halts at the exact specified side; when instructed to turn around before exiting to the living room, the robot faithfully completes the detour rather than taking a direct path. In the four representative examples we showcase, the robot consistently translates fine-grained verbal directives into accurate navigation behaviors across multiple rooms, confirming that Qwen-RobotNav provides reliable and precise language-based control in complex indoor environments. We further evaluate the agentic navigation interface in a real indoor environment with a mobile robot. As shown in Figure 18, the user gives an open-ended instruction: the robot should check whether a green umbrella was left at Cotti Coffee and also report salient observations along the way. Unlike a short route-following command, this task requires the system to infer the destination from language, maintain task progress over many decision turns, use visual landmarks for localization, and update its plan as new evidence is collected. The selected turns in Figure 18 summarize the high-level agent loop rather than every low-level control step. The agent first parses the user request and observes the initial scene to establish its starting position. It then updates its memory, identifies useful landmarks, and decides to follow the corridor toward the likely store area. During navigation, the agent records intermediate cues such as the Alibaba logo, uses them to refine its localization, and continues searching for the exact Cotti Coffee location. After reaching the store area, the robot inspects the scene and observes a green umbrella, allowing the agent to produce the final response that the umbrella appears to still be there. 

This example illustrates the intended division of labor in the agentic Qwen-RobotNav system. The upper-level agent decomposes the open-ended user request into successive sub-goals and maintains an evidence notebook across turns, while Qwen-RobotNav executes grounded navigation segments 

You are currently in the living room. Turn around and walk straight ahead. Keep walking until you see a supermarket. Just before entering the supermarket, turn right. You will see a large glass sign that says "Training zone". Continue walking straight forward. On your left, you will see a black desk. After you reach the black desk, turn right. You will see a hospital room. Walk straight into the room and stop next to the bed. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/d9b6b40d26bf69c80c6ed4bc9205cac99f4950fa964e3b32ff04ec1f7af8f063.jpg)


You are currently in a hospital room. Please turn around and walk straight ahead. You will see a robot sitting on the chair; turn left there and continue walking forward. Ahead of you, there will be a standing a green zone. Before you enter the green zone, you will see a black table on your left. Walk past the black table and turn left, and you will see a living room. Enter the living room and stop beside the sofa. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/a0e15c8854be87da58a4c85f1a4f94c3325d42947ade9c587342269f4980ed26.jpg)



Figure 16: Real-world VLN deployment in an unseen exhibition hall. The robot dog navigates 21.78 m from a living room to a medical room following pure language instructions, leveraging different visual landmarks along the route. Upon receiving a reverse language command, the robot precisely walks backward to its original starting position.


and returns trajectory evidence for subsequent planning. The real-world episode shows that the same interface can support long-horizon navigation, landmark-based reasoning, memory update, and final evidence-grounded response generation outside simulated benchmark settings. 

Walk straight to the front of the table, turn left, and go around the bed. When you reach the door, turn right and head out into the living room. Once you are there, you will see a chair. Walk straight toward the side of the chair, turn slightly to the right, and pass the chair. Finally, come to a stop in front of the sofa. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/7ee9a8fd93e5c31c28344b0192b8011741c1aa7ad51ff88fa1a67e7240009e4f.jpg)


Turn around and walk straight to head out the door. Turn left in front of the coffee table and continue straight ahead. Pass between the cabinet on your left and the mirror on your right. Once you reach the bathroom, turn right to enter. Stop in front of the toilet. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/325e9a2b1cfd42a75f77dd1305ca3e4f15b9d925dcf5cb4fa8dd211cfd945e27.jpg)


Move forward, then turn right at the cabinet, and go into the bedroom. Once you are in the bedroom, turn left cross the bed, then turn right, please stop in front of the nightstand on the left side of the bed. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/fd17a212040562f01555fb6db97490c477e12d2165cb42f01360694c6caccdfb.jpg)


You are currently in the bathroom. Turn left, exit the door, then turn left. Walk straight into the living room and continue forward until you reach the small sofa, then stop. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/7a8c4c08bdf245f0165ba450b08c0748577ac2e6c821cc793e8f042fccc63e6e.jpg)



Figure 17: Indoor deployment with verbal commands. The robot executes navigation tasks in an apartment setting using step-by-step verbal instructions, traversing between the bedroom, living room, and bathroom while responding to fine-grained spatial directives.


## 6 Conclusion

We have presented Qwen-RobotNav, a unified navigation model built on Qwen3-VL that reframes the central challenge of multi-task navigation as a context modeling problem rather than an architecture "I thinkImay have left my green umbrella somewhere at Cotti Coffee earlier. Could you go check whether it is stillthere? Also, please note anything interesting you see along the way." 


Turn 1


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/e23cdd65df97d8cc6cbc40eebe35b5748cfd0925271e864474fd852d9842b76f.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/eeaa9b07053b7793ae133afc6b7d722ebf123232338b1c823200518761096a8f.jpg)



Turn 3


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/7def414b49f6a49dcbd363d4380d298ece34fda6497617a8a7d776fa25b5a172.jpg)



Turn 4


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/568b3280c74069475057e9be83ab7da183cfb1e9f8d5c67047d0f83238615e66.jpg)



Turn 7


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/ab7660d1f51a85c10f372463fb78c99e576362faa237522097694cf7385bfede.jpg)



Turn 21


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/5eb8f0849516409e2b1e3b2ad04cba7a568229ee3d8cead43ec94556cfbb2d96.jpg)



Turn 29


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/e1951d65e99f73a68ddd9e7fc0c56ff15a3d592133b1908b494007c652c69b30.jpg)



Turn 34


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/7ee3351f982cf23f6c6ae1e40e9ffcaf15a1f8e0cf7df5ddcff40eec3917bdbd.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-06-19/62fc0c79-ec50-4eb4-9f81-b84b549960a6/768139ca72faa733d8c6965f5abe02954b87830ad7088bf4893c80656b38ffa1.jpg)


Yes, there appears to be a green umbrella at Cotti Coffee. 

Figure 18: Real-world long-horizon navigation with agentic Qwen-RobotNav. On a real robot, the agent answers an open-ended request by decomposing the task into sub-goals, following landmarks to Cotti Coffee, and verifying the green umbrella from visual evidence. Selected turns show the loop of upper-level planning, Qwen-RobotNav execution, memory updates, and final response generation. 

or task-head problem. Our key insight is that diverse navigation tasks, from instruction following to target tracking to autonomous driving, share the same perception and planning backbone but demand fundamentally different strategies for consuming the observation stream. Rather than committing to a single fixed strategy, Qwen-RobotNav exposes a parameterised observation encoding interface that allows an outer agent or human operator to dynamically select the appropriate context modeling strategy for each sub-task at inference time, with training-time randomisation ensuring robustness to any configuration without retraining. 

This design principle yields two broader implications. First, it transforms a navigation model from a fixed policy into a reconfigurable navigation primitive: the same model seamlessly switches between global-history mode for exploration and recency-focused mode for precise local manoeuvring, enabling complex long-horizon behaviours to emerge from the composition of simple, well-defined working modes. Second, natural-language viewpoint and temporal identification demonstrate that architectural simplicity can be a strength: by communicating structure through ordinary vocabulary tokens rather than learned positional embeddings, the model preserves its open-world language grounding at zero additional cost. 



Angel X. Chang, Angela Dai, Thomas A. Funkhouser, Maciej Halber, Matthias Nießner, Manolis Savva, Shuran Song, Andy Zeng, and Yinda Zhang. Matterport3d: Learning from RGB-D data in indoor environments. In 2017 International Conference on 3D Vision, 3DV 2017, Qingdao, China, October 10- 12, 2017, pp. 667–676. IEEE Computer Society, 2017. doi: 10.1109/3DV.2017.00081. URL https: //doi.org/10.1109/3DV.2017.00081. 





Sahar Kazemzadeh, Vicente Ordonez, Mark Matten, and Tamara L. Berg. Referitgame: Referring to objects in photographs of natural scenes. In Proceedings of the 2014 Conference on Empirical Methods in Natural Language Processing, EMNLP 2014, October 25-29, 2014, Doha, Qatar, A meeting of SIGDAT, a Special Interest Group of the ACL, pp. 787–798. ACL, 2014. doi: 10.3115/V1/D14-1086. URL https: //doi.org/10.3115/v1/d14-1086. 





Ilya Loshchilov and Frank Hutter. Decoupled weight decay regularization. In 7th International Conference on Learning Representations, ICLR 2019, New Orleans, LA, USA, May 6-9, 2019. OpenReview.net, 2019. URL https://openreview.net/forum?id=Bkg6RiCqY7. 





Michael Tschannen, Alexey Gritsenko, Xiao Wang, Muhammad Ferjad Naeem, Ibrahim Alabdulmohsin, Nikhil Parthasarathy, Talfan Evans, Lucas Beyer, Ye Xia, Basil Mustafa, et al. Siglip 2: Multilingual vision-language encoders with improved semantic understanding, localization, and dense features. arXiv preprint arXiv:2502.14786, 2025. 





Hang Yin, Haoyu Wei, Xiuwei Xu, Wenxuan Guo, Jie Zhou, and Jiwen Lu. Gc-vln: Instruction as graph constraints for training-free vision-and-language navigation. arXiv preprint arXiv:2509.10454, 2025a. 





Gengze Zhou, Yicong Hong, Zun Wang, Xin Eric Wang, and Qi Wu. Navgpt-2: Unleashing navigational reasoning capability for large vision-language models. In European Conference on Computer Vision, pp. 260–278. Springer, 2024a. 



Extensive experiments across navigation, tracking, autonomous driving, and embodied question answering validate these principles, and zero-shot transfer to real-world robots confirms that the unified design generalises beyond simulation. We believe the task-adaptive observation encoding introduced here, treating observation context as a first-class, externally controllable degree of freedom, offers a promising direction for building navigation foundation models that are both broadly capable and practically deployable within agentic navigation systems. 



Yitong Chen, Lingchen Meng, Wujian Peng, Zuxuan Wu, and Yu-Gang Jiang. Comp: Continual multimodal pre-training for vision foundation models. arXiv preprint arXiv:2503.18931, 2025a. 





Jacob Krantz, Erik Wijmans, Arjun Majumdar, Dhruv Batra, and Stefan Lee. Beyond the nav-graph: Vision-and-language navigation in continuous environments. In European Conference on Computer Vision (ECCV), 2020. 





Arjun Majumdar, Anurag Ajay, Xiaohan Zhang, Pranav Putta, Sriram Yenamandra, Mikael Henaff, Sneha Silwal, Paul Mcvay, Oleksandr Maksymets, Sergio Arnaud, Karmesh Yadav, Qiyang Li, Ben Newman, Mohit Sharma, Vincent Berges, Shiqi Zhang, Pulkit Agrawal, Yonatan Bisk, Dhruv Batra, Mrinal Kalakrishnan, Franziska Meier, Chris Paxton, Alexander Sax, and Aravind Rajeswaran. Openeqa: Embodied question answering in the era of foundation models. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 16488–16498, 2024. 





Junyang Wang, Shuxun Zou, Qianli Liu, Jiangming Zou, Hui Li, and Jianxin Liu. GN0: Unified end-to-end training paradigm for vision-and-language navigation. arXiv preprint arXiv:2606.03233, 2026. 





Hang Yin, Xiuwei Xu, Linqing Zhao, Ziwei Wang, Jie Zhou, and Jiwen Lu. Unigoal: Towards universal zero-shot goal-oriented navigation. arXiv preprint arXiv:2503.10630, 2025b. 





Gengze Zhou, Yicong Hong, and Qi Wu. Navgpt: Explicit reasoning in vision-and-language navigation with large language models. In Proceedings of the AAAI Conference on Artificial Intelligence, volume 38, pp. 7641–7649, 2024b. 



## 7 Contributions and Acknowledgments



Yuntao Chen, Yuqi Wang, and Zhaoxiang Zhang. Drivinggpt: Unifying driving world modeling and planning with multi-modal autoregressive transformers. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 26890–26900, 2025b. 





Alexander Ku, Peter Anderson, Roma Patel, Eugene Ie, and Jason Baldridge. Room-across-room: Multilingual vision-and-language navigation with dense spatiotemporal grounding. In Proceedings of the 2020 Conference on Empirical Methods in Natural Language Processing (EMNLP), pp. 4392–4412, 2020. 





Lingchen Meng, Jianwei Yang, Rui Tian, Xiyang Dai, Zuxuan Wu, Jianfeng Gao, and Yu-Gang Jiang. Deepstack: Deeply stacking visual tokens is surprisingly simple and effective for lmms. In Advances in Neural Information Processing Systems, volume 37, pp. 23464–23487, 2024. 





Liuyi Wang, Xinyuan Xia, Hui Zhao, Hanqing Wang, Tai Wang, Yilun Chen, Chengju Liu, Qijun Chen, and Jiangmiao Pang. Rethinking the embodied gap in vision-and-language navigation: A holistic study of physical and visual disparities. CoRR, abs/2507.13019, 2025a. doi: 10.48550/ARXIV.2507.13019. URL https://doi.org/10.48550/arXiv.2507.13019. 





Hang Yin, Jiazhao Zhang, Yinan Liang, Jiahang Liu, Minghan Li, and He Wang. Alldaynav: Lifelong navigation via real-world reinforcement learning. arXiv preprint arXiv:2606.10927, 2026. 





Gengze Zhou, Yicong Hong, Zun Wang, Chongyang Zhao, Mohit Bansal, and Qi Wu. Same: Learning generic language-guided visual navigation with state-adaptive mixture of experts. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 7794–7807, 2025a. 



Core Contributors: Jiazhao Zhang*†, Gengze Zhou*†, Hale Yin*, Yiyang Huang*, Zixing Lei*, Qihang Peng*, Haoqi Yuan, Jie Zhang, Xudong Guo, Xiaoyue Chen, An Yang, Fei Huang, Junyang Lin, Dayiheng Liu, Jingren Zhou, Chenxu Lv‡, Chenfei Wu‡, Xiong-Hui Chen‡ 



An-Chieh Cheng, Yandong Ji, Zhaojing Yang, Xueyan Zou, Jan Kautz, Erdem Biyik, Hongxu Yin, Sifei Liu, and Xiaolong Wang. NaVILA: Legged robot vision-language-action model for navigation. In Robotics: Science and Systems (RSS), 2025. 





Yuxuan Kuang, Hai Lin, and Meng Jiang. Openfmnav: Towards open-set zero-shot object navigation via vision-language foundation models. arXiv preprint arXiv:2402.10670, 2024. 





Dujun Nie, Xianda Guo, Yiqun Duan, Ruijun Zhang, and Long Chen. Wmnav: Integrating visionlanguage models into world models for object goal navigation. arXiv preprint arXiv:2503.02247, 2025. 





Shaoan Wang, Jiazhao Zhang, Minghan Li, Jiahang Liu, Anqi Li, Kui Wu, Fangwei Zhong, Junzhi Yu, Zhizheng Zhang, and He Wang. TrackVLA: Embodied visual tracking in the wild. arXiv preprint arXiv:2505.23189, 2025b. 





Naoki Yokoyama, Sehoon Ha, Dhruv Batra, Jiuguang Wang, and Bernadette Bucher. Vlfm: Visionlanguage frontier maps for zero-shot semantic navigation. In 2024 IEEE International Conference on Robotics and Automation (ICRA), pp. 42–48. IEEE, 2024a. 





Zewei Zhou, Tianhui Cai, Seth Z Zhao, Yun Zhang, Zhiyu Huang, Bolei Zhou, and Jiaqi Ma. Autovla: A vision-language-action model for end-to-end autonomous driving with adaptive reasoning and reinforcement fine-tuning. arXiv preprint arXiv:2506.13757, 2025b. 



Contributors: Zhuoyuan Yu, Jingyang Fan, Zhixuan Liang, Pei Lin, Ye Wang, Anzhe Chen, Kun Yan, Xiao Xu, Jiahao Li, Lulu Hu, Minying Zhang, Shurui Li, Wenhu Xiao, Shuai Bai, Xuancheng Ren 



Kashyap Chitta, Aditya Prakash, Bernhard Jaeger, Zehao Yu, Katrin Renz, and Andreas Geiger. Transfuser: Imitation with transformer-based sensor fusion for autonomous driving. IEEE transactions on pattern analysis and machine intelligence, 45(11):12878–12895, 2022. 





Chengyang Li, Shuai Wang, Kejiang Ye, Weijie Yuan, Boyu Zhou, Yik-Chung Wu, Cheng-Zhong Xu, and Huseyin Arslan. Memory centric power allocation for multi-agent embodied question answering. CoRR, abs/2604.17810, 2026a. 





Qihang Peng, Henry Zheng, and Gao Huang. Proxytransformation: Preshaping point cloud manifold with proxy attention for 3d visual grounding. In Proceedings of the Computer Vision and Pattern Recognition Conference, pp. 24582–24592, 2025. 





Zihan Wang, Xiangyang Li, Jiahao Yang, Yeqi Liu, Junjie Hu, Ming Jiang, and Shuqiang Jiang. Lookahead exploration with neural radiance representation for continuous vision-language navigation. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pp. 13753–13762, 2024. 





Naoki Yokoyama, Ram Ramrakhya, Abhishek Das, Dhruv Batra, and Sehoon Ha. HM3D-OVON: A dataset and benchmark for open-vocabulary object goal navigation. arXiv preprint arXiv:2409.14296, 2024b. 





Fengda Zhu, Xiwen Liang, Yi Zhu, Qizhi Yu, Xiaojun Chang, and Xiaodan Liang. Soon: Scenario oriented object navigation with graph-based exploration. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 12689–12699, June 2021. 



Acknowledgments: We acknowledge the National Pilot Base for Embodied Intelligence Applications for providing the real-robot experimental environment and equipment. We appreciate that Unitree provides their Unitree Go 2 for our experiments. 



OpenScene Contributors. Openscene: The largest up-to-date 3d occupancy prediction benchmark in autonomous driving. https://github.com/OpenDriveLab/OpenScene, 2023. 





Kailin Li, Zhenxin Li, Shiyi Lan, Yuan Xie, Zhizhong Zhang, Jiayi Liu, Zuxuan Wu, Zhiding Yu, and Jose M Alvarez. Hydra-mdp++: Advancing end-to-end driving via expert-guided hydra-distillation. arXiv preprint arXiv:2503.12820, 2025a. 





Qihang Peng, Xuesong Chen, Chenye Yang, Shaoshuai Shi, and Hongsheng Li. Colavla: Leveraging cognitive latent reasoning for hierarchical parallel trajectory planning in autonomous driving. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 17809– 17819, June 2026. 





Zun Wang, Jialu Li, Yicong Hong, Songze Li, Kunchang Li, Shoubin Yu, Yi Wang, Yu Qiao, Yali Wang, Mohit Bansal, and Limin Wang. Bootstrapping language-guided navigation learning with self-refining data flywheel. In International Conference on Learning Representations, volume 2025, pp. 23542–23568, 2025c. 





Chengran Yuan, Zhanqi Zhang, Jiawei Sun, Shuo Sun, Zefan Huang, Christina Dao Wen Lee, Dongen Li, Yuhang Han, Anthony Wong, Keng Peng Tee, et al. Drama: An efficient end-to-end motion planner for autonomous driving with mamba. arXiv preprint arXiv:2408.03601, 2024. 





Ziyu Zhu, Xilin Wang, Yixuan Li, Zhuofan Zhang, Xiaojian Ma, Yixin Chen, Baoxiong Jia, Wei Liang, Qian Yu, Zhidong Deng, et al. Move to understand a 3d scene: Bridging visual grounding and exploration for efficient and versatile embodied navigation. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 8120–8132, 2025. 



## References



Abhishek Das, Samyak Datta, Georgia Gkioxari, Stefan Lee, Devi Parikh, and Dhruv Batra. Embodied question answering. In Proceedings of the IEEE conference on computer vision and pattern recognition, pp. 1–10, 2018. 





Pengxiang Li, Yinan Zheng, Yue Wang, Huimin Wang, Hang Zhao, Jingjing Liu, Xianyuan Zhan, Kun Zhan, and XianPeng Lang. Discrete diffusion for reflective vision-language-action models in autonomous driving. In The Fourteenth International Conference on Learning Representations, 2026b. URL https://openreview.net/forum?id=XJxXSMLDoZ. 





Yuankai Qi, Qi Wu, Peter Anderson, Xin Wang, William Yang Wang, Chunhua Shen, and Anton van den Hengel. Reverie: Remote embodied visual referring expression in real indoor environments. In CVPR, 2020. 





Meng Wei, Chenyang Wan, Jiaqi Peng, Xiqian Yu, Yuqiang Yang, Delin Feng, Wenzhe Cai, Chenming Zhu, Tai Wang, Jiangmiao Pang, et al. Ground slow, move fast: A dual-system foundation model for generalizable vision-and-language navigation. arXiv preprint arXiv:2512.08186, 2025a. 





Kuo-Hao Zeng, Zichen Zhang, Kiana Ehsani, Rose Hendrix, Jordi Salvador, Alvaro Herrasti, Ross Girshick, Aniruddha Kembhavi, and Luca Weihs. Poliformer: Scaling on-policy rl with transformers results in masterful navigators. arXiv preprint arXiv:2406.20083, 2024. 





Yash Goyal, Tejas Khot, Douglas Summers-Stay, Dhruv Batra, and Devi Parikh. Making the v in vqa matter: Elevating the role of image understanding in visual question answering. In CVPR, 2017. 





Yingyan Li, Lue Fan, Jiawei He, Yuqi Wang, Yuntao Chen, Zhaoxiang Zhang, and Tieniu Tan. Enhancing end-to-end autonomous driving with latent world model. arXiv preprint arXiv:2406.08481, 2024. 





Santhosh Kumar Ramakrishnan, Aaron Gokaslan, Erik Wijmans, Oleksandr Maksymets, Alexander Clegg, John M. Turner, Eric Undersander, Wojciech Galuba, Andrew Westbury, Angel X. Chang, Manolis Savva, Yili Zhao, and Dhruv Batra. Habitat-matterport 3d dataset (HM3D): 1000 large-scale 3d environments for embodied AI. In Proceedings of the Neural Information Processing Systems Track on Datasets and Benchmarks 1, NeurIPS Datasets and Benchmarks 2021, December 2021, virtual, 2021. URL https://datasets-benchmarks-proceedings.neurips.cc/paper/2021/hash/ 34173cb38f07f89ddbebc2ac9128303f-Abstract-round2.html. 





Meng Wei, Chenyang Wan, Xiqian Yu, Tai Wang, Yuqiang Yang, Xiaohan Mao, Chenming Zhu, Wenzhe Cai, Hanqing Wang, Yilun Chen, et al. Streamvln: Streaming vision-and-language navigation via slowfast context modeling. arXiv preprint arXiv:2507.05240, 2025b. 





Haochen Zhang, Nirav Savaliya, Faizan Siddiqui, and Enna Sachdeva. FAST-EQA: efficient embodied question answering with global and local region relevancy. In IEEE/CVF Winter Conference on Applications of Computer Vision, WACV 2026, Tucson, AZ, USA, March 6-10, 2026, pp. 1664–1673. IEEE, 2026. 





AMAP CV Lab. ABot-N0: Technical report on the vla foundation model for versatile embodied navigation. arXiv preprint, 2025. 





Monika Gupta, Ritu Tiwari, and G Laxmidhar Raja. A novel policy for multi-robot navigation using image-based visual servoing. In Proceedings of the International Conference on Informatics and Analytics, 2016. 





Yingyan Li, Yuqi Wang, Yang Liu, Jiawei He, Lue Fan, and Zhaoxiang Zhang. End-to-end driving with online trajectory evaluation via bev world model. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 27137–27146, 2025b. 





Allen Z. Ren, Jaden Clark, Anushri Dixit, Masha Itkina, Anirudha Majumdar, and Dorsa Sadigh. Explore until confident: Efficient exploration for embodied question answering. In Dana Kulic, Gentiane Venture, Kostas E. Bekris, and Enrique Coronado (eds.), Robotics: Science and Systems XX, Delft, The Netherlands, July 15-19, 2024, 2024. 





Xinshuo Weng, Boris Ivanovic, Yan Wang, Yue Wang, and Marco Pavone. Para-drive: Parallelized architecture for real-time autonomous driving. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pp. 15449–15458, 2024. 





Jiazhao Zhang, Kunyu Wang, Rongtao Xu, Gengze Zhou, Yicong Hong, Xiaomeng Fang, Qi Wu, Zhizheng Zhang, and He Wang. Navid: Video-based vlm plans the next step for vision-and-language navigation. arXiv preprint arXiv:2402.15852, 2024a. 





Dong An, Hanqing Wang, Wenguan Wang, Zun Wang, Yan Huang, Keji He, and Liang Wang. Etpnav: Evolving topological planning for vision-language navigation in continuous environments. IEEE Transactions on Pattern Analysis and Machine Intelligence, 2024. 





Weijing Hu, Jun Wang, Teng Hu, Jiteng Chen, Siwen Xue, Yufeng Yue, Haoran Xie, Weixun Zhang, Huchuan Lu, Zongqing Lu, Haibin He, and Bolei Wang. OmniNav: A unified framework for prospective exploration and visual-language navigation. arXiv preprint arXiv:2510.06436, 2025. 





Yongkang Li, Kaixin Xiong, Xiangyu Guo, Fang Li, Sixu Yan, Gangwei Xu, Lijun Zhou, Long Chen, Haiyang Sun, Bing Wang, et al. Recogdrive: A reinforced cognitive framework for end-to-end autonomous driving. arXiv preprint arXiv:2506.08052, 2025c. 





Manolis Savva, Jitendra Malik, Devi Parikh, Dhruv Batra, Abhishek Kadian, Oleksandr Maksymets, Yili Zhao, Erik Wijmans, Bhavana Jain, Julian Straub, Jia Liu, and Vladlen Koltun. Habitat: A platform for embodied AI research. In 2019 IEEE/CVF International Conference on Computer Vision, ICCV 2019, Seoul, Korea (South), October 27 - November 2, 2019, pp. 9338–9346. IEEE, 2019. doi: 10.1109/ICCV.2019.00943. URL https://doi.org/10.1109/ICCV.2019.00943. 





Erik Wijmans, Abhishek Kadian, Ari Morcos, Stefan Lee, Irfan Essa, Devi Parikh, Manolis Savva, and Dhruv Batra. DD-PPO: Learning near-perfect pointgoal navigators from 2.5 billion frames. In International Conference on Learning Representations (ICLR), 2020. 





Jiazhao Zhang, Anqi Li, Yunpeng Qi, Minghan Li, Jiahang Liu, Shaoan Wang, Haoran Liu, Gengze Zhou, Yuze Wu, Xingxing Li, et al. Embodied navigation foundation model. arXiv preprint arXiv:2509.12129, 2025a. 





Peter Anderson, Qi Wu, Damien Teney, Jake Bruce, Mark Johnson, Niko Sünderhauf, Ian Reid, Stephen Gould, and Anton Van Den Hengel. Vision-and-language navigation: Interpreting visually-grounded navigation instructions in real environments. In CVPR, 2018. 





Weijing Hu, Jun Wang, Teng Hu, Jiteng Chen, Siwen Xue, Yufeng Yue, Yanyun Wu, Haibin He, Bolei Wang, Huchuan Lu, and Zongqing Lu. AstraNav-World: World model for foresight control and consistency. arXiv preprint arXiv:2603.23745, 2026. 





Zhenxin Li, Shihao Wang, Shiyi Lan, Zhiding Yu, Zuxuan Wu, and Jose M Alvarez. Hydra-next: Robust closed-loop driving with open-loop training. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 27305–27314, 2025d. 





Saumya Saxena, Blake Buchanan, Chris Paxton, Bingqing Chen, Narunas Vaskevicius, Luigi Palmieri, Jonathan Francis, and Oliver Kroemer. Grapheqa: Using 3d semantic scene graphs for real-time embodied question answering. CoRR, abs/2412.14480, 2024. 





Chenfei Wu, Jiahao Li, Jingren Zhou, Junyang Lin, Kaiyuan Gao, Kun Yan, Shengming Yin, Shuai Bai, Xiao Xu, Yilei Chen, Yuxiang Chen, Zecheng Tang, Zekai Zhang, Zhengyi Wang, An Yang, Bowen Yu, Chen Cheng, Dayiheng Liu, Deqing Li, Hang Zhang, Hao Meng, Hu Wei, Jingyuan Ni, Kai Chen, Kuan Cao, Liang Peng, Lin Qu, Minggang Wu, Peng Wang, Shuting Yu, Tingkun Wen, Wensen Feng, Xiaoxiao Xu, Yi Wang, Yichang Zhang, Yongqiang Zhu, Yujia Wu, Yuxuan Cai, and Zenan Liu. Qwen-image technical report. CoRR, abs/2508.02324, 2025. doi: 10.48550/ARXIV.2508.02324. URL https://doi.org/10.48550/arXiv.2508.02324. 





Jiazhao Zhang, Kunyu Wang, Shaoan Wang, Minghan Li, Haoran Liu, Songlin Wei, Zhongyuan Wang, Zhizheng Zhang, and He Wang. Uni-NaVid: A video-based vision-language-action model for unifying embodied navigation tasks. Robotics: Science and Systems, 2025b. 





Dhruv Batra, Aaron Gokaslan, Aniruddha Kembhavi, Oleksandr Maksymets, Roozbeh Mottaghi, Manolis Savva, Alexander Toshev, and Erik Wijmans. ObjectNav revisited: On evaluation of embodied agents navigating to objects. In arXiv preprint arXiv:2006.13171, 2020. 





Yihan Hu, Jiazhi Yang, Li Chen, Keyu Li, Chonghao Sima, Xizhou Zhu, Siqi Chai, Senyao Du, Tianwei Lin, Wenhai Wang, et al. Planning-oriented autonomous driving. In Proceedings of the IEEE/CVF conference on computer vision and pattern recognition, pp. 17853–17862, 2023. 





Zhixuan Liang, Yuxiao Chen, Yurong You, Peter Karkus, Wenhao Ding, Boyi Li, Alexander Popov, Yan Wang, Maximilian Igl, Yiming Li, et al. Planning-aligned token compression for long-context autonomous driving. arXiv preprint arXiv:2606.07464, 2026. 





Shuai Shao, Zeming Li, Tianyuan Zhang, Chao Peng, Gang Yu, Xiangyu Zhang, Jing Li, and Jian Sun. Objects365: A large-scale, high-quality dataset for object detection. In Proceedings of the IEEE/CVF international conference on computer vision, pp. 8430–8439, 2019. 





Ziyuan Xia, Jingyi Xu, Chong Cui, Yuanhong Yu, Jiazhao Zhang, Qingsong Yan, Tao Ni, Junbo Chen, Xiaowei Zhou, Hujun Bao, et al. Habitat-gs: A high-fidelity navigation simulator with dynamic gaussian splatting. arXiv preprint arXiv:2604.12626, 2026. 





Lingfeng Zhang, Qiang Zhang, Hao Wang, Erjia Xiao, Zixuan Jiang, Honglei Chen, and Renjing Xu. Trihelper: Zero-shot object navigation with dynamic assistance, 2024b. URL https://arxiv.org/abs/ 2403.15223. 





Holger Caesar, Varun Bankiti, Alex H Lang, Sourabh Vora, Venice Erin Liong, Qiang Xu, Anush Krishnan, Yu Pan, Giancarlo Baldan, and Oscar Beijbom. nuscenes: A multimodal dataset for autonomous driving. In Proceedings of the IEEE/CVF conference on computer vision and pattern recognition, pp. 11621–11631, 2020. 





Bo Jiang, Shaoyu Chen, Hao Gao, Bencheng Liao, Qian Zhang, Wenyu Liu, and Xinggang Wang. Vadv2: End-to-end vectorized autonomous driving via probabilistic planning. In The Fourteenth International Conference on Learning Representations, 2024. 





Sihao Lin, Zerui Li, Xunyi Zhao, Gengze Zhou, Liuyi Wang, Rong Wei, Rui Tang, Juncheng Li, Hanqing Wang, Jiangmiao Pang, Anton van den Hengel, Jiajun Liu, and Qi Wu. Vlnverse: A benchmark for vision-language navigation with versatile, embodied, realistic simulation and evaluation. CoRR, abs/2512.19021, 2025. doi: 10.48550/ARXIV.2512.19021. URL https://doi.org/10.48550/arXiv.2512. 19021. 





Pei Sun, Henrik Kretzschmar, Xerxes Dotiwalla, Aurelien Chouard, Vijaysai Patnaik, Paul Tsui, James Guo, Yin Zhou, Yuning Chai, Benjamin Caine, Vijay Vasudevan, Wei Han, Jiquan Ngiam, Hang Zhao, Aleksei Timofeev, Scott Ettinger, Maxim Krivokon, Amy Gao, Aditya Joshi, Yu Zhang, Jonathon Shlens, Zhifeng Chen, and Dragomir Anguelov. Scalability in perception for autonomous driving: Waymo open dataset. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 2446–2454, 2020. 





Zebin Xing, Xingyu Zhang, Yang Hu, Bo Jiang, Tong He, Qian Zhang, Xiaoxiao Long, and Wei Yin. Goalflow: Goal-driven flow matching for multimodal trajectories generation in end-to-end autonomous driving. In Proceedings of the Computer Vision and Pattern Recognition Conference, pp. 1602–1611, 2025. 





Tongjie Y Zhang and Ching Y. Suen. A fast parallel algorithm for thinning digital patterns. Communications of the ACM, 27(3):236–239, 1984. 





Wenzhe Cai, Delin Feng, Yu Liu, Jiangmiao Pang, Jiaqi Peng, Chenyang Wan, Hanqing Wang, Liuyi Wang, Tai Wang, Meng Wei, Yuqiang Yang, Xiqian Yu, and Chenming Zhu. Internvla-n1: An open dual-system vision-language navigation foundation model with learned latent plans. Technical report, Shanghai AI Laboratory, 2025. URL https://internrobotics.github.io/internvla-n1.github.io/. 





Kaixuan Jiang, Yang Liu, Weixing Chen, Jingzhou Luo, Ziliang Chen, Ling Pan, Guanbin Li, and Liang Lin. Beyond the destination: A novel benchmark for exploration-aware embodied question answering. In IEEE/CVF International Conference on Computer Vision, ICCV 2025, Honolulu, HI, USA, October 19-25, 2025, pp. 9091–9101. IEEE, 2025. 





Tsung-Yi Lin, Michael Maire, Serge Belongie, James Hays, Pietro Perona, Deva Ramanan, Piotr Dollár, and C Lawrence Zitnick. Microsoft coco: Common objects in context. In ECCV, 2014. 





Jesse Thomason, Michael Murray, Maya Cakmak, and Luke Zettlemoyer. Vision-and-dialog navigation. In Proceedings of the Conference on Robot Learning, volume 100 of Proceedings of Machine Learning Research, pp. 394–406. PMLR, 2020. 





An Yang, Anfeng Li, Baosong Yang, Beichen Zhang, Binyuan Hui, Bo Zheng, Bowen Yu, et al. Qwen3 technical report, 2025a. 





Xunyi Zhao, Gengze Zhou, and Qi Wu. Vln-mme: Diagnosing mllms as language-guided visual navigation agents. arXiv preprint arXiv:2512.24851, 2025. 





Yihan Cao, Jiazhao Zhang, Zhinan Yu, Shuzhen Liu, Zheng Qin, Qin Zou, Bo Du, and Kai Xu. Cognav: Cognitive process modeling for object goal navigation with llms. arXiv preprint arXiv:2412.10439, 2024. 





Justin Johnson, Bharath Hariharan, Laurens van der Maaten, Li Fei-Fei, C Lawrence Zitnick, and Ross Girshick. Clevr: A diagnostic dataset for compositional language and elementary visual reasoning. In CVPR, 2017. 





Jiahang Liu, Yunpeng Qi, Jiazhao Zhang, Minghan Li, Shaoan Wang, Kui Wu, Hanjing Ye, Hong Zhang, Zhibo Chen, Fangwei Zhong, et al. Trackvla++: Unleashing reasoning and memory capabilities in vla models for embodied visual tracking. arXiv preprint arXiv:2510.07134, 2025. 





Yuncong Yang, Han Yang, Jiachen Zhou, Peihao Chen, Hongxin Zhang, Yilun Du, and Chuang Gan. 3dmem: 3d scene memory for embodied exploration and reasoning. In IEEE/CVF Conference on Computer Vision and Pattern Recognition, CVPR 2025, Nashville, TN, USA, June 11-15, 2025, pp. 17294–17303. Computer Vision Foundation / IEEE, 2025b. 





Henry Zheng, Hao Shi, Yong Xien Chng, Rui Huang, Zanlin Ni, Tianyi Tan, Qihang Peng, Yepeng Weng, Zhongchao Shi, and Gao Huang. Denseg: Alleviating vision-language feature sparsity in multi-view 3d visual grounding. In Autonomous Grand Challenge CVPR 2024 Workshop, volume 2, pp. 6, 2024. 





Kushal Kafle, Brian Price, Scott Cohen, and Christopher Kanan. Dvqa: Understanding data visualizations via question answering. In CVPR, 2018. 





Yuxing Long, Wenzhe Cai, Hongcheng Wang, Guanqi Zhan, and Hao Dong. Instructnav: Zero-shot system for generic instruction navigation in unexplored environment. In Pulkit Agrawal, Oliver Kroemer, and Wolfram Burgard (eds.), Conference on Robot Learning, 6-9 November 2024, Munich, Germany, Proceedings of Machine Learning Research, pp. 2049–2060. PMLR, 2024. 





Hang Yin, Xiuwei Xu, Zhenyu Wu, Jie Zhou, and Jiwen Lu. Sg-nav: Online 3d scene graph prompting for llm-based zero-shot object navigation. arXiv preprint arXiv:2410.08189, 2024. 





Henry Zheng, Hao Shi, Qihang Peng, Yong Xien Chng, Rui Huang, Yepeng Weng, zhongchao shi, and Gao Huang. Densegrounding: Improving dense language-vision semantics for ego-centric 3d visual grounding. In The Thirteenth International Conference on Learning Representations, 2025. URL https://openreview.net/forum?id=iGafR0hSln. 





Samira Ebrahimi Kahou, Vincent Michalski, Adam Atkinson, Ákos Kádár, Adam Trischler, and Yoshua Bengio. Figureqa: An annotated figure dataset for visual reasoning. arXiv preprint arXiv:1710.07300, 2018. 





Fangwei Zhong, Kui Wu, Hai Ci, Churan Wang, and Hao Chen. Empowering embodied visual tracking with visual foundation models and offline rl. In European Conference on Computer Vision, pp. 139–155. Springer, 2024. 





Yufeng Zhong, Chengjian Feng, Feng Yan, Fanfan Liu, Liming Zheng, and Lin Ma. Robotrom-nav: A unified framework for embodied navigation integrating perception, planning, and prediction. In Proceedings of the IEEE/CVF International Conference on Computer Vision, pp. 6416–6425, 2025. 

