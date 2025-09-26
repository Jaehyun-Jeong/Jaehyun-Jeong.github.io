---
title: Monte Carlo Tree Search
tags: RL Open-Loop Planning
article_header:
  type: cover
---

Monte Carlo Tree Search (MCTS) works very well in practice, but it has theoretical difficulty.
In this writing, I want to talk about pseudo code of MCTS, and why this algorithm works.

Open-loop algorithm, like this, can plan future actions from initial state $s_1$, and its primal reason is that they know environments model either stochastically or deterministically. Therefore, we can say typical reinforcement learning algorithm is closed-loop problem which decides action at every step from state $s_t$.


<pre class="mermaid">
flowchart LR

%% ================== Tree 0: 초기 ==================
subgraph T0["Tree 0 (init)"]
direction TB
  t0_s1["s1<br/>Q=0.00<br/>N=0"]
end

%% ================== Tree 1: 1회차 - 좌측 확장 ==================
subgraph T1["Tree 1 (after iter 1: expand left, rollout G1=0.60)"]
direction TB
  t1_s1["s1<br/>Q=0.60<br/>N=1"]
  t1_s1 -->|a1 = 0| t1_s2L["s2<br/>Q=0.60<br/>N=1"]
  t1_s2L -->|a2 = 0| t1_s3L0["s3<br/>Q=0.60<br/>N=1"]
  t1_pi1["π(a_t &#124; s_t)"]
  t1_s3L0 -.-> t1_pi1
end

%% ================== Tree 2: 2회차 - 우측 확장 ==================
subgraph T2["Tree 2 (after iter 2: expand right, rollout G2=0.20)"]
direction TB
  t2_s1["s1<br/>Q=0.40<br/>N=2"]
  t2_s1 -->|a1 = 0| t2_s2L["s2<br/>Q=0.60<br/>N=1"]
  t2_s1 -->|a1 = 1| t2_s2R["s2<br/>Q=0.20<br/>N=1"]
  t2_s2L -->|a2 = 0| t2_s3L0["s3<br/>Q=0.60<br/>N=1"]
  t2_s2R -->|a2 = 1| t2_s3R1["s3<br/>Q=0.20<br/>N=1"]
  t2_pi1["π(a_t &#124; s_t)"]
  t2_pi2["π(a_t &#124; s_t)"]
  t2_s3L0 -.-> t2_pi1
  t2_s3R1 -.-> t2_pi2
end

%% ================== Tree 3: 3회차 - 좌측 a2=1 확장 ==================
subgraph T3["Tree 3 (after iter 3: expand left a2=1, rollout G3=0.90)"]
direction TB
  t3_s1["s1<br/>Q=0.57<br/>N=3"]
  t3_s1 -->|a1 = 0| t3_s2L["s2<br/>Q=0.75<br/>N=2"]
  t3_s1 -->|a1 = 1| t3_s2R["s2<br/>Q=0.20<br/>N=1"]

  t3_s2L -->|a2 = 0| t3_s3L0["s3<br/>Q=0.60<br/>N=1"]
  t3_s2L -->|a2 = 1| t3_s3L1["s3<br/>Q=0.90<br/>N=1"]

  t3_s2R -->|a2 = 0| t3_s3R0["s3<br/>Q=NA<br/>N=0"]
  t3_s2R -->|a2 = 1| t3_s3R1["s3<br/>Q=0.20<br/>N=1"]

  t3_pi1["π(a_t &#124; s_t)"]
  t3_pi2["π(a_t &#124; s_t)"]
  t3_pi3["π(a_t &#124; s_t)"]
  t3_pi4["π(a_t &#124; s_t)"]
  t3_s3L0 -.-> t3_pi1
  t3_s3L1 -.-> t3_pi2
  t3_s3R0 -.-> t3_pi3
  t3_s3R1 -.-> t3_pi4
end

%% ---- 스냅샷 간 진행 (트리 -> 트리 -> 트리 -> 트리) ----
t0_s1 --> t1_s1 --> t2_s1 --> t3_s1
</pre>


> ## UCT TreePolicy($s_t$) *(UCT: Upper Confidence Boundary of Tree)*
>
> If $s_t$ is not fully expanded, choose new $a_t$ (random selection)<br>
> else choose child with best $\text{Score}(s_{t+1})$
>
> $$
> \text{Score}(s_t) = \frac{Q(s_t)}{N(s_t)} + 2C\sqrt{\frac{2ln(N(s_{t-1}))}{N(s_t)}}
> $$
>
> $ Q $: Q-value 
>
> $ N(s_t) $: The number of visitation of s_t
>
> First term identifies how good $s_t$ is compared to the number of visitation, it is a trade off between **exploitation and exploration**.<br>
> Second term helps new state to be visited
