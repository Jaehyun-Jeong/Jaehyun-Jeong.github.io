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
graph TB

%% ============ ROW 1 ============
subgraph R1L["Iter 1 - Selection / Expansion / Simulation"]
  direction TB
  r1_s1["s1"]
  r1_s1 -- "a1 = 0" --> r1_s2["s2"]
  r1_s2 -- "a2 = 0" --> r1_s3["s3"]
  r1_s3 -. "rollout  pi(a_t | s_t)" .-> r1_pi["pi(a_t | s_t)"]
end

subgraph R1R["Iter 1 - Backpropagation (update N,Q)"]
  direction TB
  r1b_s1["s1"]
  r1b_s1 -- "a1 = 0 | N=1, Q=10" --> r1b_s2["s2"]
  r1b_s2 -- "a2 = 0 | N=1, Q=10" --> r1b_s3["s3"]
  r1b_s3 -.-> r1b_pi["pi(a_t | s_t)"]
end

r1_s1 --> r1b_s1  %% same-row alignment

%% ============ ROW 2 ============
subgraph R2L["Iter 2 - Selection / Expansion / Simulation"]
  direction TB
  r2_s1["s1"]
  r2_s1 -- "a1 = 1" --> r2_s2["s2"]
  r2_s2 -- "a2 = 1" --> r2_s3["s3"]
  r2_s3 -. "rollout  pi(a_t | s_t)" .-> r2_pi["pi(a_t | s_t)"]
end

subgraph R2R["Iter 2 - Backpropagation (aggregate)"]
  direction TB
  r2b_s1["s1"]
  r2b_s1 -- "a1 = 0 | N=1, Q=10" --> r2b_s2L["s2"]
  r2b_s1 -- "a1 = 1 | N=1, Q=12" --> r2b_s2R["s2"]
  r2b_s2L -- "a2 = 0 | N=1, Q=10" --> r2b_s3L["s3"]
  r2b_s2R -- "a2 = 1 | N=1, Q=12" --> r2b_s3R["s3"]
  r2b_s3L -.-> r2b_piL["pi(a_t | s_t)"]
  r2b_s3R -.-> r2b_piR["pi(a_t | s_t)"]
end

r2_s1 --> r2b_s1      %% same-row alignment
r1_s1 --> r2_s1       %% left column top->down
r1b_s1 --> r2b_s1     %% right column top->down

%% ============ ROW 3 ============
subgraph R3L["Iter 3 - Best-first Selection / Expansion"]
  direction TB
  r3_s1["s1"]
  r3_s1 -- "a1 = 0 | N=2, Q=11" --> r3_s2L["s2"]
  r3_s1 -- "a1 = 1 | N=1, Q=12" --> r3_s2R["s2"]
  r3_s2L -- "a2 = 1" --> r3_s3L2["s3"]
  r3_s3L2 -. "rollout  pi(a_t | s_t)" .-> r3_pi["pi(a_t | s_t)"]
end

subgraph R3R["After several iters - Choose action by max N"]
  direction TB
  r3b_s1["s1"]
  r3b_s1 -- "a1 = 0 | N=3, Q=11.3" --> r3b_s2L["s2"]
  r3b_s1 -- "a1 = 1 | N=2, Q=12.0" --> r3b_s2R["s2"]
  r3b_s2L -- "a2 = 0 | N=2, Q=11" --> r3b_s3L0["s3"]
  r3b_s2L -- "a2 = 1 | N=1, Q=12" --> r3b_s3L1["s3"]
  r3b_s2R -- "a2 = 0 | N=1, Q=10" --> r3b_s3R0["s3"]
  r3b_s2R -- "a2 = 1 | N=1, Q=16" --> r3b_s3R1["s3"]
  r3b_s3L0 -.-> r3b_pi0["pi(a_t | s_t)"]
  r3b_s3L1 -.-> r3b_pi1["pi(a_t | s_t)"]
  r3b_s3R0 -.-> r3b_pi2["pi(a_t | s_t)"]
  r3b_s3R1 -.-> r3b_pi3["pi(a_t | s_t)"]
end

r3_s1 --> r3b_s1
r2_s1 --> r3_s1
r2b_s1 --> r3b_s1
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
