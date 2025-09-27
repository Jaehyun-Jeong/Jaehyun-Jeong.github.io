---
title: Monte Carlo Tree Search
tags: RL Open-Loop Planning
article_header:
  type: cover
pseudocode: true
---

## Intro

Monte Carlo Tree Search (MCTS) works well in practice but poses theoretical challenges.
In this writing, I want to describe MCTS algorithm, and why this algorithm works.

Open-loop planning algorithms like MCTS, can plan future actions from an initial state $s_0$. They assume access to a model of the environment, either stochastically or deterministically. By contrast, typical reinforcement learning algorithm is closed-loop: at each time step it selects an action based on the current state $s_t$

## Intuition

MCTS selects the action expected to yeild the highest return. However, evaluating every state's value is usually infeasible; if all values were known, the agent could simply choose the best action. Thus we must estimate values, and **rollout** make this possible. A single rollout can be inaccurate, but MCTS mitigates this by repeating rollouts and expanding the tree: as the number of visits increases, the estimates become more accurate.

For deep trees, computation can be expensive: with a fixed branching factor (number of actions), the cost grows exponentially with depth. I believe there might be some sort of trade-off methologies. I plan to discuss these after further study.

## Pseudocode

{% raw %}
<pre class="pseudocode">
\begin{algorithm}
\caption{MCTS: Selection–Expansion–Rollout (from $s_0$)}
\begin{algorithmic}
\STATE current $\leftarrow$ $s_0$
\WHILE{not Leaf(current)}
  \STATE current $\leftarrow$ $\arg\max_{s_i \in \mathcal{C}(\text{current})}\; \text{UCB1}(s_i)$
\ENDWHILE
\IF{$N(\text{current}) = 0$}
  \Return Rollout(current) \Comment{unvisited leaf $\rightarrow$ rollout}
\ELSE
  \FOR{each action $a$ available from current}
    \STATE addNewStateToTree(current, $a$)
  \ENDFOR
  \STATE current $\leftarrow$ firstNewChild(current)
  \Return Rollout(current) \Comment{expand then rollout}
\ENDIF
\end{algorithmic}
\end{algorithm}
</pre>
{% endraw %}

{% raw %}
<pre class="pseudocode">
\begin{algorithm}
\caption{Rollout$(s_i)$}
\begin{algorithmic}
\STATE $s \gets s_i$
\WHILE{true}
  \IF{$\operatorname{Terminal}(s)$}
    \Return $\operatorname{Value}(s)$
  \ENDIF
  \STATE $a \gets \operatorname{Random}(\operatorname{AvailableActions}(s))$
  \STATE $s \gets \operatorname{Simulate}(a, s)$
\ENDWHILE
\end{algorithmic}
\end{algorithm}
</pre>
{% endraw %}

**Note.** The pseudocode above shows how to run MCTS algorithm step by step so that the agent can choose the action with the highest estimated value. A node's value is typically computed as the averaged sum of leaves' values including the node's own value.

> ### UCB1
>
> $$
> \text{UCB1}(s_t) = \frac{Q(s_t)}{N(s_t)} + C\sqrt{\frac{ln(N(s_{t-1}))}{N(s_t)}}
> $$
>
> $ Q(s_t) $: cumulative return
>
> $ N(s_t) $: The number of visits
>
> The first term is the empirical mean (exploitation).<br>
> The second term encourages exploration of less-visited nodes and shrinks as $N(s_t)$ grows.

## Example

{% raw %}
<style>
  /* 3행×2열 그리드 (반응형: 좁으면 1열) */
  .mmd-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(320px, 1fr));
    gap: 16px;
    align-items: start;
  }
  @media (max-width: 720px) {
    .mmd-grid { grid-template-columns: 1fr; }
  }

  /* 카드 + 제목 */
  .mmd-card {
    border: 1px solid #D9F99D;
    border-radius: 10px;
    padding: 12px 12px 16px;
    background: #F2FCE7; /* 연노랑 배경(원하면 변경) */
  }
  .mmd-title {
    margin: 0 0 8px;
    text-align: center;
    font-weight: 600;
    font-size: 14px;
  }

  /* Mermaid 중앙 정렬 */
  .mmd-card .mermaid { display: grid; place-items: center; }
  .mmd-card .mermaid > svg { display: block; margin: 0 auto; max-width: 100%; height: auto; }
</style>

<div class="mmd-grid">

  <!-- 1 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 0: Initialization</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T0[" "]
      direction TB
        t0_s0["s0<br/>Q=0<br/>N=0"]
      end
    </div>
  </section>

  <!-- 2 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 1: Expand</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T1[" "]
      direction TB
        t1_s0["s0<br/>Q=0.00<br/>N=0"]
        t1_s0 -->|a1 = 0| t1_s1L["s1<br/>Q=0<br/>N=0"]
        t1_s0 -->|a1 = 1| t1_s1R["s1<br/>Q=0<br/>N=0"]
      end
    </div>
  </section>

  <!-- 3 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 2: Rollout & Backpropagation</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T2[" "]
      direction TB
        t2_s0["s0<br/>Q=20<br/>N=1"]
        t2_s0 -->|a1 = 0| t2_s1L["s1<br/>Q=20<br/>N=1"]
        t2_s0 -->|a1 = 1| t2_s1R["s1<br/>Q=0<br/>N=0"]
        t2_s1L -.->|"π(a_t &#124; s_t)"| t2_terL["s_ter"]
      end
    </div>
  </section>

  <!-- 4 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 3: Rollout & Backpropagation</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T2[" "]
      direction TB
        t2_s0["s0<br/>Q=15<br/>N=2"]
        t2_s0 -->|a1 = 0| t2_s1L["s1<br/>Q=20<br/>N=1"]
        t2_s0 -->|a1 = 1| t2_s1R["s1<br/>Q=10<br/>N=1"]
        t2_s1R -.->|"π(a_t &#124; s_t)"| t2_terR["s_ter"]
      end
    </div>
  </section>

  <!-- 5 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 4: Expand</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T3[" "]
      direction TB
        t3_s0["s0<br/>Q=15<br/>N=2"]
        t3_s0 -->|a1 = 0| t3_s1L["s1<br/>Q=20<br/>N=1"]
        t3_s0 -->|a1 = 1| t3_s1R["s1<br/>Q=10<br/>N=1"]
        t3_s1L -->|a2 = 0| t3_s2LL["s2<br/>Q=0<br/>N=0"]
        t3_s1L -->|a2 = 1| t3_s2LR["s2<br/>Q=0<br/>N=0"]
      end
    </div>
  </section>

  <!-- 6 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 5: Rollout & Backpropagation</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T4[" "]
      direction TB
        t4_s0["s0<br/>Q=10<br/>N=3"]
        t4_s0 -->|a1 = 0| t4_s1L["s1<br/>Q=10<br/>N=2"]
        t4_s0 -->|a1 = 1| t4_s1R["s1<br/>Q=10<br/>N=1"]
        t4_s1L -->|a2 = 0| t4_s2LL["s2<br/>Q=0<br/>N=1"]
        t4_s1L -->|a2 = 1| t4_s2LR["s2<br/>Q=0<br/>N=0"]
        t4_s2LL -.->|"π(a_t &#124; s_t)"| t4_terL["s_ter"]
      end
    </div>
  </section>

  <!-- 7 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 6: Expand</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T5[" "]
      direction TB
        t5_s0["s0<br/>Q=10<br/>N=3"]
        t5_s0 -->|a1 = 0| t5_s1L["s1<br/>Q=10<br/>N=2"]
        t5_s0 -->|a1 = 1| t5_s1R["s1<br/>Q=10<br/>N=1"]
        t5_s1L -->|a2 = 0| t5_s2LL["s2<br/>Q=0<br/>N=1"]
        t5_s1L -->|a2 = 1| t5_s2LR["s2<br/>Q=0<br/>N=0"]
        t5_s1R -->|a2 = 0| t5_s2RL["s2<br/>Q=0<br/>N=0"]
        t5_s1R -->|a2 = 1| t5_s2RR["s2<br/>Q=0<br/>N=0"]
      end
    </div>
  </section>

  <!-- 8 -->
  <section class="mmd-card">
    <h4 class="mmd-title">Step 7: Rollout & Backpropagation</h4>
    <div class="mermaid">
    %%{init:{'flowchart':{'useMaxWidth':false,'htmlLabels':true}}}%%
    flowchart TB
      subgraph T6[" "]
      direction TB
        t6_s0["s0<br/>Q=11<br/>N=4"]
        t6_s0 -->|a1 = 0| t6_s1L["s1<br/>Q=10<br/>N=2"]
        t6_s0 -->|a1 = 1| t6_s1R["s1<br/>Q=12<br/>N=2"]
        t6_s1L -->|a2 = 0| t6_s2LL["s2<br/>Q=0<br/>N=1"]
        t6_s1L -->|a2 = 1| t6_s2LR["s2<br/>Q=0<br/>N=0"]
        t6_s1R -->|a2 = 0| t6_s2RL["s2<br/>Q=14<br/>N=1"]
        t6_s1R -->|a2 = 1| t6_s2RR["s2<br/>Q=0<br/>N=0"]
        t6_s2RL -.->|"π(a_t &#124; s_t)"| t6_terL["s_ter"]
      end
    </div>
  </section>
</div>
{% endraw %}
