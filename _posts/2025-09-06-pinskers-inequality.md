---
title: Pinsker's Inequality
tags: Statistics, RL
article_header:
  type: cover
---

---

## *Th) Pinsker's Inequality*

$\forall$ \( P, Q \): probability distributions on measurable space \( X, \sigma \),


$delta(P, Q) \leq \sqrt{\frac{1}{2} D_{\text{KL}}(P \,\|\, Q)}$


- $\delta(P, Q)$ : Total variation
- $D_{\text{KL}}(P \,\|\, Q)$ : KL divergence

---

Proof)

A special case of Pinsker's Inequality first be proved to prove the above theorem.

To prove above theorem, special case of Pinsker's Inequality should be proved first.

Let

$p(x) := P_P(x)$<br>
$q(x) := P_Q(x)$<br>
$A := \{x | p(x) \geq q(x) \}$

then define random variable

$
Z :=
\begin{cases}
1 & (x \in A)  \\
0 & (x \notin A)
\end{cases}
$

then, below holds

> ## *Th) chain rule of KL divergence*
> $D_{\text{KL}}(P \,\|\, Q) = D_{\text{KL}}(P(Z) \,\|\, Q(Z)) + D_{\text{KL}}(P \,\|\, Q | Z)$
>
> Proof)
> 
> $
> \begin{align}
> D_{\text{KL}}(P(Z) \,\|\, Q(Z))
>     &= P(A) \log \frac{P(A)}{Q(A)} + P(A^c) \log \frac{P(A^c)}{Q(A^c)} \\
>     &= \sum_{x \in A} p(x) \log \frac{P(A)}{Q(A)}
>    + \sum_{x \notin A} p(x) \log \frac{P(A^c)}{Q(A^c)} \quad\cdots\quad \text{(1)}
> \end{align}
> $
> 
> $
\begin{align}
D_{\text{KL}}(P \,\|\, Q \mid Z)
  &= \mathbb{E}_{Z \sim P(Z)} \left[ D_{\text{KL}}(P(P \mid Z=z) \,\|\, P(Q \mid Z=z)) \right] \\
  &= P(A) D_{\text{KL}}(P(P \mid Z=1) \,\|\, P(Q \mid Z=1)) \\
  &+ P(A^c) D_{\text{KL}}(P(P \mid Z=0) \,\|\, P(Q \mid Z=0)) \\
  &= P(A) \sum_{x \in A} p(x \mid z=1) \log \frac{p(x \mid z=1)}{q(x \mid z=1)} \\
  &+ P(A^c) \sum_{x \notin A} p(x \mid z=0) \log \frac{p(x \mid z=0)}{q(x \mid z=0)} \quad\cdots\quad \text{(2)}
\end{align}
$
>
> Combine (1), (2), then
> 
> $D_{\text{KL}}(P(Z) \,\|\, Q(Z)) + D_{\text{KL}}(P \,\|\, Q \mid Z)$
>
> $= \sum_{x \in A} p(x) \log \frac{P(A)}{Q(A)} + \sum_{x \notin A} p(x) \log \frac{P(A^c)}{Q(A^c)} + \sum_{x \in A} p(x) \log \frac{p(x)}{q(x)} \cdot \frac{Q(A)}{P(A)} + \sum_{x \notin A} p(x) \log \frac{p(x)}{q(x)} \cdot \frac{Q(A^c)}{P(A^c)}$
>
> $= \sum_{x \in A} p(x) \left( \log \frac{p(x)}{q(x)} \cdot \frac{Q(A)}{P(A)} + \log \frac{P(A)}{Q(A)} \right) + \sum_{x \notin A} p(x) \left( \log \frac{p(x)}{q(x)} \cdot \frac{Q(A^c)}{P(A^c)} + \log \frac{P(A^c)}{Q(A^c)} \right)$
>
> $= \sum_{x \in \mathcal{X}} p(x) \log \frac{p(x)}{q(x)}$
>
> $= D_{\text{KL}}(P \,\|\, Q)$
>
> $\blacksquare$

---

Let

$
P_A :=
\begin{cases}
1 & \text{w.p. } \sum_{x \in A} p(x) \\
0 & \text{w.p. } \sum_{x \notin A} p(x)
\end{cases}
\quad
Q_A :=
\begin{cases}
1 & \text{w.p. } \sum_{x \in A} q(x) \\
0 & \text{w.p. } \sum_{x \notin A} q(x)
\end{cases}
$

Then,

$
\|P - Q\|_1 = \sum_x |p(x) - q(x)|
$

$
= \sum_{x \in A} (p(x) - q(x)) + \sum_{x \notin A} (q(x) - p(x)) \quad (\text{since } p(x) \geq q(x) \text{ for } x \in A)
$

$
= \left| \sum_{x \in A} p(x) - \sum_{x \in A} q(x) \right| + \left| \sum_{x \notin A} q(x) - \sum_{x \notin A} p(x) \right|
$
