---
title: Pinsker's Inequality
tags: ProbabilityTheory RL
article_header:
  type: cover
---

## *Th) Pinsker's Inequality*

$\forall$ \( P, Q \): probability distributions on measurable space $\( U, \Sigma \)$,


$\delta(P, Q) \leq \sqrt{\frac{1}{2} D_{\text{KL}}(P \\| Q)}$


- $\delta(P, Q)$ : Total variation
- $D_{\text{KL}}(P \\| Q)$ : KL divergence

---

Proof)

I only prove for discrete case.

A special case of Pinsker's Inequality first be proved for whole proof.

> ## *Special case of Pinsker's Inequality*
> <p>
> \( P = \begin{cases}
> 1 & \text{w.p. } p \\
> 0 & \text{w.p. } 1-p
> \end{cases} \)
> </p>
>
> <p>
> \( Q = \begin{cases}
> 1 & \text{w.p. } q \\
> 0 & \text{w.p. } 1-q
> \end{cases} \)
> </p>
>
> s.t. $ p \ge q $
> 
> $ \|P-Q\|_1 = \|p-q\| + \|(1-p) - (1-q)\| = 2\|p-q\| = 2(p-q) \quad (\because p \geq q)$
> 
> $f(p,q) = p \log \frac{p}{q} + (1-p)\log \frac{1-p}{1-q} - \frac{1}{2 \ln 2}(2(p-q))^2$
>
> and
> 
> $\frac{\partial f}{\partial q} = \frac{\partial}{\partial q}\left(p\log p - p\log q\right) + \frac{\partial}{\partial q}\left((1-p)(\log(1-p) - \log(1-q))\right) - \frac{\partial}{\partial q}\frac{1}{2\ln 2}(2(p-q))^2$
> 
> $= -\frac{p}{q \ln 2} + \frac{1-p}{(1-q)\ln 2} - \frac{1}{2\ln 2}\cdot 2(2(p-q))(-2)$
> 
> $= \frac{1}{\ln 2}\left(-\frac{p}{q} + \frac{1-p}{1-q}\right) + \frac{4}{\ln 2}(p-q)$
> 
> $= \frac{1}{\ln 2}\left(-\frac{p}{q} + \frac{1-p}{1-q} + 4(p-q)\right)$
> 
> $= -\frac{p-q}{\ln 2}\left(\frac{1}{q(1-q)} - 4\right) \le 0 \quad (\because p \ge q \land \frac{1}{q(1-q)} \ge 4)$
>
> and
> 
> $q = p \implies f(p,q)=0$
> 
> $\therefore f(p,q)\ge 0 \quad (p \ge q)$
>
> which means that
> 
> $f(p,q) = D_{\mathrm{KL}}(P \\| Q) - \tfrac{1}{2 \ln 2} \|P - Q\|_1^2 \ge 0$
>
> $\therefore D_{\mathrm{KL}}(P \\| Q) \ge \tfrac{1}{2 \ln 2} \|P - Q\|_1^2 \quad \cdots \quad (1)$


Let

$p(x) := P_P(x)$<br>
$q(x) := P_Q(x)$<br>
$ A := \\{x \mid p(x) \geq q(x) \\} $

then define random variable

<p>
\( Z(x) := \begin{cases}
1 & (x \in A)  \\
0 & (x \notin A)
\end{cases} \)
</p>

then, below holds

> ## *Th) chain rule of KL divergence*
> $D_{\text{KL}}(P \\| Q) = D_{\text{KL}}(P(Z) \\| Q(Z)) + D_{\text{KL}}(P \\| Q | Z)$
>
> Proof)
> 
> $D_{\text{KL}}(P(Z) \\| Q(Z))$
>
> $= P(A) \log \frac{P(A)}{Q(A)} + P(A^c) \log \frac{P(A^c)}{Q(A^c)}$
>
> $= \sum_{x \in A} p(x) \log \frac{P(A)}{Q(A)} + \sum_{x \notin A} p(x) \log \frac{P(A^c)}{Q(A^c)} \quad\cdots\quad \text{(2)}$
>
> and
> 
> $D_{\text{KL}}(P \\| Q \mid Z)$
>
> <p>
> \(
> = \mathbb{E}_{Z \sim P(Z)}\!\left[
>   D_{\mathrm{KL}}\!\left( P(P \mid Z=z)\,\|\,P(Q \mid Z=z) \right)
> \right] \quad (\text{KL divergence between two conditional probability distributions})
> \)
> </p>
> 
> $= P(A) D_{\text{KL}}(P(P \mid Z=1) \,\\|\, P(Q \mid Z=1)) + P(A^c) D_{\text{KL}}(P(P \mid Z=0) \\| P(Q \mid Z=0))$
>
> $= P(A) \sum_{x \in A} p(x \mid Z=1) \log \frac{p(x \mid Z=1)}{q(x \mid Z=1)} + P(A^c) \sum_{x \notin A} p(x \mid Z=0) \log \frac{p(x \mid Z=0)}{q(x \mid Z=0)}$
>
> $ = \sum_{x\in A} p(x)\,\log\frac{p(x)}{q(x)}\cdot\frac{Q(A)}{P(A)}+\sum_{x\notin A} p(x)\,\log\frac{p(x)}{q(x)}\cdot\frac{Q(A^c)}{P(A^c)} \quad\cdots\quad \text{(3)}$
>
> $ \left( \because p(x \mid Z=1) = \frac{p(x)}{P(A)} \text{, } q(x \mid Z=1) = \frac{q(x)}{Q(A)} \text{, } p(x \mid Z=0) = \frac{p(x)}{P(A^{c})} \text{, } q(x \mid Z=0) = \frac{q(x)}{Q(A^{c})}\right) $
>
> Combine (2), (3), then
> 
> $D_{\text{KL}}(P(Z) \\| Q(Z)) + D_{\text{KL}}(P \\| Q \mid Z)$
>
> $= \sum_{x \in A} p(x) \log \frac{P(A)}{Q(A)} + \sum_{x \notin A} p(x) \log \frac{P(A^c)}{Q(A^c)} + \sum_{x \in A} p(x) \log \frac{p(x)}{q(x)} \cdot \frac{Q(A)}{P(A)} + \sum_{x \notin A} p(x) \log \frac{p(x)}{q(x)} \cdot \frac{Q(A^c)}{P(A^c)}$
>
> $= \sum_{x \in A} p(x) \left( \log \frac{p(x)}{q(x)} \cdot \frac{Q(A)}{P(A)} + \log \frac{P(A)}{Q(A)} \right) + \sum_{x \notin A} p(x) \left( \log \frac{p(x)}{q(x)} \cdot \frac{Q(A^c)}{P(A^c)} + \log \frac{P(A^c)}{Q(A^c)} \right)$
>
> $= \sum_{x \in U} p(x) \log \frac{p(x)}{q(x)}$
>
> $= D_{\text{KL}}(P \\| Q)$
>
> $\blacksquare$

---

Let

<p>
\( P_A := \begin{cases}
1 & \text{w.p. } \sum_{x \in A} p(x) \\
0 & \text{w.p. } \sum_{x \notin A} p(x)
\end{cases} \)
</p>
<p>
\( Q_A :=
\begin{cases}
1 & \text{w.p. } \sum_{x \in A} q(x) \\
0 & \text{w.p. } \sum_{x \notin A} q(x)
\end{cases} \)
</p>

Then,

$
\|P - Q\|_1
$

$
= \sum_x |p(x) - q(x)|
$

$
= \sum_{x \in A} (p(x) - q(x)) + \sum_{x \notin A} (q(x) - p(x)) \quad (\because p(x) \geq q(x) \, \forall x \in A)
$

$
= \left| \sum_{x \in A} p(x) - \sum_{x \in A} q(x) \right| + \left| \sum_{x \notin A} q(x) - \sum_{x \notin A} p(x) \right|
$

$
= |P(P_A = 1) - P(Q_A = 1)| + |P(P_A = 0) - P(Q_A = 0)|
$

$
= \sum_{x \in \\{0,1\\}} |P(P_A = x) - P(Q_A = x)|
$

$
= \|P(P_A) - P(Q_A)\|_1 \quad\cdots\quad (4)
$

Therefore, below holds

$
D_{\mathrm{KL}}(P \\| Q)
$

$
\ge D_{\mathrm{KL}}(P(Z) \\| Q(Z)) \quad (\because\text{Chain rule of KL divergence})
$

$
= D_{\mathrm{KL}}(P(P_A) \\| P(Q_A)) \quad (\because (4))
$

$
\Rightarrow D_{\mathrm{KL}}(P \\| Q) \ge D_{\mathrm{KL}}(P(P_A) \\| P(Q_A))
$

$
\ge \frac{1}{2 \ln 2} \|P(P_A) - P(Q_A)\|_1^2 \quad (\because\text{Special case of pinsker's inequality})
$

$
= \frac{1}{2 \ln 2} \|P - Q\|_1^2 \quad (\because (4))
$

$
\Rightarrow \sqrt{\tfrac{1}{2} D_{\mathrm{KL}}(P \\| Q)} \ge \sqrt{\tfrac{1}{4 \ln 2}} \, \|P - Q\|_1 \quad\cdots\quad (5)
$

$
\text{Let } A^\ast \in \Sigma \quad\text{s.t.}\quad \sup_{A^\ast} \|P(A^\ast) - Q(A^\ast)\| = \|P(A^\ast) - Q(A^\ast)\| \quad (\because\text{Hahn decomposition theory})
$

$
\text{then let } p := P(A^\ast), \; q := Q(A^\ast)
$

$
\|P - Q\|_1 = \|P(A^\ast) - Q(A^\ast)\| + \|P((A^{\ast})^c) - Q((A^{\ast})^c)\|
$

$
= \|P(A^\ast) - Q(A^\ast) - (P((A^{\ast})^c) + Q((A^{\ast})^c))\|
$

$
= \|p - q - (1-p) + (1-q)\|
$

$
= 2(p - q)
$

$
= 2 \delta(P,Q) \quad\cdots\quad (6)
$

$
\therefore \sqrt{\tfrac{1}{2} D_{\mathrm{KL}}(P \\| Q)} \;\;\ge\;\; \sqrt{\tfrac{1}{4 \ln 2}} \, \|P - Q\|_1 \quad (\because(5))
$

$
= \sqrt{\tfrac{1}{4 \ln 2}} \cdot 2 \delta(P,Q) \;\ge\; \delta(P,Q) \quad (\because(6))
$

$
\Rightarrow \sqrt{\tfrac{1}{2} D_{\mathrm{KL}}(P \\| Q)} \ge \delta(P,Q)
$

$\blacksquare$

