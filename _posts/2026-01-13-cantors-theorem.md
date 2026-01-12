---
layout: article
title: "Cantor’s Diagonal Argument: Not All Infinities Are Equal"
tags: SetTheory
article_header:
  type: overlay
  theme: light
  background_color: '#f8f9fa'
  background_image:
    gradient: 'linear-gradient(135deg, rgba(52, 152, 219, .3), rgba(155, 89, 182, .3))'
    src: /assets/images/2026-01-13-cantors-theorem/header.png
---


One of the biggest surprises that I encountered while majoring in applied mathematics was the statement that "the cardinal numbers of $\mathbb{N}$ (the set of natural numbers) and $\mathbb{Z}$ (the set of integers) are equal". The cardinal number of a set is defined as the size of the set. For finite sets, if a set $A$ is empty then the cardinal number of $A$ is 0, and if the set $A$ has $k$ elements then the cardinal number of $A$ is $k$. However, for infinite sets $A$ and $B$, their cardinal numbers are equal if and only if there exists a one-to-one correspondence (bijective) from $A$ to $B$. We can deduce that aforementioned statement is true from the definition of cardinal number. The really interesting part of this theorem is that the mismatch between our intuition and a rigorous mathematical concepts, since our brains tend to believe, Intuitively, that the size of $\mathbb{Z}$ should be twice the size of $\mathbb{N}$, plus one.

Additionally, it is also worth noting that "the cardinal numbers of $\mathbb{N}$ and $\mathbb{R}$ are different". This also blew my mind, and it explained how countable and uncountable sets can be different. In this post, I want to prove that the cardinal numbers of $\mathbb{N}$ and $\mathbb{R}$ are different by proving the theorems below.

* An open interval (0, 1) is not denumerable.
* The sets (0, 1) and $\mathbb{R}$ are equipotent. (have the same size)

## Def) Denumerable set

A set $S$ is denumerable if and only if there exists a bijective function from $S$ to $\mathbb{N}$.

**NOTE: in this case, we denote $S \sim \mathbb{N}$, and we say the sets A and B are equipotent.**

## Th 1) The open unit interval (0, 1) of real numbers is nondenumerable

$
\underline{\text{proof}}
$

$
\forall x \in (0, 1) \quad \exists x_1, x_2, x_3 \dots \in \{0, 1, \dots, 9\} \quad \text{s.t.} \quad x = 0.x_1 x_2 x_3 \dots
$

$
(\text{For example, } x = \frac{1}{3} = 0.333\dots \implies x_1 = 3 \land x_2 = 3 \land \dots)
$

$
\text{we will treat repeating zeros } \(\text{such as } \frac{1}{4} = 0.25000\dots\) \text{ by decreasing the last non-zero digit by 1 }
$

$
\(\text{as } \frac{1}{4} = 0.24999\dots\)
$

$
\text{Under this agreement, assume that } (0, 1) \text{ is denumerable so that}
$

$
\exists \text{ bijective } f: \mathbb{N} \to (0, 1) \quad \text{s.t.}
$

$
f(1) = 0.x_{11} x_{12} x_{13} \dots
$

$
f(2) = 0.x_{21} x_{22} x_{23} \dots
$

$
f(3) = 0.x_{31} x_{32} x_{33} \dots
$

$
\vdots
$

$
f(k) = 0.x_{k1} x_{k2} x_{k3} \dots
$

$
\text{and let } z \in (0, 1) \text{ be defined as follows:}
$

$$
z = 0.z_1 z_2 z_3 \dots \quad \text{s.t.} \quad \forall k \in \mathbb{N}, \quad
\begin{cases}
z_k = 1 & (x_{kk} \neq 1) \\
z_k = 2 & (x_{kk} = 1)
\end{cases}
$$

$
\text{then } \forall n \in \mathbb{N}, \quad f(n) \neq z \quad (\because \forall n \in \mathbb{N}, \quad x_{nn} \neq z_n \implies f(n) \neq z)
$

$
\therefore \text{This contradicts our assumption} \quad \blacksquare
$

## Th 2) Open intervals (0, 1) and (-1, 1) are equipotent.

$
1) \ (0,1) \sim (-1,1)
$

$
\underline{\text{proof}}
$

$
\text{The function } f: (0,1) \to (-1,1) \text{ given by } f(x) = 2x - 1 \text{ is one-to-one correspondence.}
$

$
\because
$

$
\forall x_1, x_2 \in (0,1) \quad \text{s.t.} \quad x_1 \neq x_2
$

$
f(x_1) = 2x_1 - 1 \neq 2x_2 - 1 = f(x_2) \  (\because 2x_1 - 1 = 2x_2 - 1 \iff 2x_1 = 2x_2 \iff x_1 = x_2)
$

$
\therefore f \text{ is injective}
$

$
\forall y \in (-1,1) \ \exists x \in (0,1) \quad \text{s.t.} \quad
$

$
y = 2x - 1 \ (\because -1 \lt y \lt 1 \Rightarrow 0 \lt y+1 \lt 2 \Rightarrow 0 \lt \frac{y+1}{2} \lt 1)
$

$
\therefore f \text{ is surjective} \quad \blacksquare
$

## Th 3) The open intervals (-1, 1) and $\mathbb{R}$ are equipotent.

$
2) \ (-1,1) \sim \mathbb{R}
$

$
\underline{\text{proof}}
$

$
\text{The function } g: (-1,1) \to \mathbb{R} \text{ given by } g(x) = \tan(\frac{\pi}{2}x) \text{ is one-to-one correspondence}
$

$
\because
$

$
\forall x_1, x_2 \in (-1,1) \quad \text{s.t.} \quad x_1 \neq x_2 
$

$
g(x_1) \neq g(x_2) \ (\because \tan(x) \text{ is one-to-one correspondence})
$

$
\therefore g \text{ is injective}
$

$
\forall y \in \mathbb{R} \ \exists x \in (-1,1) \quad \text{s.t.} \quad g(x) = y
$

$
\left( \because \exists x' \quad \text{s.t.} \quad \tan(x')=y \text{ and } \frac{\pi}{2}x = x' \Rightarrow x = \frac{2}{\pi}x' \right)
$

$
\therefore g \text{ is surjective} \quad \blacksquare
$

## Conclusion

$
\text{By Th 1, Th 2, and Th 3}
$

$
\mathbb{N} \nsim (0,1) \text{ and } (0,1) \sim \mathbb{R}
$

$
\therefore \mathbb{N} \nsim \mathbb{R} \quad \blacksquare
$
