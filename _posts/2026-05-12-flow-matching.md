---
layout: article
title: "Flow Matching from scratch"
tags: GenerativeAI
pseudocode: true
article_header:
  type: overlay
  theme: light
  background_color: '#f8f9fa'
  background_image:
    gradient: 'linear-gradient(135deg, rgba(52, 152, 219, .3), rgba(155, 89, 182, .3))'
    src: /assets/images/2026-05-12-flow-match/header.png
---

<style>
  /* Left-align all display math in this post only (MathJax v3 CHTML output) */
  mjx-container[display="true"] {
    text-align: left !important;
    margin-left: 0 !important;
  }
  mjx-container[display="true"] > mjx-math {
    margin: 0 !important;
  }
</style>

## Introduction

There are two large branches of generative AI. One is denoising diffusion models and the other is the flow matching. These models are widely used including *Image Generation*, *Video Generation*, and *VLA models (vision language action models)* in robotics. In this posts I just want to explain mathematical background of flow matching. Since I'm also learning Flow Matching myself, it is impossible to show a proper example of it. However, I'm planning to write about example in Flow Matching (A very simple lines of code to run flow matching algorithm). I am very fascinated by recent advancements in Physical AI. So I thought that I should study more about Generative AI. I especially studied the reference of **MIT Class 6.S184: Generative AI With Stochastic Differential Equations, 2026**.

## 1. ODEs

Understanding Flow Matching starts from understanding Orinary Differential Equations (ODEs). ODEs are just equations composed by an unknown function and its derivatives. Due to their deterministicity, it is possible to predict the function value at time $t$.

$\frac{d}{dt}X_{t} = u_{t}(X_{t})$
*A simple example of ODE*

On top of that, it is also true that every ODE can be defined by a *vector field* (functions from vector to vector) like the function $u$ below.

$u:\mathbb{R}^d \times [0,1] \rightarrow \mathbb{R}^d, \: (x, t) \mapsto u_{t}(x)$

For every time $t$ and location $x$, we get a vector $u_{t}(x) \in \mathbb{R}^{d}$ specify a velocity in space. In a sense of Generative AI, $u$ tells where to modify to create a sample $X_1 \sim p_{data}$ from total noise $X_0$.

## 2. Flow

What is the ${X_t}$ when $t \neq 0$? Flow is a function which answers this question by definition below.

$\psi:\mathbb{R}^{d} \times [0,1] \rightarrow \mathbb{R}^{d}, \: (x_0, t) \mapsto \psi_{t}(x_{0})$
$\frac{d}{dt}\psi_{t}(x_{0}) = u_{t}(\psi_{t}(x_{0})) \: (\text{flow ODE})$
$\psi_{0}(x_{0}) = x_{0} \: (\text{flow initial condition})$

Flow is the $\psi_{t}$ which is the mapping from $x_0$ to $X_{t}, \: \forall t \in [0,1]$. Again, in a sense of Generative AI, it means that we can generate $X_{1} (\text{sample})$ from $x_{0} (\text{noise})$.

In this point, some people might think that "Then, how about fitting neural network to this flow?". **Unfortunately, that's a very complex problem due the complexity of $X_1$ and difficulty in maintaining Diffeomorphism(continuously differentiable including the inverse).**

It is also worth noting that the theorem below holds and these conditions are almost always fulfilled in machine learning.

> ### Theorem) Flow existence and uniqueness 
> Vector field $u: \mathbb{R} \times [0,1] \rightarrow \mathbb{R}^{d}$ is continuously differentiable(diff + derivative is also diff) with bounded derivative(Norm of Jacobian is upperbounded)
> $\Rightarrow$
> (1) ODE $u$ has a unique solution $\psi_{t}$
> (2) $\psi_{t}$ is diffeomorphism(continuously diff + continuously diff inverse)

We can wrap up concepts as **"vector fields define ODEs whose solutions are flows"**.

## 3. Flow Models

As I said in the section 2, it is difficult to train neural network to fit flow. **Surprisingly, target of flow matching is not a flow but a vector field**. We can use trained vector field with **Euler Method** to recover $X_1$. In this way, $X_{1} \sim p_{data}$ can be restored from $X_0 \sim p_{init}(\text{usually } \mathcal{N}(0, I_{d}))$. A **flow model** is then described by the ODE

$$
X_0 \sim p_{init}
\frac{d}{dt}X_{t} = u^{\theta}_{t}(X_{t})
$$

And the goal of flow matching is to make the endpoint $X_1$ of the trajectory have distribution $p_{data}$

$$
X_1 \sim p_{data} \; \iff \; \psi^{\theta}_{1}(X_{0}) \sim p_{data} 
$$

If we successfully trained vector field (even though we can't yet), then we can generate a sample with the algorithm below.

{% raw %}
<pre class="pseudocode">
\begin{algorithm}
\caption{Sampling from a Flow Model with Euler method}
\begin{algorithmic}
\REQUIRE Neural network vector field $u_t^\theta$, number of steps $n$
\STATE Set $t = 0$
\STATE Set step size $h = \frac{1}{n}$
\STATE Draw a sample $X_0 \sim p_{\text{init}}$
\FOR{$i = 1, \dots, n$}
  \STATE $X_{t+h} = X_t + h u_t^\theta(X_t)$
  \STATE Update $t \leftarrow t + h$
\ENDFOR
\RETURN $X_1$
\end{algorithmic}
\end{algorithm}
</pre>
{% endraw %}

If we have a marginal vector field ($u_{t}(x)$), it is possible to create a random sample from random noise through Euler method by going through the algorithm 1. This is exactly what we want to do, we do want to find the vector field. But to understand it, we need to follow the steps below.

1. Conditional probability path can be defined with samples (images, videos, texts).
2. We can calculate the conditional vector field from conditional probability path.
3. Conditional probability path and vector field can build marginal vector field with marginal probability path. ($\because$ **Marginalization trick**)
4. Marginal vector field derived by conditional vector field and conditional probability path is a flow model ($\because$ **Continuity Equation**)

## 4. Conditional and Marginal Probability Path

In this section, I am going to explain about 1 and 2.

First off, probability path is the sequence of distributions like $(p_t)_{t \in [0,1]}$ in below.

$$
\forall t \in [0,1], (p_{t})_{t \in [0, 1]} \; (p_t \text{ is a distribution})
$$

And we also can define **conditional (interpolating) probability path** $p_{t}(x\|z)$ over $\mathbb{R}^{d}$ such that

$$
\begin{aligned}
&p_{0}(\cdot \mid z) = p_{init}, \; p_{1}(\cdot \mid z) = \delta_{z} \; \forall z \in \mathbb{R}^d \\\\
&\delta_{z}\text{: Dirac delta distribution}
\end{aligned}
$$

Now let's see the example of **Gaussian probability path**. This is the probability path used by most state-of-the-art models.

$$
\begin{aligned}
&\alpha_{t}, \beta_{t} \text{: noise schedulers (monotonic function) such that} \\\\
&\alpha_{0} = \beta_{1} = 0 \text{ and } \alpha_{1} = \beta_{0} = 1
\end{aligned}
$$

Now we can define Gaussian conditional path like below

$$
\begin{aligned}
&p_{t}(\cdot | z) = \mathcal{N} (\alpha_{t}z, \beta^{2}_{t}I_{d}) \text{ such that} \\\\
&p_{0}(\cdot | z) = \mathcal{N} (\alpha_{0}z, \beta^{2}_{0}I_{d}) = \mathcal{N} (0, I_{d}) \; \text{ and } \; p_{1} (\cdot | z) = \mathcal{N} (\alpha_{1}z, \beta^{2}_{1}I_d) = \delta_{z},
\end{aligned}
$$

Definition above fulfills the condition of conditional probability path and now we can sample from this by adding noise.

$$
\begin{aligned}
& z \sim p_{data}, \; \epsilon \sim p_{init} = \mathcal{N}(0, I_{d}) \Rightarrow x = \alpha_{t}z + \beta_{t}\epsilon \sim p_{t}
&
\end{aligned}
$$

Note that, $p_{t}(x) = \int p_{t}(x \| z) p_{data}(z) dz$ is mathmatically correct. But we don't know the density values $p_{t}(x)$ as the integral is intractable.

## 5. Conditional and Marginal Vector Fields
