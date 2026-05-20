---
layout: article
title: "Step by Step Flow Matching"
tags: GenerativeAI
pseudocode: true
article_header:
  type: overlay
  theme: light
  background_color: '#f8f9fa'
  background_image:
    gradient: 'linear-gradient(135deg, rgba(52, 152, 219, .3), rgba(155, 89, 182, .3))'
    src: /assets/images/2026-05-20-flow-match/header.png
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

There are two major branches of generative AI. One is denoising diffusion models, and the other is flow matching. These models are widely used in areas including image generation, video generation, and vision-language-action (VLA) models in robotics. In this post I want to explain the mathematical background of flow matching. Since I am still learning flow matching myself, I have not yet included a worked example. However, I plan to write a worked example of flow matching (a few simple lines of code to run the flow matching algorithm). I am fascinated by recent advances in Physical AI, so I thought I should study Generative AI more deeply. I drew especially on materials from **MIT 6.S184: Generative AI with Stochastic Differential Equations (2026)**.

## 1. ODEs

Understanding flow matching starts with understanding ordinary differential equations (ODEs). An ODE is an equation relating an unknown function to its derivatives. Because ODEs are deterministic, the function value at time $t$ is predictable from its initial condition.

$\frac{d}{dt}X_{t} = u_{t}(X_{t})$
*A simple example of ODE*

Equivalently, every ODE can be defined by a *vector field* (a function from vectors to vectors), such as $u$ below.

$u:\mathbb{R}^d \times [0,1] \rightarrow \mathbb{R}^d, \: (x, t) \mapsto u_{t}(x)$

For every time $t$ and location $x$, we get a vector $u_{t}(x) \in \mathbb{R}^{d}$ that specifies a velocity in space. In the context of Generative AI, $u$ tells us how to move points to create a sample $X_1 \sim p_{data}$ from total noise $X_0$.

## 2. Flow

What is $X_{t}$ when $t \neq 0$? The flow is the function that answers this question, defined as follows.

$\psi:\mathbb{R}^{d} \times [0,1] \rightarrow \mathbb{R}^{d}, \: (x_0, t) \mapsto \psi_{t}(x_{0})$
$\frac{d}{dt}\psi_{t}(x_{0}) = u_{t}(\psi_{t}(x_{0})) \: (\text{flow ODE})$
$\psi_{0}(x_{0}) = x_{0} \: (\text{flow initial condition})$

The flow $\psi_{t}$ is the mapping from $x_{0}$ to $X_{t}$ for all $t \in [0,1]$. Again, in the context of generative AI, this means we can generate a sample $X_{1}$ from noise $x_{0}$.

At this point, you might ask: why not fit a neural network directly to this flow? **This is difficult because of the complexity of $X_{1}$ and the difficulty of maintaining a diffeomorphism (a continuously differentiable map with a continuously differentiable inverse).**

It is also worth noting that the theorem below holds, and its conditions are almost always satisfied in machine learning.

<div class="theorem-box">
<div class="theorem-header">Theorem (Flow Existence and Uniqueness)</div>
<div class="theorem-body" markdown="1">

Vector field $u: \mathbb{R} \times [0,1] \rightarrow \mathbb{R}^{d}$ is continuously differentiable (diff + derivative is also diff) with bounded derivative (Norm of Jacobian is upperbounded) $\Rightarrow$

1. ODE $u$ has a unique solution $\psi_{t}$
2. $\psi_{t}$ is diffeomorphism (continuously diff + continuously diff inverse)

</div>
</div>

We can summarize these concepts as **"vector fields define ODEs whose solutions are flows"**.

## 3. Flow Models

As stated in Section 2, it is difficult to train a neural network to fit the flow. **The training target of flow matching is not the flow itself but the vector field.** We can use the trained vector field with **Euler's Method** to recover $X_1$. In this way, $X_{1} \sim p_{data}$ can be recovered from $X_{0} \sim p_{init}$ (usually $\mathcal{N}(0, I_{d})$). A **flow model** is then described by the ODE.

$$
\begin{aligned}
&X_{0} \sim p_{init} \\\\
&\frac{d}{dt}X_{t} = u^{\theta}_{t}(X_{t})
\end{aligned}
$$

The goal of flow matching is to make the endpoint $X_1$ have distribution $p_{data}$, since a flow model must be able to sample $X_{1}$ from $X_{0}$.

$$
X_1 \sim p_{data} \;\;\;\; \iff \;\;\;\; \psi^{\theta}_{1}(X_{0}) \sim p_{data} 
$$

If we have a successfully trained vector field (even though we cannot do so yet), we can generate a sample with the algorithm below.

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

Given a marginal vector field $u_{t}(x)$, we can create a random sample from random noise via Euler's method by running Algorithm 1. This is exactly what we want to do: find the vector field. However, to understand it, we follow the steps below.

1. The conditional probability path can be defined from samples (images, videos, texts).
2. We can calculate the conditional vector field from the conditional probability path.
3. The conditional probability path and the conditional vector field together build the marginal vector field and the marginal probability path. ($\because$ **Marginalization trick**)
4. The marginal vector field derived from the conditional vector field and conditional probability path is a flow model ($\because$ **Continuity Equation**)

## 4. Conditional and Marginal Probability Path

In this section, I explain points 1 and 2.

A probability path is a sequence of distributions $(p_{t})_{t \in [0,1]}$, as shown below.

$$
\forall t \in [0,1], (p_{t})_{t \in [0, 1]} \;\;\;\; (p_t \text{ is a distribution})
$$

We can also define the **conditional (interpolating) probability path** as follows.

<div class="theorem-box definition">
<div class="theorem-header">Definition (Conditional Probability Path)</div>
<div class="theorem-body" markdown="1">

A conditional (interpolating) probability path $p_{t}(x \mid z)$ over $\mathbb{R}^{d}$ satisfies

$$
\begin{aligned}
&p_{0}(\cdot \mid z) = p_{init}, \;\;\;\; p_{1}(\cdot \mid z) = \delta_{z} \;\;\;\; \forall z \in \mathbb{R}^d \\\\
&\delta_{z}\text{: Dirac delta distribution}
\end{aligned}
$$

</div>
</div>

As an example, consider the **Gaussian probability path**. This is the probability path used by most state-of-the-art models.

<div class="theorem-box definition">
<div class="theorem-header">Definition (Noise Schedulers)</div>
<div class="theorem-body" markdown="1">

$\alpha_{t}, \beta_{t}$ are noise schedulers (monotonic functions) such that

$$
\alpha_{0} = \beta_{1} = 0 \;\;\;\; \text{and} \;\;\;\; \alpha_{1} = \beta_{0} = 1.
$$

</div>
</div>

We can now define the Gaussian conditional path as follows.

<div class="theorem-box definition">
<div class="theorem-header">Definition (Gaussian Conditional Probability Path)</div>
<div class="theorem-body" markdown="1">

$$
\begin{aligned}
&p_{t}(\cdot | z) = \mathcal{N} (\alpha_{t}z, \beta^{2}_{t}I_{d}) \text{ such that} \\\\
&p_{0}(\cdot | z) = \mathcal{N} (\alpha_{0}z, \beta^{2}_{0}I_{d}) = \mathcal{N} (0, I_{d}) \;\;\;\; \text{ and } \;\;\;\; p_{1} (\cdot | z) = \mathcal{N} (\alpha_{1}z, \beta^{2}_{1}I_d) = \delta_{z},
\end{aligned}
$$

</div>
</div>

The definition above satisfies the conditions of a conditional probability path, and we can sample from it by adding noise.

$$
\begin{aligned}
& z \sim p_{data}, \;\;\;\; \epsilon \sim p_{init} = \mathcal{N}(0, I_{d}) \Rightarrow x = \alpha_{t}z + \beta_{t}\epsilon \sim p_{t}
&
\end{aligned}
$$

Note that the identity $p_{t}(x) = \int p_{t}(x \mid z)\, p_{data}(z)\, dz$ holds. However, we do not know the density values $p_{t}(x)$, because the integral is intractable.

<div class="note-box">
<div class="note-header">Note</div>
<div class="note-body" markdown="1">

The marginal density $p_{t}(x)$ is intractable in practice, but the **marginalization trick** in section 6 will let us bypass it entirely. We only need access to the *conditional* objects $p_{t}(x \mid z)$ and $u_{t}^{target}(x \mid z)$, which we can write down in closed form for Gaussian paths.

</div>
</div>

## 5. Conditional and Marginal Vector Fields

We now define the conditional vector field as follows.

<div class="theorem-box definition">
<div class="theorem-header">Definition (Conditional Vector Field)</div>
<div class="theorem-body" markdown="1">

$$
\begin{aligned}
&\forall z \in \mathbb{R}^{d}, \text{ let } u_{t}^{target}(\cdot | z) \text{ denote a conditional vector field such that} \\\\
&X_{0} \sim p_{init}, \frac{d}{dt} X_{t} = u_{t}^{target} (X_{t} | z) \Rightarrow X_{t} \sim p_{t} (\cdot | z) \;\;\;\; (0 \leq t \leq 1)
\end{aligned}
$$

</div>
</div>

For the Gaussian probability path, we can derive the conditional vector field analytically.

$$
\begin{aligned}
&\frac{d}{dt} \psi^{target}_{t} (x | z) = u^{target}_{t} (\psi^{target}_{t} (x | z) | z)\;\;\;\; \forall x, z \in \mathbb{R}^{d} \;\;\;\; (\text{by the definition of the flow}) \\\\
&\iff \;\;\;\; \dot{\alpha}_{t} z + \dot{\beta}_{t} x = u^{target}_{t} (\alpha_{t} z + \beta_{t} x | z) \;\;\;\; \forall x, z \in \mathbb{R}^{d} \\\\
&\iff \;\;\;\; \dot{\alpha}_{t} z + \dot{\beta}_{t} (\frac{x - \alpha_{t} z}{\beta_{t}}) = u^{target}_{t} (x | z) \;\;\;\; \forall x, z \in \mathbb{R}^d
\end{aligned}
$$

## 6. From Conditional Flow Model to Marginal Flow Model

In this section, I explain how the conditional probability path and the conditional vector field together produce a marginal vector field (the marginalization trick), and how to prove that the marginal vector field induces a flow model (the continuity equation).

<div class="theorem-box">
<div class="theorem-header">Theorem (Marginalization Trick)</div>
<div class="theorem-body" markdown="1">

Let $u_{t}^{target}(x \mid z)$ be a conditional vector field. Then the marginal vector field $u_{t}^{target}(x)$ defined as

$$
u_{t}^{target}(x) = \int u_{t}^{target}(x \mid z) \, \frac{p_{t}(x \mid z) \, p_{data}(z)}{p_{t}(x)} \, dz,
$$

follows the marginal probability path, i.e.

$$
X_{0} \sim p_{init}, \;\;\;\; \frac{d}{dt} X_{t} = u_{t}^{target}(X_{t}) \;\;\;\; \Rightarrow \;\;\;\; X_{t} \sim p_{t} \;\;\;\; (0 \leq t \leq 1).
$$

In particular, $X_{1} \sim p_{data}$ for this ODE, so that we might say "$u_{t}^{target}$ converts noise $p_{init}$ into data $p_{data}$".

</div>
</div>

The marginalization trick holds because of the conditional vector field $u_{t}^{target}(x \mid z)$ and the conditional probability path $p_{t}(x \mid z)$ derived earlier.

<div class="theorem-box">
<div class="theorem-header">Theorem (Continuity Equation)</div>
<div class="theorem-body" markdown="1">

Let us consider a flow model with vector field $u_{t}^{target}$ with $X_{0} \sim p_{init} = p_{0}$. Then: 
$$
X_{t} \sim p_{t} \;\;\;\;
\iff \;\;\;\; \partial_{t} p_{t}(x) = -\text{div}(p_{t} u_{t}^{target})(x) \;\;\;\; \forall x \in \mathbb{R}^{d},\, 0 \leq t \leq 1,
$$

where $\partial_{t} p_{t}(x) = \frac{d}{dt} p_{t}(x)$ denotes the time-derivative of $p_{t}(x)$.

</div>
</div>

We can now prove the **continuity equation for the marginal vector field**. This proof provides theoretical justification for why the flow model holds for the marginal vector field and why this approach is correct. Recall that we need the marginal vector field because it can generate a sample from noise drawn at random. If the model is a conditional vector field, we can generate only a single, pre-specified sample $z \sim p_{data}$.

$$
\begin{aligned}
\partial_{t} p_{t}(x) &= \partial_{t} \int p_{t}(x|z) p_{data}(z) \, dz = \int \partial_{t} p_{t}(x|z) p_{data}(z) \, dz \;\;\;\; (\because \text{definition of }p_{t}(x)) \\\\
&= \int -\text{div}\!\left(p_{t}(\cdot|z) \, u_{t}^{target}(\cdot|z)\right)\!(x) \, p_{data}(z) \, dz \\\\ &(\because \text{continuity equation for the conditional probability path}) \\\\
&= -\text{div}\!\left(\int p_{t}(x|z) \, u_{t}^{target}(x|z) \, p_{data}(z) \, dz\right) \\\\
&\left(\because x \text{ and } z \text{ are independent, which means } \frac{\partial z}{\partial x} = 0 \right) \\\\
&= -\text{div}\!\left(p_{t}(x) \int u_{t}^{target}(x|z) \, \frac{p_{t}(x|z) \, p_{data}(z)}{p_{t}(x)} \, dz\right)\!(x) \\\\
&(\because \text{multiplying and dividing by } p_{t}(x)) \\\\
&= -\text{div}\!\left(p_{t} \, u_{t}^{target}\right)\!(x) \;\;\;\; (\because \text{Marginalization Trick}) \;\;\;\; \blacksquare \\\\
\end{aligned}
$$

## 7. Learning the Marginal Vector Field

We have now proved the existence of a marginal vector field satisfying the continuity equation, so we can generate data $X_{1} \sim p_{data}$ from noise $X_{0} \sim p_{init}$. As previously stated, the goal of flow matching is to train the neural network $u_{t}^{\theta}$ such that it equals the marginal vector field $u_{t}^{target}$. But how can we train the marginal vector field? In short, **minimizing the conditional flow matching loss defined below approximates the marginal vector field.**

<div class="theorem-box definition">
<div class="theorem-header">Definition (Flow Matching Loss and Conditional Flow Matching Loss)</div>
<div class="theorem-body" markdown="1">

Let Unif be the uniform distribution on $[0, 1]$, \\
$p_{t}$: data distribution at time $t$, \\
$u_{t}^{\theta}(x)$: marginal vector field (neural network, object of learning), \\
$u_{t}^{target}(x)$: target marginal vector field (unknown), and \\
$u_{t}^{target}(x \| z)$: target conditional vector field (known)

$$
\begin{aligned}
\mathcal{L}_{\text{FM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x) \|^{2} \, \right] \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x) \|^{2} \, \right], \\\\[0.6em]
\mathcal{L}_{\text{CFM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x|z) \|^{2} \, \right].
\end{aligned}
$$

We implement this sampling procedure as: sample time $t$, then data $z$, then noise to obtain $x \sim p_{t}(\cdot \mid z)$.
</div>
</div>

I claimed that minimizing the CFM loss also minimizes the FM loss (driving $u_{t}^{\theta}(x)$ toward $u_{t}^{target}(x)$). Remarkably, regressing against the tractable conditional vector field implicitly regresses against the intractable marginal vector field. I state the theorem first and then prove it.

<div class="theorem-box">
<div class="theorem-header">Theorem (FM Loss = CFM Loss + Constant)</div>
<div class="theorem-body">
<p>The marginal flow matching loss equals the conditional flow matching loss up to a constant. That is,</p>

$$
\mathcal{L}_{\text{FM}}(\theta) = \mathcal{L}_{\text{CFM}}(\theta) + C,
$$

<p>where $C$ is independent of $\theta$. Therefore, their gradients coincide:</p>

$$
\nabla_{\theta} \mathcal{L}_{\text{FM}}(\theta) = \nabla_{\theta} \mathcal{L}_{\text{CFM}}(\theta).
$$

<p>Hence, minimizing $\mathcal{L}_{\text{CFM}}(\theta)$ with e.g. stochastic gradient descent (SGD) is equivalent to minimizing $\mathcal{L}_{\text{FM}}(\theta)$ in the same fashion. In particular, <strong>for the minimizer $\theta^{\ast}$ of $\mathcal{L}_{\text{CFM}}(\theta)$, it will hold that $u_{t}^{\theta^{\ast}} = u_{t}^{target}$, i.e. the neural network will equal the marginal vector field</strong> (assuming an infinitely expressive parameterization).</p>
</div>
</div>

**Proof.** First, expand the squared norm in the FM loss:

$$
\begin{aligned}
\mathcal{L}_{\text{FM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, \| u_{t}^{\theta}(x) - u_{t}^{target}(x) \|^{2} \,\right] (\because \text{definition})\\\\
&= \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, \| u_{t}^{\theta}(x) \|^{2} - 2\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x) + \| u_{t}^{target}(x) \|^{2} \,\right] \;\;\;\; (\because || a - b ||^{2} = ||a||^{2} - 2a^{T}b + ||b||^{2}) \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, \| u_{t}^{\theta}(x) \|^{2} \,\right] - 2\, \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x) \,\right] + \underbrace{\mathbb{E}_{t \sim \text{Unif}_{[0,1]},\, x \sim p_{t}}\!\left[\, \| u_{t}^{target}(x) \|^{2} \,\right]}_{=:\, C_{1}} \;\;\;\; (\because \text{ the last term is not related to } \theta)\\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, \| u_{t}^{\theta}(x) \|^{2} \,\right] - 2\, \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x) \,\right] + C_{1}.
\end{aligned}
$$

The middle term can be rewritten using the marginalization trick:

$$
\begin{aligned}
\mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}}\!\left[\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x) \,\right]
&= \int_{0}^{1} \!\!\int p_{t}(x)\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x) \, dx \, dt \;\;\;\; (\because \text{definition}) \\\\
&= \int_{0}^{1} \!\!\int p_{t}(x)\, u_{t}^{\theta}(x)^{T} \left[\, \int u_{t}^{target}(x|z)\, \frac{p_{t}(x|z)\, p_{data}(z)}{p_{t}(x)} \, dz \,\right] dx \, dt \;\;\;\; (\because \text{Marginalization Trick})\\\\
&= \int_{0}^{1} \!\!\int\!\!\int u_{t}^{\theta}(x)^{T}\, u_{t}^{target}(x|z)\, p_{t}(x|z)\, p_{data}(z) \, dz \, dx \, dt \;\;\; (\because u_{t}^{\theta}(x) \text{ and } p_{t}(x)\text{ are constant in terms of } z)\\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x|z) \,\right]. \;\;\;\; (\because \text{definition})
\end{aligned}
$$

Substituting back and completing the square:

$$
\begin{aligned}
\mathcal{L}_{\text{FM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, \| u_{t}^{\theta}(x) \|^{2} \,\right] - 2\, \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x|z) \,\right] + C_{1} \;\;\;\; (\because \text{substituting the second term}) \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, \| u_{t}^{\theta}(x) \|^{2} - 2\, u_{t}^{\theta}(x)^{T} u_{t}^{target}(x|z) + \| u_{t}^{target}(x|z) \|^{2} - \| u_{t}^{target}(x|z) \|^{2} \,\right] + C_{1} \\\\
&\left(\because \text{linearity of expectation and adding and subtracting } \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}(||u_{t}^{target}(x|z)||^{2}) \right) \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, \| u_{t}^{\theta}(x) - u_{t}^{target}(x|z) \|^{2} \,\right] + \underbrace{\mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)}\!\left[\, -\| u_{t}^{target}(x|z) \|^{2} \,\right]}_{C_{2}} + C_{1} \\\\
&\left( \because || a - b ||^{2} = ||a||^{2} - 2a^{T}b + ||b||^{2} \text{ and the second term is not related to } \theta \right) \\\\
&= \mathcal{L}_{\text{CFM}}(\theta) + \underbrace{C_{2} + C_{1}}_{=:\, C}. \;\;\;\; \blacksquare
\end{aligned}
$$

We have now seen why minimizing the CFM loss ultimately minimizes the FM loss. To complete the flow matching training procedure, all that remains is the Gaussian form of the CFM loss. Let us see how it is derived.

## 8. CFM Loss for Gaussian Probability Path

Recall from sections 4 and 5 that for the Gaussian conditional path $p_{t}(\cdot\|z) = \mathcal{N}(\alpha_{t} z, \beta_{t}^{2} I_{d})$, we can sample $x_{t}$ by adding noise:

$$
\epsilon \sim \mathcal{N}(0, I_{d}) \;\;\;\; \Rightarrow \;\;\;\; x_{t} = \alpha_{t} z + \beta_{t} \epsilon \sim \mathcal{N}(\alpha_{t} z, \beta_{t}^{2} I_{d}) = p_{t}(\cdot|z).
$$

We have already derived the closed form of the conditional vector field for this path:

$$
u_{t}^{target}(x|z) = \left(\dot{\alpha}_{t} - \frac{\dot{\beta}_{t}}{\beta_{t}} \alpha_{t} \right) z + \frac{\dot{\beta}_{t}}{\beta_{t}} x.
$$

Substituting both into the CFM loss gives a directly trainable objective:

$$
\begin{aligned}
\mathcal{L}_{\text{CFM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim \mathcal{N}(\alpha_{t} z, \beta_{t}^{2} I_{d})} \!\left[\, \| u_{t}^{\theta}(x) - \left(\dot{\alpha}_{t} - \frac{\dot{\beta}_{t}}{\beta_{t}} \alpha_{t}\right) z - \frac{\dot{\beta}_{t}}{\beta_{t}} x \|^{2} \,\right] \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, \epsilon \sim \mathcal{N}(0, I_{d})} \!\left[\, \| u_{t}^{\theta}(\alpha_{t} z + \beta_{t} \epsilon) - (\dot{\alpha}_{t} z + \dot{\beta}_{t} \epsilon) \|^{2} \,\right]. \;\;\;\; (\because x = \alpha_{t} z + \beta_{t} \epsilon)
\end{aligned}
$$

The second form is what we actually implement: sample $z$ from the data, sample $\epsilon$ from a unit Gaussian, then regress $u_{t}^{\theta}(x)$ against the target

$$
x = \alpha_{t} z + \beta_{t} \epsilon, \;\;\;\; \text{target} = \dot{\alpha}_{t} z + \dot{\beta}_{t} \epsilon,
$$

both of which we can compute in closed form.

For the simplest **linear interpolant** schedule, take

$$
\alpha_{t} = t, \;\;\;\; \beta_{t} = 1 - t \;\;\;\; \Rightarrow \;\;\;\; \dot{\alpha}_{t} = 1, \;\;\;\; \dot{\beta}_{t} = -1.
$$

The corresponding probability path

$$
p_{t}(x|z) = \mathcal{N}\!\left(tz,\, (1-t)^{2} I_{d}\right)
$$

is sometimes referred to as the **(Gaussian) CondOT probability path** (CondOT = Conditional Optimal Transport). The loss simplifies to:

$$
\mathcal{L}_{\text{cfm}}(\theta) = \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, \epsilon \sim \mathcal{N}(0, I_{d})} \!\left[\, \| u_{t}^{\theta}(tz + (1-t)\epsilon) - (z - \epsilon) \|^{2} \,\right].
$$

This is the most common form used in practice, and it is exactly what we would implement in code for a minimal flow matching training loop.

Finally, we obtain the training loop for flow matching as Algorithm 2. Once the vector field has been trained, you can run the simulation via Algorithm 1.

{% raw %}
<pre class="pseudocode">
\begin{algorithm}
\caption{Flow Matching Training Procedure (for Gaussian CondOT path $p_t(x|z) = \mathcal{N}(tz, (1-t)^2 I_d)$)}
\begin{algorithmic}
\REQUIRE A dataset of samples $z \sim p_{data}$, neural network $u_t^\theta$
\FOR{each mini-batch of data}
  \STATE Sample a data example $z$ from the dataset.
  \STATE Sample a random time $t \sim \text{Unif}_{[0,1]}$.
  \STATE Sample noise $\epsilon \sim \mathcal{N}(0, I_d)$.
  \STATE Set $x = tz + (1-t)\epsilon$ \COMMENT{General case: $x \sim p_t(\cdot|z)$}
  \STATE Compute loss $\mathcal{L}(\theta) = \|u_t^\theta(x) - (z - \epsilon)\|^2$ \COMMENT{General case: $\|u_t^\theta(x) - u_t^{target}(x|z)\|^2$}
  \STATE Update $\theta \leftarrow \text{grad\_update}(\mathcal{L}(\theta))$.
\ENDFOR
\end{algorithmic}
\end{algorithm}
</pre>
{% endraw %}
