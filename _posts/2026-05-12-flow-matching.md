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

<div class="theorem-box">
<div class="theorem-header">Theorem (Flow Existence and Uniqueness)</div>
<div class="theorem-body" markdown="1">

Vector field $u: \mathbb{R} \times [0,1] \rightarrow \mathbb{R}^{d}$ is continuously differentiable (diff + derivative is also diff) with bounded derivative (Norm of Jacobian is upperbounded) $\Rightarrow$

1. ODE $u$ has a unique solution $\psi_{t}$
2. $\psi_{t}$ is diffeomorphism (continuously diff + continuously diff inverse)

</div>
</div>

We can wrap up concepts as **"vector fields define ODEs whose solutions are flows"**.

## 3. Flow Models

As I said in the section 2, it is difficult to train neural network to fit flow. **Surprisingly, the training target of flow matching is not a flow but a vector field**. We can use trained vector field with **Euler Method** to recover $X_1$. In this way, $X_{1} \sim p_{data}$ can be restored from $X_0 \sim p_{init}(\text{usually } \mathcal{N}(0, I_{d}))$. A **flow model** is then described by the ODE.

$$
\begin{aligned}
&X_{0} \sim p_{init} \\\\
&\frac{d}{dt}X_{t} = u^{\theta}_{t}(X_{t})
\end{aligned}
$$

And the goal of flow matching is to make the endpoint $X_1$ of the trajectory have distribution $p_{data}$ since to be a flow model, it must can sample $X_{1}$ from $X_{0}$.

$$
X_1 \sim p_{data} \;\;\;\; \iff \;\;\;\; \psi^{\theta}_{1}(X_{0}) \sim p_{data} 
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
\forall t \in [0,1], (p_{t})_{t \in [0, 1]} \;\;\;\; (p_t \text{ is a distribution})
$$

And we also can define **conditional (interpolating) probability path** as below.

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

Now let's see the example of **Gaussian probability path**. This is the probability path used by most state-of-the-art models.

<div class="theorem-box definition">
<div class="theorem-header">Definition (Noise Schedulers)</div>
<div class="theorem-body" markdown="1">

$\alpha_{t}, \beta_{t}$ are noise schedulers (monotonic functions) such that

$$
\alpha_{0} = \beta_{1} = 0 \;\;\;\; \text{and} \;\;\;\; \alpha_{1} = \beta_{0} = 1.
$$

</div>
</div>

Now we can define Gaussian conditional path like below.

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

Definition above fulfills the condition of conditional probability path and now we can sample from this by adding noise.

$$
\begin{aligned}
& z \sim p_{data}, \;\;\;\; \epsilon \sim p_{init} = \mathcal{N}(0, I_{d}) \Rightarrow x = \alpha_{t}z + \beta_{t}\epsilon \sim p_{t}
&
\end{aligned}
$$

Note that, $p_{t}(x) = \int p_{t}(x \| z) p_{data}(z) dz$ is mathmatically correct. But we don't know the density values $p_{t}(x)$ as the integral is intractable.

<div class="note-box">
<div class="note-header">Note</div>
<div class="note-body" markdown="1">

The marginal density $p_{t}(x)$ is intractable in practice, but the **marginalization trick** in section 6 will let us bypass it entirely. We only need access to the *conditional* objects $p_{t}(x \mid z)$ and $u_{t}^{target}(x \mid z)$, which we can write down in closed form for Gaussian paths.

</div>
</div>

## 5. Conditional and Marginal Vector Fields

Now, let's define the conditional vector field as below.

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

And we can find the conditional vector field analytically by hand in the case of Gaussian probability path.

$$
\begin{aligned}
&\frac{d}{dt} \psi^{target}_{t} (x | z) = u^{target}_{t} (\psi^{target}_{t} (x | z) | z)\;\;\;\; \forall x, z \in \mathbb{R}^{d} \;\;\;\; (\text{by the definition of the flow}) \\\\
&\iff \;\;\;\; \dot{\alpha}_{t} z + \dot{\beta}_{t} x = u^{target}_{t} (\alpha_{t} z + \beta_{t} x | z) \;\;\;\; \forall x, z \in \mathbb{R}^{d} \\\\
&\iff \;\;\;\; \dot{\alpha}_{t} z + \dot{\beta}_{t} (\frac{x - \alpha_{t} z}{\beta_{t}}) = u^{target}_{t} (x | z) \;\;\;\; \forall x, z \in \mathbb{R}^d
\end{aligned}
$$

## 6. From Conditional Flow Model to Marginal Flow Model

In this section I will explain how conditional probability path and conditional vector field can create a marginal vector field (Marginalization Trick) and how can we prove the marginal vector creates a flow model (Continuity Equation).

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

When it comes to Marginalization Trick, we already know that the theorem holds due to the conditional vector field $u_{t}^{target} (x \| z)$ and conditional probability path $p_{t}(x \| z)$ we already deduced.

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

So now, we can prove **Continuity Equation on Marginal Vector Field**. Proving this will provide a theoriotical background and answer a question about why the flow model holds on marginal vector field and why it is the right way to do. Remember that the reason why we need marginal vector field is that it can generate a random sample from random noise. If the model is a conditional vector field, we can only generate a single pre-determined sample $z \sim p_{data}$.

$$
\begin{aligned}
\partial_{t} p_{t}(x) &= \partial_{t} \int p_{t}(x|z) p_{data}(z) \, dz = \int \partial_{t} p_{t}(x|z) p_{data}(z) \, dz \;\;\;\; (\because \text{definition of }p_{t}(x)) \\\\
&= \int -\text{div}\!\left(p_{t}(\cdot|z) \, u_{t}^{target}(\cdot|z)\right)\!(x) \, p_{data}(z) \, dz \\\\ &(\because \text{continuity eqation for the conditional probability path}) \\\\
&= -\text{div}\!\left(\int p_{t}(x|z) \, u_{t}^{target}(x|z) \, p_{data}(z) \, dz\right) \\\\
&\left(\because x \text{ and } z \text{ are independent, which means } \frac{\partial z}{\partial x} = 0 \right) \\\\
&= -\text{div}\!\left(p_{t}(x) \int u_{t}^{target}(x|z) \, \frac{p_{t}(x|z) \, p_{data}(z)}{p_{t}(x)} \, dz\right)\!(x) \\\\
&(\because \text{Multiplied and devided by } p_{t}(x)) \\\\
&= -\text{div}\!\left(p_{t} \, u_{t}^{target}\right)\!(x) \;\;\;\; (\because \text{Marginalization Trick}) \;\;\;\; \blacksquare \\\\
\end{aligned}
$$

## 7. Learning the Marginal Vector Field

Now we proved the existence of the Marginal Vector Field and satisfying Continuity Equation so that we can generate a data $(X_{1} \sim p_{data})$ from a noise $(X_{0} \sim p_{init})$. As previously stated, the goal of flow matching is to train the neural network $u_{t}^{\theta}$ such that it equals the marginal vector field $u_{t}^{target}$. But, how can we train the marginal vector field? Cut to the chase, **minimizing Conditional Flow Matching Loss defined like below achieves the goal of approaching marginal vector field.**

<div class="theorem-box definition">
<div class="theorem-header">Definition (Flow Matching Loss and Conditional Flow Matching Loss)</div>
<div class="theorem-body" markdown="1">

Let Unif be a Uniform distribution in $[0, 1]$, \\
$p_{t}$: data distribution at time $t$, \\
$u_{t}^{\theta}(x)$: marginal vector field (neural network, object of learning), \\
$u_{t}^{target}(x)$: target marginal vector field (unknown), and \\
$u_{t}^{target}(x \| z)$ target conditional vector field (known)

$$
\begin{aligned}
\mathcal{L}_{\text{FM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, x \sim p_{t}} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x) \|^{2} \, \right] \\\\
&= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x) \|^{2} \, \right], \\\\[0.6em]
\mathcal{L}_{\text{CFM}}(\theta) &= \mathbb{E}_{t \sim \text{Unif},\, z \sim p_{data},\, x \sim p_{t}(\cdot|z)} \left[ \, \| u_{t}^{\theta}(x) - u_{t}^{target}(x|z) \|^{2} \, \right].
\end{aligned}
$$

$i$ means implementing sampling precedure (sample time $t$ $\rightarrow$ sample data $z$ $\rightarrow$ add noise to the data $x \sim p_{t}(\cdot \| z)$).
</div>
</div>

I said that minimizing CFM Loss can minimize FM Loss (approaching $u_{t}^{\theta}(x)$ to $u_{t}^{target}(x)$). Fascinatingly, it turned out that regressing against the tractable, conditional vector field can implicitly regress the intractable, marginal vector field. I will show you the proof with the theorem itself.

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

So we now understood why minimizing the CFM Loss is ultimately minimize the FM Loss. To fulfill the requirements for the flow matching training, we now have left the guaussian form of CFM loss. So let's see how it derived.

## 8. CFM Loss for Gaussian Probability Path

Recall from sections 4 and 5 that for the Gaussian conditional path $p_{t}(\cdot\|z) = \mathcal{N}(\alpha_{t} z, \beta_{t}^{2} I_{d})$, we can sample $x_{t}$ by adding noise:

$$
\epsilon \sim \mathcal{N}(0, I_{d}) \;\;\;\; \Rightarrow \;\;\;\; x_{t} = \alpha_{t} z + \beta_{t} \epsilon \sim \mathcal{N}(\alpha_{t} z, \beta_{t}^{2} I_{d}) = p_{t}(\cdot|z).
$$

And we already derived the closed form of the conditional vector field for this path:

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

The second form is what we actually implement: sample $z$ from data, sample $\epsilon$ from a unit Gaussian, then regress $u_{t}^{\theta}(x)$ against the target

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

This is the most common form used in practice, and it is exactly what we'd implement in code for a minimal flow matching training loop.

Finally, we get the final training loop for flow matching as algorithm 2. And if you finished to training the vector field, you can run simulation through algorithm 1.

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
