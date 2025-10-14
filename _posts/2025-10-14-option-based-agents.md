---
title: Option-based Agents
tags: RL Planning
article_header:
  type: cover
pseudocode: true
---

![option-based agent](/assets/images/2025-10-14-option-based-agents/option_fourrooms.gif)<br>
*Running an option-based agent policy, it has 8 options with 2 different options for each room*

## Intro

I have learned about option-based agent from this video, so I chose to learn about option-based agent using four-rooms env.
 
{% include extensions/youtube.html id='eJSoV2fSab4' %}

*I was impressed what he is saying "Learning and planning is very similar, if an agent can learn, then the agent can plan and vice versa.".*

The main idea is that "Choose option(which is temporally extended action) in appropriate timestep or state". Options have their own strategy and action selection methology. When RL addressing **problems which have multiple subproblems**, We could say "What if the agent can choose a different strategy(option) in a diffrent situation?". Option-based methods accomplish this mission by seperating problem while solving ultimate problem(maximizing reword).

I usually prefer to understand using math and source code because of the vagueness in languages. So, I chose [this](https://github.com/TristanBester/options/tree/main) repository since it is short, simple and I have no experience on four-rooms domain.

## Intuition

Option-based RL is sharing common idea with typical RL. Yet, as far as I understand, they have some major differences, such as

1. Choose option instead of action.
    * In four rooms env, actions were already chosen, for each option, so the problem is in here is deciding options.
2. Although chosen action is not optimal, options do not end until $\gamma$ terminates current option.
    * In four rooms env, hallways connects rooms are termination states. It means that option ends and agent should choose next option from 8 different options by comparing their Q-value

## Step-by-Step understanding

```python
a = 1
```
