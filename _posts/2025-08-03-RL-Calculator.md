---
title: RL Calculator
tags: RL
---

# Deep RL Calculator Project

## 1. Introduction

이 프로젝트는 계산 문제를 강화학습으로 해결하고자 하는 프로젝트이다.<br/>

## 2. Discrete Action Space

### 2.1. Environment

계산을 Discrete 하게 처리하기 위해 환경은 다음과 같이 만들었다.<br/><br/>

- **숫자는 1부터 10까지만 나온다.** (classification 문제를 간단하게 만들기 위해서다.)<br/>
- **나누기는 하지 않는다.** (정수가 아닌 값이 나오므로, class의 개수를 특정하기 복잡하기 때문이다.)<br/><br/>

각각의 계산을 agent에 넣어주기 위해 one-hot encoding을 사용했다. 즉, 계산은 (3,) 모양을 가진다.<br/>
> '+'의 경우 [1, 0, 0], '-'의 경우 [0, 1, 0], '\*'의 경우 [0, 0, 1]<br/><br/>

**따라서 계산과 숫자를 포함해 state는 (5,) 모양을 가진다.**<br/><br/>

**이 환경은 Single Step Episode라는 특징을 가지므로 다음과 같은 사항을**

### 2.2 Agent

이제, 위 환경의 조건을 가지고 다음과 같은 agent를 만들 수 있다.<br/><br/>

- **1 - 10 = -9가 가장 작고, 10 * 10 = 100이 가장 크다. 따라서 가능한 행동의 수는 110가지이다**<br/>
- **문제를 맞히면 보상이 1, 못 맞추면 보상이 0이다.**<br/><br/>

### 2.3. DQN

이 환경을 해결하기 위해 DQN 알고리즘을 사용했다.<br/><br/>

행동 선택은 epsilon greedy를 사용했고, epsilon scheduling에는 exponential scheduling을 사용했다. 파라미터는 다음과 같다.<br/>

```python
params_dict = {
    'device': device, # device to use, 'cuda' or 'cpu'
    'env': env,
    'model': model, # torch models for policy and value funciton
    'optimizer': optimizer, # torch optimizer
    'maxTimesteps': 1000, # maximum timesteps agent take 
    'discount': 0, # step-size for updating Q value
    'maxMemory': 50000,
    'numBatch': 32,
    'useTensorboard': True,
    'tensorboardParams': {
        'logdir': "../../runs/DQN_Calculator_v0",
        'tag': "Averaged Returns/ANN_Cal_lr=1e-4"
    },
    'actionParams': {
        # for DISCRETE
        'algorithm': "greedy",  # greedy, stochastic
        'exploring': "epsilon",  # epsilon, None
        'exploringParams': {
            'start': 0.95,
            'end': 0.05,
            'decay': 1000000,
        },
    },
    'verbose': 2,
}
```

**위 코드에서 discount가 0임에 주목하자.**<br/>
![](./static/DQN_discount.jpg)<br/>
> **이는 위 알고리즘에서 discount rate를 0으로 만들어 미래의 보상을 고려하지 않게 만든다. 왜냐하면 계산은 하나의 상태(문제)가 다음 상태(문제) 보상에 영항을 주지 않기 때문이다.**<br/>

### 2.4. 결과

![](./static/DQN_100000.png)<br/>
![](./static/DQN_1000000.png)<br/>
*timestep 100000과 1000000의 결과는 위와 같다.*<br/><br/>

**학습이 완료된 후의 결과는 다음과 같다.**<br/>
![](./static/DQN_4000000.png)<br/>
![](./static/calculator_results.PNG)<br/>

---

If you like this post, don't forget to give me a star. :star2:

[![Star This Project](https://img.shields.io/github/stars/kitian616/jekyll-TeXt-theme.svg?label=Stars&style=social)](https://github.com/Jaehyun-Jeong/rl_operator)
