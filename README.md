**`reinforo_biblio` =  bibliography on "reinforcement learning for robotics"**

# :dart: scope and ambition

centralize my notes on some works about
- **robotics**
  - here mostly robot arms
- **_learning_** something
  - _learning-based_ as opposed to _hand-crafted_ control
  - mostly `reinforcement learning`
  - :warning: not all papers use _learning_
- **real world**
  - understand the concrete challenges of real-world deployment

---
---


# :mechanical_arm: :mechanical_arm: sim2real - big projects

**`"Learning agile and dynamic motor skills for legged robots"`**

- **[** `2019` **]**
  **[[:memo:](https://arxiv.org/abs/1901.08652)]**
  **[[🎞️](https://www.youtube.com/watch?v=aTDkYFZFWug)]**
  **[[🎞️](https://www.youtube.com/watch?v=Afi17BnSuBM)]**
  **[[:octocat:️](https://github.com/junja94/anymal_science_robotics_supplementary)]**
- **[** _`actuator-net`, `curriculum factors`, `quadrupedal system`, `elastic actuators`, `sim2real`, `randomization`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2019_hwangbo_1.png) | 
|:--:| 
| *__agent__: `state`-to-`action`. **actuator-net**: `action`-to-`torque`. **rigid-body sim**: `torque`-to-`state` [source](https://arxiv.org/abs/1901.08652)* |

| ![](media/2019_hwangbo_1.gif) | 
|:--:| 
| *using a `actuator-net` in the simulator [source](https://www.youtube.com/watch?v=aTDkYFZFWug)* |

| ![](media/2019_hwangbo_2.gif) | 
|:--:| 
| *`command`-conditioned task [source](https://www.youtube.com/watch?v=aTDkYFZFWug)* |

| ![](media/2019_hwangbo_3.gif) | 
|:--:| 
| *`recovery from a fall` task [source](https://www.youtube.com/watch?v=aTDkYFZFWug)* |

| ![](media/2019_hwangbo_2.png) | 
|:--:| 
| *[source](https://arxiv.org/abs/1901.08652)* |

| ![](media/2019_hwangbo_3.png) | 
|:--:| 
| *`reward` (here `cost`) terms for the __`command`-conditioned task__. The __`kc` curriculum factors__ apply on the __constraints__ (not the __main objective__) terms and increase during training from `0` to `1` [source](https://arxiv.org/abs/1901.08652)* |

initial questions
- input / output of policy?
- input / output of actuator-net?
- difference with 6/7-dof arm?
- predicting torques or ref-joint-poses?
- what data is collected for training the actuator-net?
- is `history` needed for both `actuator net` and `policy`? how long?

idea:
- train a **`RL` agent** in a **simulator**
  - agent: `state`-to-`action`
  - (sim 1/2) actuator-net: `action`-to-`torque`
  - (sim 2/2) rigid-body sim: `torque`-to-`state`
- `action` = desired `joint positions`

real hardware
- `32 kg`
- `55 cm` long legs
  - > "`ANYmal` has a much **larger leg length relative to footprint**, making it more dynamic, less statically stable, and therefore more difficult to control"
- `3` actuated DOF -> **`12` joins**
  - **hip** abduction/adduction
  - **hip** flexion/extension
  - **knee** flexion/extension
- **`12` Series-Elastic Actuators** (`SEA`)
  - **electric motor** -> gear reduction -> **elastic element** -> load
- actuation chain: **from `position command` (`action`) to `joint torque`**
  - a **`PD`-controller** converts [`position command`] to [`desired torque`]
    - [_as for_ **_nvidia orbit_**](https://github.com/NVIDIA-Omniverse/Orbit/issues/49)
    - the policy network outputs a `joint position` references, which are converted to `torque` using fixed gains (`kP` and `kD`)
  - a `PID`-controller converts [`desired torque`] to [`desired current`]
  - a Field-Oriented Controller (`FOC`) converts [`desired current`] to [`phase voltage`]
  - this voltage produces the `torque` at the **input of the transmission**
  - the deflection of **elastic element** transfers [`torque` from the output of the transmission] to [`torque` at the **joint**]
- **`policy` frequency**
  - `200 Hz` for `command-conditioned locomotion` / `high-speed locomotion`
  - `100 Hz` for `recovery from a fall`

`RL` sim
- two parts:
  - a **rigid-body simulation** (with `fast contact solver`)
    - in: **joint `torques`**
    - in: current robot `state`
    - out: next `state` (`position` and `velocities`)
    - one challenge: dynamics at intermittent contacts
    - one challenge: **system identification** (`mass`, `dimension` ...). Sol: **parameter randomization**
  - `12` **actuator nets**
    - in: history of **`joint position` errors** (using the `joint position` targets (`action`))
    - in: history of `joint velocities`
    - out: `12` joint `torque` values
- `500k` `time-steps / sec`
  - half `rigid-body-sim` / `half actuator-net`

two general approaches to **bridging the reality gap** (in practice, both)
- `1-` improve simulation **fidelity**
  - either analytically (hard and extremely laborious)
  - or in a data-driven way (`system identification`)
- `2-` **accept the imperfections** of simulation and aim to make the **controller robust** to variations in system properties
  - **randomization**of the simulation: **stochastic policy**, randomizing the **dynamics**, adding **noise to the `observations`**, and **perturbing the system** with **random disturbances**
  - training with **`30` different `ANYmal` models**
    - stochastically sampled **inertial properties**: `center of mass` positions, `masses of links`, `joint positions`

modelling `SEA` is difficult:
- the **actuator dynamics** (especially with **elastics**)
- the `delays` in **control signals** introduced by multiple hardware and software layers
  - > "a complex sensor suite and multiple layers of software bring `noise` and `delays` to information transfer"
  - > "the ideal actuator model assumes that there is **no communication `delay`** and that the actuator can generate any commanded torque **instantly**"
- the low-level **controller dynamics**
- the compliance/damping at the joints

_ - _ - _

actuator-net: **data-collection**
- _what is collected?_
  - `joint position` **errors**
    - `error = actual - desired`
  - `joint velocities`
  - `torques`
  - current `state`
  - history: **last `2` `states`** (`t−0.01` and `t−0.02` sec)
    - how dense/sparse, how far in the past?
    - **tuned** wrt. the **validation error**
- _how is it collected?_
  - **foot trajectories** in the form of a **sine wave**
    - with parameterized controller and **inverse kinematics**
  - the feet constantly **made or break a contact** with the ground
  - **amplitude** in (`5∼10 cm`)
  - **frequency** in (`1∼25 Hz`)
  - **the robot is manually disturbed** during data collection. _how exactly?_
- more than a **million samples**, in `4 min` (at `400 Hz` using the `12` actuators)

actuator-net: training
- `MLP` with `3` hidden layers
  - `32` + `32` + `32`
- the **non-linearity** has a strong effect on performance on the physical system
  - **bounded** activation functions, such as `tanh,` yield **less aggressive trajectories** when **subjected to disturbances**
    - > "**unbounded** activation functions, such as `ReLU`, can **degrade** performance on the real robot, since actions can have **very high magnitude when the robot reaches states that were not visited** during training."
  - [**`softsign` activation function**]():
    - like `sigmoid`, but **(-1, 1)**, and **zero-centred**
    - like `tanh`, but converges **polynomially** (not exponentially)
    - `12.2 µs` with `softsign`
    - `31.6 µs` with `tanh`
- **error** on the **validation** set: `0.74 Nm`
- **error** on the **test** set: `0.96 Nm`
- resolution of the `torque` measurement: `0.2 Nm`

`observation` noise
- `joint velocities` cannot be directly measured on the real robot
  - estimate: **numerically differentiation** of the `position` signal
  - imperfection modelling: a strong noise (`U(−0.5, 0.5) rad/s`) to the `joint velocity` measurements during training
- **noise** added during training to:
  - the **observed linear velocity** (`U(−0.08, 0.08) m/s`)
  - the **angular velocity** (`U(−0.16, 0.16) m/s`) of the base

`dynamics` noise
- not mentioned here, but in [`ANYmal` in `Omniverse`](https://www.youtube.com/watch?v=Afi17BnSuBM) the robot it pushed
  - this **force it to learn robust gaits**

_ - _ - _

policy: a **`joint position` policy** (not `torque`)
- we use the **policy network** as an **`impedance` controller**
  - > "Simpler control methods, such as `position control` or `torque control`, **perform poorly** when the manipulator experiences **contacts**. Thus, **[`impedance control`](https://en.wikipedia.org/wiki/Impedance_control)** is commonly used in these settings."
- out: `actions` are the **low-`impedance` joint position** commands
  - [`mechanical impedance`](https://en.wikipedia.org/wiki/Impedance_control) = `force output` / `motion input`
  - > "If we control the `impedance` of a mechanism, we are controlling the **`force` of resistance** to external motions that are imposed by the environment"
- > "a `position` policy has an advantage (over `torque`) in training since it starts as a **standing controller** whereas a `torque` controller initially creates many trajectories that result in falling. Thus, we use the `policy` network as an `impedance` controller."

`MDP`.`task`
- locomotion, given target `X-Y velocity` and `yaw rate`
- recovery from a fall

`MDP`.`action`
- `joint position` target for the `12` joints

`MDP`.`observation`
  - **history** of the robot's states (`joint velocities` and **`joint position` errors**)
    - _how old?_ sampled at `tk − 0.01 s` and `tk − 0.02 s`
    - very important to **detect contacts**
  - current robot's `state` (`joint velocities` and `joint position`)
  - **previous `action`** (_only one?_)
  - **body** state (`pose`, `twist` and `speed`)
  - **`command`**
  - removing the `velocity` makes the **training fail**
    - > "even though in theory the `policy` network could infer `velocities` as finite differences of observed `positions`"
    - **input normalization** is necessary in most learning procedures

`MDP`.**commands**
- three components:
  - `forward velocity`
  - `lateral velocity`
  - `yaw rate`
- part of the `observation`

`MDP`.`reward`
- complex!
- **curriculum factor** (`kc`)
  - `r` = `r_objective` + `kc_1` * `r_contraint_1` + `kc_2` * `r_contraint_2` + `...`
  - `kc_i` increases from `0` to `1`
  - > "This way, the robot **first** learns **how to achieve the `objective`** and **then** how to **respect various `constraints`**."
  - > "Instead of altering the `samples`, we **alter the `objective`** to control the training difficult"
- note that many `cost` terms are also **multiplied by the time step `∆t`** since we are interested in the **integrated value over time**
  - _strange_

`MDP`.`gamma`
- hyper-param **tuning**
- we used `γ` = `0.9988` which corresponds to a **half-life of `5.77 s`**
  - `γ^n=exp(n*ln(γ))=0.5` for `n=ln(0.5)/ln(γ)=557`
- depends on the timestep
  - _with `dt=0.01`, `n*dt=5.77`_

`MDP`.`termination`
- _`truncation`?_ after **`6` seconds**
- `termination` (`r=-1`): violating **joint limits**
- `termination` (`r=-1`): hitting the ground with the base

`policy` net
- light `MLP`
  - `2` hidden layers (`256` + `128`)
  - **`25 µs` inference** on a single CPU thread

_ - _ - _

performance
- **high speed** motion (**learnt gait** pattern: **trot** with full flight phase)
- fast and flexible **recovery after a fall**
- fewer engineering / expertise than full-modelling
- **fast inference**
- energy-efficiency

differences from [`ANYmal` in `Omniverse`](https://www.youtube.com/watch?v=Afi17BnSuBM)
- `MDP`.`obs`: also the terrain height
- **game-inspired curriculum**: one single env with different difficulties
  - discover what terrains are hard (instead of decide it)
  - the terrain is not re-generated
- `rigid-body sim`: randomize friction coefficient
- `rigid-body sim`: push
- `actuator net`: LSTM

</details>

---

**`"Learning Dexterous In-Hand Manipulation"`**

- **[** `2019` **]**
  **[[:memo:](https://arxiv.org/pdf/1808.00177.pdf)]**
  **[[🎞️](https://openai.com/research/learning-dexterity)]**
  **[[🎞️](https://www.youtube.com/watch?v=6fo5NhnyR8I)]**
- **[** _`openAI`, `randomization`, `RNN`, `privileged learning`, `external pose estimator`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2020_andrychowicz_1.png) | 
|:--:| 
| *many environments are simulated [source](https://www.youtube.com/watch?v=jwSbzNHGflM)* |

| ![](media/2020_andrychowicz_2.png) | 
|:--:| 
| *randomization of __`physics` parameters__. Other parameters are randomized, not just `Physics` [source](https://www.youtube.com/watch?v=jwSbzNHGflM)* |

| ![](media/2020_andrychowicz_3.png) | 
|:--:| 
| *`Asymmetric Actor-Critic`: to make the training easier, the `Value` network can have access to **information that is not available on the real robot system** [source](https://www.youtube.com/watch?v=jwSbzNHGflM)* |

| ![](media/2020_andrychowicz_1.gif) | 
|:--:| 
| *a `pose estimator` is learnt to feed the `pose` (not the `image`) to the `policy`. And __internal sensors__ are not used because __hard to model__. [source](https://www.youtube.com/watch?v=jwSbzNHGflM)* |

ideas
- train in **simulated** environments with **randomization** of
  - `dynamics`: physical properties, e.g. **friction coefficients**
  - `observation`: object's appearance
- key ingredients
  - **extensive randomization**
  - `RNN`: **memory-augmented** control polices
    - > "Many of the randomizations we employ **persist across an episode**, and thus it should be possible for a **memory augmented policy** to **identify properties** of the current environment and adapt its own behavior accordingly"
    - `RNN` in both the `policy` and `value` function
    - `LSTM` state is **predictive of the environment randomization**
  - **distributed `RL`**

control policy
- `obs`
  - cube pose
  - fingertips locations
  - sensors in real world
    - (markers) 16 PhaseSpace tracking cameras
    - (vision) 3 Basler RGB cameras
      - 3 cameras to resolve pose ambiguities that may occur with monocular vision
    - > "we do not use the **touch sensors** embedded in the hand"
      - because they are subject to **`state`-dependent noise** that would have been **difficult to model** in the simulator
- `action`
  - desired `joints angles` relative to the current ones
  - [`24`-DoF `5`-finger Hand](https://www.shadowrobot.com/dexterous-hand-series/)
  - `action` discretized into **`11` bins**
    - > "While `PPO` can handle both `continuous` and `discrete` action spaces, we noticed that **`discrete` `action` spaces** work much better"
- `reward`
  - error in `rotation angles`
  - bonus if success
  - penalty if cube falls
- `dt`
  - forward path in NN: `25ms` 
  - > "We update the targets of the `low`-level controller, which runs at roughly **`1` kHz**, with **relative positions** given by the `control policy` at roughly **`12` Hz**"
  - > "Each environment step corresponds to **`80 ms` of real time** and consists of **`10` consecutive `MuJoCo` steps**, each corresponding to `8 ms`"

**pose estimator**
- a pre-trained **encoder** of the `observation`
- **trained separately** from the `control policy`
- in: 3 cam images
- out: `position` and `orientation`

simulator
- MuJoCo
- Unity for rendering
- the model of the Hand is improved using calibration

[`Rapid`]()
- `PPO` scales up easily and requires **little hyperparameter tuning**
- > The vast majority of training time is spent **making the policy robust** to different physical dynamics
  - one domain: 3 years
  - all domains: 100 years of experience

**privileged** learning
- in `PPO`, the `value` network is **only used during training**
- `Asymmetric Actor-Critic`: value network can have access to **information that is not available on the real robot system**, to make the training easier

**reality gap**
- no **`tendon`-based actuation**
  - rather `torque` applied to `joints`
- no **deformable body contact** models
  - rather **rigid body contact** models 

randomization
- `Observation` noise
  - Gaussian noise to `policy` `obs`
  - > "we apply a `correlated noise` which is sampled **once per episode** as well as an `uncorrelated noise` sampled at **every timestep**"
- Physics randomization
  - `calibration` = `system identification`
  - sampled at the start of each episode and held fixed
- Unmodeled effects
  - > "To account for **imperfect actuation**, we use a simple model of **motor backlash** and introduce `action delays` and `action noise` before applying them in simulation"
  - against short-time **loss of tracking** and marker **occlusion**
    - **freezing the `position`** of a simulated marker with low probability for a short period of time in simulation
  - > "To handle additional unmodeled dynamics, we apply **small random forces** to the object"
- Visual appearance randomization
  - camera positions and intrinsics
  - the pose of the hand and object
  - lighting, materials and textures

</details>

---
---

# :mechanical_arm: sim2real - small projects

**`"Sim-to-Real Reinforcement Learning for Vision-Based Dexterous Manipulation on Humanoids"`**

- **[** `2025` **]**
  **[[🎞️](https://toruowo.github.io/recipe/)]**
- **[** _`sim-to-real`, `learning-from-demonstration`, `multi-fingered hands`, `isaac-sim`_ **]**

<details>
  <summary>Click to expand</summary>

|                    ![](media/2025_lin_1.gif)                     | 
|:----------------------------------------------------------------:| 
| *Some of the tests. [source](https://toruowo.github.io/recipe/)* |

|                               ![](media/2025_lin_1.png)                               | 
|:-------------------------------------------------------------------------------------:| 
| *Four challenges and proposed solutions. [source](https://toruowo.github.io/recipe/)* |

|                                                                                             ![](media/2025_lin_2.png)                                                                                             | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *Overview. In one sentence: **learning-from-demonstrations**, where **demonstrations are collected from learnt RL sub-task policies** in the simulation environment. [source](https://toruowo.github.io/recipe/)* |

|                         ![](media/2025_lin_3.png)                         | 
|:-------------------------------------------------------------------------:| 
| *Domain randomization setup. [source](https://toruowo.github.io/recipe/)* |

|                             ![](media/2025_lin_2.gif)                              | 
|:----------------------------------------------------------------------------------:| 
| *RL helps **discovering strategies**. [source](https://toruowo.github.io/recipe/)* |

**Four challenges** and proposed **solutions**:
- Challenge-1: **environment modeling** when using a **simulator**: **sim-to-real** gap.
  - **Simulation parameters** are **adjusted automatically** to closely match real-world conditions.
  - > "The `autotune` module enables rapid **calibration of simulator parameters** to match real robot behavior by automatically searching the parameter space to identify optimal values for both simulator physics and robot model constants in under four minutes (or `2000` simulated steps in `10 Hz`)."
  - Which parameters? Not detailed.
    - Simulator physics parameters affecting kinematics and dynamics.
    - Robot model constants from the URDF file (including link inertia values, joint limits, and joint/link poses).

- Challenge-2: **reward design** for **contact-rich** and **long-horizon** **manipulation** tasks.
  - The idea is to disentangle a full task into intermediate **"contact goals"** and **"object goals"**.
  - > "each motion sequence to execute a task can be defined as a combination of **hand-object contact** and **object states**."
  - Example for the **handover** task:
    - (1) `contact-state`: hand-A contacting the object
      - -> `contact-goal`: penalizing the distance from fingers to desirable contact points.
    - (2) `object-state`: object being lifted to a position near the hand-B
      - -> `object-goal`: penalizing the distance from the object to the desirable position.
    - (3) `contact-state`: hand-B contacting the object
      - -> `contact-goal`
    - (4) `object-state`: object being transferred to the final goal position
      - -> `object-goal`

- Challenge-3: **Sample efficient** policy learning (**exploration** problem in **sparse reward settings**).
  - **Divide-and-conquer** + **distillation** process: One idea is to divide the challenging task into easier sub-tasks, then **distilling the subtask specialists** into **a generalist policy**.
    - I.e. break down the **explorable state** space itself.
    - Example: a **multi-object** manipulation task can be divided into multiple **single-object** manipulation tasks.
    - > "This effectively brings RL closer to **learning from demonstrations**, where the sub-task policies act as teleoperators for **task data collection** in the simulation environment, and the **generalist policy** acts as a centralized model trained from curated data."
    - > "For each **sub-task specialist policy**, we evaluate for **5000 steps** over **100 environments**, saving trajectories filtered **by success** at episode reset on the hard disk. We then **treat the saved data as "demonstrations"** and learn a generalist policy for each task with [**Diffusion Policies**](https://diffusion-policy.cs.columbia.edu/)."
  - Task-aware hand poses for initialization: A second idea is to start training episodes at **states recorded** from demonstrations (with teleoperation).

- Challenge-4: dilemma in object representation: we need to learn (1) and then transfer (2).
  - Object representations that are **more expressive and information-dense** can improve dexterity and **capability** of the learned policy, they also present a **larger sim-to-real gap**.
  - A mixture of **low-dimensional** (sparse) and **high-dimensional** (dense) object representation is proposed.
    - a low-dimensional **3D object position** (obtained from a **third-view camera** to ensure the object is also in camera view)
    - a high-dimensional **depth image**

Other info about the RL problem formulation:
- `action` space:
  - normalized 7 DoF **absolute desired joint positions** of each humanoid **arm**
  - normalized (0 to 1) 6 DoF desired **joint positions** of each humanoid **hand**
  - the desired joint positions are apparently applied using a **PD controller**.
- Policy's control frequency = **5 Hz**.
- Perception pipeline's frequency = **5 Hz**.

Domain randomization
- Physical randomization:
  - **noise** on:
    - object friction
    - mass
    - scale
  - Also, **random forces** are applied to the object to simulate the physical effects that are **not implemented by the simulator**.
- **Non-physical randomization**: noise in
  - observation (e.g., joint position measurement and detected object positions)
  - action
  - Also, some **lag is applied** to the **action** and the **observation**.

Hardware:
- **[Fourier GR1](https://www.fftai.com/products-gr1) humanoid robot** with two 7-DoF arms and two multi-fingered hands.
  - The Fourier hands have 6-actuated-DoFs and 5-underactuated-DoFs.
  - This means that each hand has a total of 11 DoFs, but **only 6 are directly controlled** via motors.
  - Since the simulator (Isaac Gym) does not natively support **underactuated joints**, the behaviour is approximated mathematically by fitting a linear function `qu = k · qa + b` (`q` is the joint angle).
  - > "We find ourselves heavily constrained by the **lack of reliable hardware** for **dexterous manipulation**. While we use **multi-fingered robot hands**, the dexterity of these hands is far from that of human hands in terms of the **active degrees of freedom**."
- RealSense D435 depth cameras: one for the **third-view** and one for the **robot's head**.

Software:
- NVIDIA **Isaac Gym** simulator.
  - Which RL framework? `rl-games`?
- **Segment Anything Model 2** (`SAM2`) to generate a segmentation mask, which is tracked throughout all remaining frames.
- > "To approximate the 3D center-of-mass coordinates of the object **(sparse representation)**, we calculate the **center position** of their masks in the image plane, then obtain noisy **depth readings** from a depth camera to recover a corresponding 3D position."
- PPO: In addition to the **policy inputs**, the following **privilege state inputs** are **provided to the asymmetric critic**:
  - arm joint velocities,
  - hand joint velocities,
  - all fingertip positions,
  - object orientation,
  - object velocity,
  - object angular velocity,
  - object mass randomization scale,
  - object friction randomization scale,
  - object shape randomization scale.  

</details>

**`"DemoStart: Demonstration-led auto-curriculum applied to sim-to-real with multi-fingered robots"`**

- **[** `2024` **]**
  **[[🎞️](https://sites.google.com/view/demostart)]**
- **[** _`auto-curriculum`, `distillation`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                       ![](media/2024_bauza_1.gif)                                                                       | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The RL `env` is reset on **states along the recorded demonstration**. States must be **informative for the agent**. [source](https://sites.google.com/view/demostart)* |

Main idea of **DemoStart**:
- A **demonstration**-led **auto**-curriculum **RL** method.
- **_"Curriculum"_**: increase the level of the task as the agent progresses. Problem: it can be complicated to **organise the levels**.
- _Why "auto"?_ A system decides the level.
  - The task stays the same, but the **initial state of the agent** can be adjusted.
  - Some successful (yet not perfect) **demonstrations** are recorded in simulation. The RL task can be **instantiated along the trajectory** of the demonstration: if starting close to the end, the task should be easy, and the agent can quickly learn.
- The starting state should **not be too easy**, but **not too hard**. A heuristic is used to decide.

Miscellaneous:
- A **sparse reward** function can be used. -> Easier to define!
- A **three-fingered** robotic hand attached to a Kuka LBR iiwa 14 arm
- Observation space: **RGB images**!
  - The RL agent learns from proprioceptive features (e.g. positions and speeds)
  - The learnt policy (using proprioceptive features) is **distilled** with **behavioural cloning** to be able to work **only with images**.
- Action space: controlling _joint speed_ or _joint position_?
  - _"We use **joint position on all tasks** apart from cube reorientation where we use joint velocity"_
- Simulator: [MuJoCo](https://mujoco.org/) - owned by Google
- **Domain randomization**
  - 1) **Perturbations**: _"We apply external force disturbances to any object"_
  - 2) **Physics**: _"We randomize the friction, mass, and inertia of every joint and body in the scene"_
  - 3) **Visual**: _"Visual: We randomize the camera poses and lighting"_
  - This allows for a zero-shot sim-to-real transfer.
- RBG images are made "realistic" using the **[Filament](https://github.com/google/filament) renderer** and Blender
- How are **demonstrations collected in simulation**?
  - With a [3Dconnexion SpaceMouse](https://3dconnexion.com/uk/product/spacemouse-wireless/)
  - The **action space differs** from the one of RL task. But this is no problem: only the states are needed.
  - 2 to 20 demos were recorded depending on the task.
  - Btw, they also collected demonstrations on the real setup. Not for the training the RL agent. Instead, to train an imitation-leaning model used as baseline.

Comments:
- Collecting successful demonstrations in simulation may not be easy.
- No code available.

</details>

**`"Learning to Grasp the Ungraspable with Emergent Extrinsic Dexterity"`**

- **[** `2022` **]**
  **[[🎞️](https://sites.google.com/view/grasp-ungraspable)]**
- **[** _`non-prehensile manipulation`, `compliant Operational Space Controller`_ **]**

<details>
  <summary>Click to expand</summary>

|          ![](media/2022_zhou_1.gif)          | 
|:--------------------------------------------:| 
| *[source](https://arxiv.org/pdf/2211.01500)* |

The task: **Occluded Grasping using External Dexterity**
- The desired grasp pose `g` is initially "occluded".
- An external wall is used for the grasp.

Interesting ideas:
- the **reward function**: **simple** and stays identical during the full episode.
- RL action: **End-effector delta** movement
- the controller: so-called **"compliant Operational Space Controller (OSC)"**.
  - the RL agent decides a **cartesian EE target pose** at 2Hz.
  - the OSC calculates the necessary **forces and torques at the EE** (not at the joints!) to reduce any error in reaching that position, **at 100Hz**.
  - For that, OSC uses a PD controller, with intentionally **low `Kp` and `Kd`**, to make the controller **"compliant"**, meaning it's softer or more flexible in how it moves.
  - I am still not fully comfortable with this term, but the **"compliance in OS"** is claimed to be beneficial for safety and control: It should **prevent the robot from applying too much force**.
  - > "If we use a controller that is compliant in the **joint** configuration space instead, we will **not have direct control over the maximum force** the end-effector might have on the object and the bin."
  - This allows the robot to **respond flexibly** to contact with objects, similar to how [impedance control](https://frankaemika.github.io/libfranka/cartesian_impedance_control_8cpp-example.html) modulates the interaction forces.
  - Finally, the desired force and torque of the EE are converted into **desired joint torques**. And sent to the robot controller.
- Automatic Domain Randomization (ADR): to "gradually expand the **range of randomization** automatically according to its performance".
- The section on the **Sim2Real gap** regarding the low-level controller
- The **failure cases** section

They use:
- [Robosuite](https://robosuite.ai/) as simulation environment, which is based on MuJoCo
- Azure Kinect as camera for object pose estimation
- a ROS interface made by their lab [FrankaPy](https://iamlab-cmu.github.io/frankapy/)
- SAC implemented with [rlkit](https://github.com/rail-berkeley/rlkit)
- Hindsight Experience Replay can be used there because the task is "conditioned" on target g. So even if `g` is not reached, a fake `g'` can be used during training loop to provide examples of a successful episodes

</details>

---

**`"Sim-to-Real Transfer of Robotic Control with Dynamics Randomization"`**

- **[** `2018` **]**
  **[[:memo:](https://arxiv.org/pdf/1710.06537.pdf)]**
  **[[🎞️](https://www.youtube.com/watch?v=XUW0cnvqbwM)]**
- **[** _`domain randomization`, `pushing task`, `privileged learninig`, `RNN`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2018_peng_1.png) | 
|:--:| 
| *only with information required to **infer the `dynamics`** are passed to the `LSTM` unit. Since the current `state` (`st`) is of particular importance for determining the appropriate `action` for the current timestep, a copy is also provided as input to the __feedforward__ branch [source](https://arxiv.org/pdf/1710.06537.pdf)* |

| ![](media/2018_peng_2.png) | 
|:--:| 
| *randomized `dynamics` parameters [source](https://arxiv.org/pdf/1710.06537.pdf)* |

| ![](media/2018_sudharsan13296_1.png) | 
|:--:| 
| *Hindsight Experience Replay (`HER`): instead of considering the **trajectory as a failure** wrt. the current `goal`, it can be seen as a **success** for a different `goal` - which is valuable to learn [source](https://github.com/sudharsan13296/Hands-On-Reinforcement-Learning-With-Python)* |

| ![](media/2018_peng_1.gif) | 
|:--:| 
| *`95` parameters are randomized. Some at each `action` step, some at each `episode` [source](https://www.youtube.com/watch?v=XUW0cnvqbwM)* |

| ![](media/2018_peng_2.gif) | 
|:--:| 
| *the system is robust to changes in contact `dynamics` (a packet of chips is attached to the bottom of the puck) [source](https://www.youtube.com/watch?v=XUW0cnvqbwM)* |

ideas
- **low fidelity** simulator and **poor calibration / `SysID`** can be enough to fully learn in `sim` and successfully deploy into `real`
- the `policy` is trained to adapt to a **wide-range of randomized environments** - hopefully it learns to adapt to yet another `env`: the **real `env`**.
- ingredients to **prevent over-fitting** and enable transfer
  - randomizing the `dynamics`
  - adding noise to `observations`
  - coping with the **latency of the controller**
- > "With **domain randomization**, discrepancies between the `source` and `target` domains are modeled as **variability in the `source` domain**"

task: **pushing**
- > "pushing, a form of _**non-prehensile**_ manipulation, is an effective strategy for positioning and orienting objects that are too large or heavy to be grasped"
- challenge: modeling the complex **contact dynamics** between surfaces

hardware
- tested on `7`-DOF [`Fetch Robotics`](https://fetchrobotics.com/) arm
- the location of the puck is tracked using the [`PhaseSpace mocap`](https://www.phasespace.com/x2e-motion-capture/) system.

`time` concepts in `MuJoCo` simulator
- one episode = `100` **_control_** (`action`) steps
- one _simulation_ timestep = `0.002s`
- one _control_ timestep = `action duration` = multiple _simulation_ steps
  - by default `20` _simulation_ steps = `dt0`
- _how to **model the `latency`** of the real robot?_
- **`action duration`** = `dt0 + eps`
  - `eps` is sampled at **each step** from `exp(λ)`, with `λ` sampled at **each episode** from `[125, 1000] s−1` 
- hence one **episode duration** = `100 * ~20 * 0.002` = `~4s`

`MDP` formulation
- `52`d `state` space:
  - **arm joints**: positions, velocities
  - **gripper**: position 
  - **puck**: position, orientation, linear and angular velocities
- `7`d `action` space:
  - > "`actions` from the `policy` specify **target joint angles** for a `position controller`. Target angles are specified as **relative offsets** from the current joint rotations"
  - _question: why caring about the `duration` between two `actions`? do they just wait for the move to finish between querying the agent for the next `action`?_ 
    - > "The timestep between `actions` specifies the amount of time an **`action` is applied** before the policy is queried again to sample a new `action`. This serves as a simple model of the **latency** exhibited by the physical controller."
    - _ok, I would understand if the `action` were a `torque`: the longer it is applied, the larger the changes. But with a target offset as `action`, the impact of the `action duration` does not make much sense to me, unless if the control is not blocking and can be interrupted/truncated._
- **sparse** `rewards`
  - > "designing a **dense** `reward` function can be challenging for more complex tasks, and may **bias the policy towards adopting less optimal behaviours**"
  - the sparse `rewards` let the agent develops **clever strategies**
    - > "these behaviours **emerged naturally** from the learning process using **only a _sparse binary_ `reward`**"

benefits of **`RNN`** (both in `Value` and `Policy`)
- in the absence of direct knowledge of the parameters, the **`dynamics` can be inferred from a history** of past `states` and `actions`
  - either: try to **explicitly identify** the parameters
  - or: `SysID` can be **implicitly embedded** into a `policy`
    - using a **recurrent** model `π(at|st, zt, g)`
    - the **internal memory `zt = z(ht)` acts as a summary of past `states` and `actions`**, thereby providing a mechanism with which the policy can use to **infer the dynamics** of the system.
- do not pass **all info** to the recurrent unit
  - the **goal location `g`** does not hold any information regarding the `dynamics` of the system: therefore processed only by the feedforward branch.
- trained with `Recurrent DPG` (`RDPG`)
- **stacking** is **simpler** and not much worse
  - **history of the `8` previously observed `states` and `actions`**
  - `FeedForw + Hist` achieves `87%` success in `sim` against `91%` for the `LSTM`

**_omniscient_ critic** for **training efficiency**
- similar to "privileged learninig" or [`Asymmetric Actor-Critic`](https://arxiv.org/abs/1710.06542)
- the **`dynamics` parameters `µ`** are passed to the `Value function` receives as input, but not to the `policy`
  - the `Value function` is used only during training, when `dynamics` parameters of the simulator are known
  - this allow the `Value function` to provide more meaningful feedback for improving the policy

**`HER` = Hindsight Experience Replay**
- idea: instead of considering a **trajectory as a failure** for the current `goal`, it can be **considered it a success for a different `goal`**
- **`HER` augments** the original training data recorded from rollouts of the policy **with additional data generated** from **replayed `goals`**, it requires **off-policy** learning

**randomization**
- _when?_
  - either at the **start of each episode**
    - > "a random set of **`dynamics` parameters** `µ` are sampled according to `ρµ` and held fixed for the duration of the episode"
  - or at **each step**
    - e.g. `action duration`, `observation noise`, `action` exploration noise
- `95` randomized parameters
  - **Mass** of each link in the robot’s body
  - logarithmically sampled:
  - Damping of each joint
  - Mass, friction, and damping of the puck
  - Height of the table
  - Gains for the position controller
  - Timestep between actions
  - Observation noise
    - independent Gaussian noise applied to each state feature
    - **standard deviation** = **`5%` of the running std** of each feature
- > "Parameters such as `mass`, `damping`, `friction`, and controller `gains` are **logarithmically sampled**, while other parameters are **uniformly sampled**"
- > "Gaussian **`action` exploration noise** is added at every step with a standard deviation of `0.01 rad`"

**system identification (`SysID`)**
- often require **meticulous calibration** of the simulation to closely conform to the physical system
- sometimes just impossible
- here: the policies are able to adapt to significant **calibration error**.

"universal" `policy`
- `policy` and `rewards` are conditioned on the target (`goal` noted `g`)

</details>

---

**`"Closing the sim-to-real Loop: Adapting simulation randomization with real world experience"`**

- **[** `2019` **]**
  **[[:memo:](https://arxiv.org/abs/1810.05687)]**
  **[[🎞️](https://www.youtube.com/watch?v=nilcJY5Kdt8)]**
  **[[🎞️](https://sites.google.com/view/simopt)]**
- **[** _`sim2real`, `domain randomization`, `simulation parameter distribution`, `franka emika`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2019_chebotar_1.gif) | 
|:--:| 
| *iterations: set params, learn a `pi` in `sim`, collect (few) samples using this `pi` in real world, compare `real` and `sim` samples, adjust params, etc. [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

| ![](media/2019_chebotar_2.gif) | 
|:--:| 
| *it is often disadvantageous to use __overly wide distributions__ of simulation parameters as they can include scenarios with __infeasible solutions__ [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

| ![](media/2019_chebotar_3.gif) | 
|:--:| 
| *updating the parameters of the `simulator parameters distribution` [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

| ![](media/2019_chebotar_4.gif) | 
|:--:| 
| *`swing-peg-in-hole` [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

| ![](media/2019_chebotar_5.gif) | 
|:--:| 
| *`cabinet drawer` [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

| ![](media/2019_chebotar_6.png) | 
|:--:| 
| *`simulation parameter distribution` is denoted by `Pφ`. To collect simulated `observation` samples, `simulation parameters` (`ξ`) are sampled from `Pφ` [source](https://arxiv.org/abs/1810.05687)* |

| ![](media/2019_chebotar_7.png) | 
|:--:| 
| *`simulation parameter distribution` [source](https://arxiv.org/abs/1810.05687)* |

| ![](media/2019_chebotar_8.png) | 
|:--:| 
| *`simulation parameter distribution` [source](https://arxiv.org/abs/1810.05687)* |

motivation
- `domain randomization` requires a **significant expertise** and **tedious manual fine-tuning** to design the `simulation parameter distribution`
- combine `system identification` and `dynamics randomization`

ideas
- **do not manually tune** the randomization of simulations
  - automate the learning of the **`simulation parameter distribution`**

how to update the parameters of the `simulation parameter distribution` (`Pφ`)?
- goal: bring `observations` induced by the `pi`(`Pφ`) closer to the `observations` of the real world
- minimization problem
  - `cost` = `Dist`(`obs-real`, `obs-sim`) 
- > "It should be noted that the **inputs** of the policy `pi`(`θ`, `pφ`) and observations used to compute `D`(`τ-obs`-`ξ` , `τ-obs-real`) are not required to be the same"
  - _but the initial `state` yes? is this open loop?_

real world setting:
- no need for **exact replication** of the **real world environment**
  - no need for `rewards`
  - partial `observations` is fine
- > "we use object tracking with [`DART`](http://www.roboticsproceedings.org/rss10/p30.pdf) to continuously **track the 3D positions** of the peg in the swing-peg-in-hole task"

training iterations
- the RL training can be **sped up** by **initializing the policy** with the weights from the previous `SimOpt` iteration
  - PPO iterations from `200` to `10` after the first `SimOpt` iteration

MDP
- `action`
  - joint velocity (`7-dof`)
  - gripper command
  - _`dt` = ?_
- `swing-peg-in-hole`
  - `obs`
    - joint configurations _???_
    - 3D position of the peg
    - :warning: _the `goal` position (hole) is not observed -> the `policy` is specific to one setting_
  - `reward`
    - distance peg-hole
    - angle alignment with the hole
    - a binary reward for solving the task
  - params
    - peg size
    - rope properties
    - size of the peg box
- `cabinet drawer`
  - `obs` (10 dim)
    - joint angles
    - 3D position of the cabinet drawer handle
  - `reward`
    - distance penalty between the handle and end-effector positions
    - the angle alignment of the end-effector and the drawer handle
    - the opening distance of the drawer
    - an indicator function ensuring that both robot fingers are on the handle
  - params
    - `position.x` of the cabinet

simulator
- > [`FleX`](https://developer.nvidia.com/flex) is a particle based simulation technique for real-time visual effects.
  - _predecessor of `IsaacSim`?_

related works
- > [`EPOpt`](https://arxiv.org/pdf/1610.01283.pdf) ([`video`](https://www.youtube.com/watch?v=w1YJ9vwaoto)) (**ensemble** optimization approach) optimizes a **risk-sensitive objective** to obtain robust policies, whereas we optimize the **average performance** which is a **risk-neutral** objective

</details>

---

**`"Data-efficient Domain Randomizationwith Bayesian Optimization"`**

- **[** `2021` **]**
  **[[:memo:](https://arxiv.org/pdf/2003.02471.pdf)]**
  **[[🎞️](https://www.youtube.com/watch?v=V1KZmIcMLt0)]**
  **[[️:octocat:](https://github.com/famura/SimuRLacra)]**
- **[** _`ball-in-a-cup`, `domain randomization`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2021_muratore_1.png) | 
|:--:| 
| *a `dynamic parameter` is introduced, it influences the `initial state` and the `transition` function. The expectation __over all domain parameters__ is to be maximized -> the `policy` should be __robust to shifts__ in the __`domain parameter distribution`__ [source](https://www.youtube.com/watch?v=V1KZmIcMLt0)* |

| ![](media/2021_muratore_1.gif) | 
|:--:| 
| *top=training: same __seed__ and __architecture__, the left __exploits the simulator__,  - not transferring, the right uses `BayRn` and is more conservative and robust [source](https://www.youtube.com/watch?v=V1KZmIcMLt0)* |

agents can exploit the simulator / the physics engine
- overfitting to features which do not occur in the real world

how is the `domain parameter distribution` updated?
- based on sparse real-world data
- using bayesian tools (Gaussian Process)

this is not `system identifaction`
- we do not care if the **parameters match the reality**
- instead, we care about the performance in real world

[`SimuRLacra`](https://github.com/famura/SimuRLacra)
- a Python/C++ framework for **RL from randomized physics simulations**

</details>

---

**`"Cyclic Policy Distillation: Sample-Efficient Sim-to-Real Reinforcement Learning with Domain Randomization"`**

- **[** `2022` **]**
  **[[:memo:](https://arxiv.org/abs/2207.14561)]**
- **[** _`domain randomization`, `sim2real`, `policy distillation`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2022_kadokawa_5.png) | 
|:--:| 
| *main idea [source](https://arxiv.org/abs/2207.14561)* |

| ![](media/2022_kadokawa_4.png) | 
|:--:| 
| *cyclic transitions [source](https://arxiv.org/abs/2207.14561)* |

| ![](media/2022_kadokawa_6.png) | 
|:--:| 
| *algorithm: the `value function` `Q(n)` of `env.n` is transferred to the __neighbouring environment__. Could a `policy` be transferred instead (making `on-policy` possible)? [source](https://arxiv.org/abs/2207.14561)* |

| ![](media/2022_kadokawa_1.png) | 
|:--:| 
| *real-world task [source](https://arxiv.org/abs/2207.14561)* |

| ![](media/2022_kadokawa_2.png) | 
|:--:| 
| *`observation` space [source](https://arxiv.org/abs/2207.14561)* |

| ![](media/2022_kadokawa_3.png) | 
|:--:| 
| *parameters randomized to help domain transfer [source](https://arxiv.org/abs/2207.14561)* |

motivation
- `sim-to-real` (for the `dynamics`, not the `observation`)
- efficient
- stable

idea
- **subdomains** (with varying parameters, e.g. `gravity`, `friction` ...) are **not selected _randomly_**
  - otherwise, the gap of subdomains in distillation can be large, making the learning results **unstable** and sample-inefficient
  - the subdomains with **adjacent index numbers** should be set up so that their **randomization ranges** are **similar**
- wait for all `sub.pi` to converge before distilling into a `global.pi`

"Cyclic Policy Distillation" (`CPD`)
- **"Policy Distillation"**
  - **transfer learned knowledge** between local policies in **divided subdomains**
    - the **dividing number `N`** controls the tradeoff between the **learning stability** and the **sample efficiency**
  - final integration of all local policies into a single **global policy**
    - learn to predict the same `action` distribution given an `observation`
    - `loss` = sum of `KL-div` between [`pi-g(s, a)`] and [`pi-n(s, a)`]
- "Cyclic"
  - transfer knowledge between **_adjacent_ subdomains**

monotonic improvement (`MI`) to update `pi-N`
- `loss` = `loss-RL` + `loss-MI`
  - `loss-MI` = KL-divergence between [`pi-N`] and [a weighted combination of `pi-N` and `pi-N'`]
- weight = `m`
  - `m` = `0` means neighboring local policies are **not exploited** for learning
  - `m` = 1 means neighboring local policies are **copied**

real-world experiment
- no need to model the full robot: **only** the **robot’s fingertip** is simulated
  - because, in the real-world environment, the robot’s kinematics do not affect the task achievement

- parameters
  - `ball-radius`, `gravity`, `friction-coefficient`, `ball-mass`
  - balls move in strange directions when deformed by the robot’s physical interaction

limits
- what simulator??
- `actor critic` methods: `SAC` but not `PPP`
- same task: all domains share the same reward function

</details>

---
---

# :chopsticks: tasks examples

**`"cartpole"`**

- **[[:memo:](https://www.gymlibrary.dev/environments/classic_control/cart_pole/)]**
  **[[:octocat:](https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py#enroll-beta)]**
- **[** _`classic control`, `gym`, `task example`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2018_inverted_pendulum_1.gif) | 
|:--:| 
| *with `PID` [source](https://www.youtube.com/watch?v=B4E9tGmONn0)* |

| ![](media/2018_inverted_pendulum_2.gif) | 
|:--:| 
| *with `MPC` or `LQR` controller? [source](https://blog.arduino.cc/2018/05/23/zipy-is-a-homebrew-inverted-pendulum/)* |

[`gym.CartPoleEnv`](https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py)
- the `dynamics` is modelled following [this paper](https://coneural.org/florian/papers/05_cart_pole.pdf)
  - the `action` represents a `force` and is converted to `acceleration`s
  - the `acceleration`s are used in integrators to derive `speed` and `position`

| Num | Action                 |
|-----|------------------------|
| 0   | Push cart to the left  |
| 1   | Push cart to the right |

```python
self.force_mag = 10  # N      
      
force = self.force_mag if action == 1 else -self.force_mag
costheta = math.cos(theta)
sintheta = math.sin(theta)

# For the interested reader:
# https://coneural.org/florian/papers/05_cart_pole.pdf
temp = (force + self.polemass_length * theta_dot**2 * sintheta) / self.total_mass
thetaacc = (self.gravity * sintheta - costheta * temp) / (
    self.length * (4.0 / 3.0 - self.masspole * costheta**2 / self.total_mass)
)
xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

if self.kinematics_integrator == "euler":
    x = x + self.tau * x_dot
    x_dot = x_dot + self.tau * xacc
    theta = theta + self.tau * theta_dot
    theta_dot = theta_dot + self.tau * thetaacc
else:  # semi-implicit euler
    x_dot = x_dot + self.tau * xacc
    x = x + self.tau * x_dot
    theta_dot = theta_dot + self.tau * thetaacc
    theta = theta + self.tau * theta_dot

self.state = (x, x_dot, theta, theta_dot)
```

_what `delta-t`?_
- `20 ms` in [`gym`](https://github.com/openai/gym/blob/dcd185843a62953e27c2d54dc8c2d647d604b635/gym/envs/classic_control/cartpole.py#L97)

</details>

---

**`"Reinforcement Learning of Motor Skills using Policy Search and Human Corrective Advice"`**

- **[** `2018` **]**
  **[[:memo:](https://www.ias.informatik.tu-darmstadt.de/uploads/Alumni/JensKober/IJRR__Revision_.pdf)]**
  **[[🎞️](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)]**
- **[** _`ball-in-a-cup`, `task example`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2018_celemin_1.gif) | 
|:--:| 
| *description [source](https://www.youtube.com/watch?v=OXzR_5TJ7Xk)* |

</details>

---
---

# :point_up: not (classical) RL

---

**`"π0-fast"`**

- **[** `2025` **]**
  **[[:memo:](https://arxiv.org/abs/2501.09747)]**
  **[[🎞️](https://www.physicalintelligence.company/research/fast)]**
  **[[🎞️](https://huggingface.co/blog/pi0)]**
  **[[:octocat:](https://github.com/Physical-Intelligence/openpi)]**

- **[** _`not RL`, `action tokenizer`, `vision-language-action models`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                                                           ![](media/2025_pertsch_1.png)                                                                                                           | 
|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The introduced FAST tokenizer maps any sequence of **robot actions** (continuous) into a **sequence of dense, discrete action tokens** for training autoregressive VLA models. [source](https://www.physicalintelligence.company/research/fast)* |

Motivation:
- Most foundation models use the **Transformer** architecture, a **sequence** model that operates on **discrete tokens**.
- Standard **binning-based discretization** techniques (e.g. per-dimension, per-timestep binning) perform poorly when learning **dexterous skills** from **high-frequency** (up to 50 Hz) robot data.
  - "**Correlations between time steps** are a major challenge for naive tokenization strategies."
  - "Low token prediction loss can often be achieved with mappings as trivial as simply **copying the most recent action token**, leaving models in poor local optima."
- **Diffusion** or **flow matching** tends to perform much better. See the `π0 VLA` below.
  - But **diffusion takes much longer to train**.
- Question: how to **represent continuous action signal**?

Main idea:
- > "Robot **action signals** need to be **compressed before training**, to **reduce correlation between consecutive tokens**."

**`FAST`:** Frequency-space **Action Sequence Tokenization**.
- An **action tokenizer**.
- Application: it enables training of **autoregressive VLA policies** via simple **next token prediction**.
- Ingredients:
  - **Discrete cosine transform** (`DCT`): a technique commonly used for **signal compression**, for instance in JPEG or MP3 codecs.
    - Because robotic actions are **continuous**.
  - **Byte pair encoding** (`BPE`): a **compression algorithm** often used for training large language models.
    - To compress robot action signals before training, in order to **reduce correlation between consecutive tokens**.
- Output: action chunks, where dense action tokens, typically 30 to 60 per chunk.
- How to decode?
  - "Since all operations are invertible, actions can be reconstructed efficiently and losslessly from tokens."
- What hyperparameters?
  - The **scaling coefficient** applied before rounding.
  - The **BPE vocabulary size**.

**`FAST+`**: a universal robot action tokenizer, based on FAST, **trained** on 1M real robot action trajectories. 
- A good off-the-shelf tokenizer for **training autoregressive VLA models**.
- What is the **training objective**?
  - The `BPE` algorithm: replacing the **highest-frequency pair of bytes** with a new byte that was not contained in the initial dataset. Until the **vocabulary size** is reached. 
- Training on **[DROID dataset](https://droid-dataset.github.io/)**.
  - A dataset containing diverse robot manipulation tasks that was collected over the span of two years by a large consortium of robotics researchers from around the world.

`π0 VLA + FAST tokeniser`:
- It manages to train a **generalist policy** on the **full dataset** that can follow language commands in **zero shot** in new environments.
- > "This gives a glimpse into a future where **we can download and directly use generalist robot policies**, just like we use language models today."

Limitations:
- The **inference** speed: the **autoregressive decoding** of `π0-FAST` is significantly slower than the **decoding via flow matching** used by `π0`.

</details>

---

**`"π0: A Vision-Language-Action Flow Model for General Robot Control"`**

- **[** `2024` **]**
  **[[:memo:](https://www.physicalintelligence.company/download/pi0.pdf)]**
  **[[🎞️](https://www.youtube.com/watch?v=https://www.physicalintelligence.company/blog/pi0)]**

- **[** _`not RL`, `foundation model`, `dexterous tasks`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                                            ![](media/2024_black_3.png)                                                                                             | 
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *Once the backbone is trained, it can be used either be **used directly** or **fine-tuned** using high-quality data. As for **base / instruct** LLMs. [source](https://www.physicalintelligence.company/blog/pi0)* |

|                                                                                        ![](media/2024_black_1.png)                                                                                         | 
|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The `π_0` model outputs a **sequence ("chunk") of low-level actions** (`H=50`). Only a fraction is used, and applied in open-loop: 18 or 25. [source](https://www.physicalintelligence.company/blog/pi0)* |

|                                                                                                              ![](media/2024_black_2.png)                                                                                                              | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *`len(q) = len(a) = 18`. For robots with **lower `q` and `a` spaces**, **zero-padding** is applied. For robots with fewer than three images, the **missing image slots are masked out**. [source](https://www.physicalintelligence.company/blog/pi0)* |

|                                                                                                                                    ![](media/2023_brohan_1.png)                                                                                                                                     | 
|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *One question for **vision-language-action** (VLA) models is the **format of the generated action**. In `RT2`, robot actions are **text strings**, i.e. **discrete** (figure above), while `π_0` generates **continuous action distributions**. [source](https://robotics-transformer2.github.io/)* |

In short:
- > "Our mission at [Physical Intelligence](https://www.physicalintelligence.company/) is to develop **foundation models** that can **control any robot** to perform any task."
- > "To our knowledge, our work demonstrates the **longest dexterous tasks** in the **end-to-end robot learning** literature." 

Some terms:
- **Generalist** robot policy
  - I.e., robot **foundation** model.
  - > "Just as a person can learn a new skill quickly by drawing on a lifetime's worth of experience, such a **generalist robot policy** could be **specialized to new tasks** with only **modest amounts of data**." 
- **Vision-language model** (VLM) backbone
  - A **multimodal** language model.
  - Based on the [PaliGemma vision-language model](https://huggingface.co/blog/paligemma)
    - Comparatively **small size** (which is useful for **real-time control**).
    - **3.3b** = 3b VLM + 0.3b action expert.
  - > "By basing our model on a VLM, we inherit the general knowledge, **semantic reasoning**, and problem-solving abilities of language- and vision-language models."
  - > "We then further train our model to incorporate **robot actions**, turning it into a **vision-language-action (VLA) model**."
- **Cross embodiment** training
  - Data from many **robot types** is combined into the same model.
- **Flow matching**
  - Question: how to **output robot action**?
  - Previous works: prior **autoregressive VLAs**, such as [RT-2](https://robotics-transformer2.github.io/)
    - Trained to output only **discrete** language tokens.
    - Problem: hard to handle **high-frequency action** chunks (up to 50 Hz) and **highly dexterous tasks**.
    - Solution: augment pre-trained VLMs with **continuous action outputs** via **flow matching**.
  - > "In order to make it possible to perform **highly dexterous** and intricate physical tasks, we use an **action chunking** architecture with **flow matching** (a variant of **diffusion**) to represent complex **continuous action** distributions."
  - I.e. it generates **continuous action distributions**. 
- **Action expert**
  - A separate set of weights is used for the robotics-specific (action and state) tokens.
  - > "This design is analogous to a **[mixture of experts](https://huggingface.co/blog/moe)** with **two** mixture elements, where the first element is used for **image and text inputs**, and the **second is used for robotics-specific inputs and outputs**. We refer to the **second set of weights** as the **_action expert_**."

`π_0` model
- input: 
  - 2 or 3 images per robot.
  - **Joint angles**.
  - A sequence of language tokens (the prompt / instruction).
    - For some temporally extended tasks (e.g. "bus the table"), a **high-level semantic policy** outputs **intermediate language commands**: more immediate subtasks (such as "pick up the napkin" or "throw the napkin into the trash").
- output:
  - "low-level motor commands"
    - **_what exactly? speed, acceleration, torque?_**
  - frequency: up to `50 Hz`.
  - > "Since the model **generates an entire `H=50`-step action chunk at once**, we can execute **up to** `H=50` actions before we need to run inference again."
    - Action chunks are executed in **open-loop**.
    - For the `20Hz` UR5e and Franka robots, we run inference every **0.8 seconds** (after executing 16 actions)
    - For all other robots, which run at `50Hz`, we run inference every **0.5 seconds** (after executing 25 actions).

Training recipe
- First pre-trained on a **very large and diverse** corpus
  - Objective: learn to **recover from mistakes** and handle **highly varied situations**.
  - (>90%) π-dataset (private): Over 10,000 hours of robot data: collected on 7 different robot configurations for **68 different tasks**. Mainly **dual-arm** robots.
  - (<10%) open-source datasets, such as [OXE](https://robotics-transformer-x.github.io/): 22 robots, with much **simpler tasks**.
- Then fine-tuned on more **narrow** and more carefully curated data
  - Objective: learn to **perform the task well**.
  - Examples: laundry folding, clearing a table, putting dishes in a microwave, stacking eggs into a carton, assembling a box, and bagging groceries. 
- > "Intuitively, training **only on high-quality data** does not teach the model how to **recover from mistakes**, since mistakes are **rarely seen** in such data. Training on **only lower-quality** pretraining data does not teach the model to **act efficiently and robustly**. Combining both provides the desired behavior: the model attempts insofar as possible to act in a manner similar to the high-quality data, but still has a repertoire of **recoveries and corrections** that it can deploy in the case of a mistake."

</details>

---

**`"Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware"`**

- **[** `2023` **]**
  **[[:memo:](https://arxiv.org/pdf/2304.13705)]**
  **[[🎞️](https://tonyzhaozh.github.io/aloha/)]**

- **[** _`end-to-end imitation learning`, `transformer`, `low cost hardware`, `action chunking`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                               ![](media/2023_finn_1.png)                                                                                | 
|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The ALOHA **teleoperation system** to collect demonstrations falls into the **"Puppeteering"** collection category. [source](https://cs224r.stanford.edu/slides/cs224r_imitation.pdf)* |

|                                                                                          ![](media/2023_finn_2.png)                                                                                           | 
|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The ALOHA system addresses the **"compounding errors"** problem by generating not just one action, but a **sequence (chunk) of actions**. [source](https://cs224r.stanford.edu/slides/cs224r_imitation.pdf)* |

|                                               ![](media/2023_zhao_1.png)                                                | 
|:-----------------------------------------------------------------------------------------------------------------------:| 
| ***Demonstrations** are recoreded with a **Bimanual Teleoperation** system. [source](https://arxiv.org/pdf/2304.13705)* |

|                   ![](media/2023_zhao_2.png)                    | 
|:---------------------------------------------------------------:| 
| *Examples of tasks. [source](https://arxiv.org/pdf/2304.13705)* |

|                                                                                                    ![](media/2023_zhao_4.png)                                                                                                    | 
|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *A **conditional VAE** (to model multi-modality) with **Transformer** (to model sequences) is used. Do not mix the "encoders/decoders": the CVAE and the Transformer have their own. [source](https://arxiv.org/pdf/2304.13705)* |

|                                                                              ![](media/2023_zhao_3.png)                                                                              | 
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *How to improve the **smoothness** of the policy? The **"temporal ensembling"** technique averages across the overlapping action chunks. [source](https://arxiv.org/pdf/2304.13705)* |

In short
- ALOHA : **A** **L**ow-cost **O**pen-source **Ha**rdware System for Bimanual Teleoperation.
- > "A low-cost system that performs **end-to-end imitation learning** directly from **real demonstrations**, collected with a **custom teleoperation** interface."

Challenges of IL and solutions (from [CS224R lecture](https://cs224r.stanford.edu/slides/cs224r_imitation.pdf)):
- 1: Supervised imitation learning struggles with **compounding errors**, particularly at **50 Hz**.
  - > "Where errors from previous timesteps **accumulate** and cause the robot to **drift off of its training distribution**, leading to **hard-to-recover states**."
  - Solution: **Action chunking**.
- 2: Human demonstrations perform tasks in **different ways**, leading to **multimodal data distribution**.
  - "We emphasize that **all human demonstrations are inherently stochastic**, even though a single person collects all the demonstrations."
  - Solution: Conditional variational auto-encoder (**VAE**) to **model multimodality**.
- > "Naive policy training achieves 0% success."

Details
- **End-to-end** learning for manipulation tasks:
  - Much simpler than **modeling** the whole environment
  - Directly maps **RGB images** to the actions
- A low-cost yet **dexterous teleoperation** system for data collection.
  - The user **teleoperates** by backdriving the **smaller WidowX** ("the leader"), whose **joints** are synchronized with the **larger ViperX** ("the follower").
  - 6 degrees of freedom -> **no redundancy**
  - No need for **inverse kinematics** (IK) module
  - **Four** logitech C922x webcams, each streaming 480×640 RGB image.
    - _todo: should these cameras be calibrated?_
  - Both the **teleoperation** and **data recording** happen at **50Hz**.
- **Action Chunking** with **Transformers** (ACT), which learns a **generative** model over **action sequences**.
  - The policy predicts the target joint positions for the **next k timesteps**, rather than just one step at a time.
  - This reduces the **effective horizon of tasks** (a `k`-fold reduction).
  - Trade-off: **Drift** vs **open-loop**
    - `k = 1` corresponds to no **action chunking**
    - `k = episode_length` corresponds to **fully open-loop control**
    - `k ~= 60` is selected, i.e. `horizon = 60 * 0.02 = 1.2 s = ~0.8 Hz`.
- **"Ensembling"** across **overlapping action chunks**.
  - To improve **smoothness**, the **policy is queried at every timestep!**

Policy:
- Observation:
  - The current **joint positions** of follower robots
  - The image feed from **4 cameras** (_todo: no past observations?_)
- Output:
  - **Sequence** of actions: **target joint absolute positions**.
  - > "Intuitively, ACT tries to imitate what a human operator **would do in the following time steps** given current observations."
  - **Absolute** joint positions give better results than **delta (relative) joint positions**. 

Conditional variational auto-encoder
- **`CVAE` decoder**: the **policy**
  - input (**conditions**):
    - the **current observations** (images + joint positions)
      - Images are processed with a **ResNet18**, and eventually flatten along the spatial dimension to obtain a sequence of `300×512`.
    - `z`: **"style variable"**.
      - > "At test time, we set z to be the mean of the prior distribution i.e. zero to deterministically decode"
  - output: the **action sequence**
- **`CVAE` encoder**: only serves to **train the `CVAE` decoder** and is discarded at test time
  - input: current **observation** (only its proprioceptive part - images are dropped for speed) and **action sequence**
  - output: predicts the **mean and variance** of the **style variable `z`**'s distribution, which is parameterized as a diagonal Gaussian 
- Training recipe:
  - Maximize the **log-likelihood** of **demonstration action chunks**
- Standard VAE objective:
  - a **reconstruction** loss, with L1 instead of the more common L2 loss, 
  - a term that **regularizes** the encoder to a Gaussian prior
- Encoder and decoder: **Transformer**
  - > "Transformers are designed for both synthesizing information across a **sequence** and **generating new sequences**."
- Inference time: `~0.01` seconds (i.e. `100 Hz`) on a single 11G RTX 2080 Ti GPU.

About **transformers**
- How are the **output tokens converted to target joint positions**? Does that imply that the actions are discrete?
- What is the `[CLS]` token? _I would need to look closer at BERT._

As the [CS224r](https://cs224r.stanford.edu/) [lecture on Imitation Learning](https://cs224r.stanford.edu/slides/cs224r_imitation.pdf) concludes: 
- Is **Imitation Learning** All You Need?
  - A simple and powerful framework for learning behavior.
- But:
  - Collecting expert demonstrations can be difficult or impossible in some scenarios
  - Learned behavior will **never be better than expert**
  - Does not provide a framework for **learning from experience**, indirect feedback
  - Can agents **learn autonomously**, from their own mistakes?

</details>

---

**`"Robotic Table Tennis: A Case Study into a High Speed Learning System"`**

- **[** `2023` **]**
  **[[:memo:](https://arxiv.org/abs/2309.03315)]**
  **[[🎞️](https://www.youtube.com/watch?v=uFcnWjB42I0)]**

- **[** _`high-speed robotics`, `latency`, `not RL`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                        ![](media/2023_dambrosio_1.gif)                                                                        | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *the agent should **return the ball** such that it **crossed the net** and **lands on the opposite side** of the table [source](https://www.youtube.com/watch?v=uFcnWjB42I0)* |

|                                                     ![](media/2023_dambrosio_2.gif)                                                      | 
|:----------------------------------------------------------------------------------------------------------------------------------------:| 
| *environment reset enables efficient evaluation and even fine-tuning - impressive [source](https://www.youtube.com/watch?v=uFcnWjB42I0)* |

|                            ![](media/2023_dambrosio_3.gif)                            | 
|:-------------------------------------------------------------------------------------:| 
| *perception based on 2 cameras [source](https://www.youtube.com/watch?v=uFcnWjB42I0)* |

|                                                                     ![](media/2023_dambrosio_4.gif)                                                                     | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *the agent is trained in a **simulator** based on `PyBullet` using an **evolutionary strategy (`ES`) algorithm** [source](https://www.youtube.com/watch?v=uFcnWjB42I0)* |

|                                                                   ![](media/2023_abeyruwan_1.gif)                                                                   | 
|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *referenced `catching` task trained with blackbox optimization [paper](https://arxiv.org/pdf/2306.08205.pdf) [source](https://www.youtube.com/watch?v=B9X94DcYybc)* |

|                                                                ![](media/2023_dambrosio_1.png)                                                                 | 
|:--------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *Policies are sensitive to `latency` and `physical` parameter values - less to `noise` and `throwing distribution` [source](https://arxiv.org/abs/2309.03315)* |


|                                 ![](media/2023_dambrosio_2.png)                                 | 
|:-----------------------------------------------------------------------------------------------:| 
| *software architecture - mixing `C++` and `python` [source](https://arxiv.org/abs/2309.03315)*  |


|                                 ![](media/2023_dambrosio_3.png)                                 | 
|:-----------------------------------------------------------------------------------------------:| 
| *`latencies` are modeled as a Gaussian distribution [source](https://arxiv.org/abs/2309.03315)* |


|                                                          ![](media/2023_dambrosio_4.png)                                                          | 
|:-------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *training a `policy` with `task`-space `actions` enables transfer to **different robot morphologies** [source](https://arxiv.org/abs/2309.03315)* |


|                                                                                            ![](media/2023_dambrosio_5.png)                                                                                            | 
|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *images are not converted to `RGB` but let in **raw `Bayer` pattern** for the detection. Together with the **`patch`-based training**, this increase the detection speed. [source](https://arxiv.org/abs/2309.03315)* |


|                                                      ![](media/2023_dambrosio_6.png)                                                       | 
|:------------------------------------------------------------------------------------------------------------------------------------------:| 
| *the `reward` function is **sparse** - only one `reward` is emitted at the end of the episode? [source](https://arxiv.org/abs/2309.03315)* |


|                    ![](media/2023_dambrosio_7.png)                    | 
|:---------------------------------------------------------------------:| 
| *post-debugging interface [source](https://arxiv.org/abs/2309.03315)* |

|                                                                                          ![](media/2021_gao_1.gif)                                                                                           | 
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The `restitution coefficients` of the ball, table, and paddle, and the `friction` of the paddle are measured using the method from `Gao et al. 2021` [source](https://www.youtube.com/watch?v=SNnqtGLmX4Y)* |


motivation
- > "Our hope is that this paper can help researchers who are **starting out in high-speed robotic learning** and serve as a discussion point for those already active in the area."

task(s)
- about table tennis:
  - "amateurs hit the ball at up to `9m/s`" -> amateur-speed ball crosses the table (`~3.6m`) in **`400 ms`**
  - **low latency** and **high precision** is required
- task-1: `hitting a thrown ball`
  - the agent should **return the ball** such that it **crossed the net** and **lands on the opposite side** of the table 
  - the arm's end effector is **3D-printed extension** attached to a **standard table tennis paddle**
- task-1 variant: `"damped hitting"`
  - the agent should **hit stronger**
  - using a lower `restitution coefficient` and a higher `linear damping` param 
- task-2: `catching a thrown ball`
  - a **lacrosse head** is used as the end effector 
  - > "This task has a much larger variance in `sim-to-real` transfer due to difficulty in **accurately modelling net & ball capture dynamics**."
  - related work (nice videos): https://sites.google.com/view/agile-catching
  - _not much info?_
  - modification needed:
    - **soft body modelling** of the lacrosse head net _(no further details?)_
    - **trajectory prediction** inputs for agents
    - handling **occlusions** when the ball is close to the net
- is `reinforcement learning` suited for this task?
  - required long-term planning: yes
  - rich agent-object interaction: no - but `atari pong` also did not

`action`-space: **`task`-space vs `joint`-space**
- **`task`-space** has several benefits:
  - is **compact**
  - is **interpretable**
  - provides a bounding cube for the end effector as a **safety** mechanism
  - aligns the robot `action` and the `observation` spaces with ball `observations`
  - possible transfer to **different robot morphologies**
  - > "`task`-space control usually shows significant improvements in learning of `locomotion and manipulation` tasks"
- _details about the IK solver? How fast?_
- results: `task`-space policies ...
  - **train faster**
  - much better than `joint`-space policies on **harder task** ("damped hitting")

`observation`-space
- ?what exactly received the policy?
- random noise is added to the ball observation. Domain randomization is also supported for many physical parameters.

`reward` function
- sparse and simple
- no action-continuity penalty is applied - _how do they get smooth behaviours?_

software architectures and choices
- modularity
  - > "hardware components (`cameras + vision` stack, `robot`, `ball thrower`) are controlled through `C++` and communicate `state` to the environment through a **custom message passing system** called `Fluxworks` (which utilizes highly optimized `Protobuffer` communication)"
  - _no ref to `Fluxworks`?_
- multi-language
  - > "this system adopts a **hybrid approach** where latency sensitive processes like control and perception are implemented in `C++` while others are partitioned into several `Python` binaries"
  - a **pure-`C++` thread** is started to handle **low-level control** for each of the `Festo` and `ABB` robots
    - communication with these threads is done asynchronously via **circular `C++` buffers**
    - the **circular buffers** are accessed from `Python` via **`Pybind11`-wrapped `C++` function calls**
- `CI/CD`
  - > "If the **system suddenly starts performing worse**, is it a _vision problem_, a _hardware failure_, or just a _bad training run_?"
  - > "many issues can only be **reproduced** when the **whole system** is running at full speed."
  - nightly tests are performed

safety layer 
- policy commands are **filtered** through a **safety simulator** before being sent to the robot
- The simulator converts a **velocity `action`** generated by the control policy to a `position` and `velocity` command required by `EGM` at each timestep
  - > "**Collisions** in the simulator generate a repulsive force that pushes the robot away, resulting in a **valid, safe command** for the real robot."

cams
- **2** hardwired synchronized `Ximea MQ013CG-ON` cameras
  - not more?
  - > "Adding more cameras to the current setup could produce still **more accurate position estimations**, but there start to be **bandwidth limitations** on a single machine and it may require remote vision devices (**increasing latency and system complexity**) or switching away from **USB3**."
- location
  - the cameras are mounted roughly `2m` above the play area on each side of the table
  - **extrinsic calibrations** performed with `cv2` using **`AprilTags` placed on the table**
- **`"fisheye"` lenses**
  - to capture more of the environment
  - > "the fisheye lens distortion introduces challenges in calibration and additional uncertainty in triangulation"
- `FPS` vs `latency`
  - **`125 FPS`** at a resolution of `1280x1024`
  - extremely low latency of `388 µs`
- **raw `Bayer`** images
  - some explanation on [RAW images](https://cs.brown.edu/courses/csci1290/labs/lab_raw/index.html)
  - the camera uses a **global shutter** with a **short (`4ms`) exposure time** and only returns the raw, **unprocessed `Bayer` pattern**
- efficiency
  - > "By skipping `Bayer` to `RGB` conversion, `1 ms` (or 15% of the time between images) of **conversion induced latency** per camera is avoided"
  - each video stream is processed independently

perception subsystem
- **detection network**
  - `CenterNet` like
    - five `spatial` convolutional layers
    - two **(buffered) `temporal` convolutions** to **capture motion features**
      - this `temporal` layer creates a **buffer to store the input feature** for the **next timestep**
  - intput
    - the **raw `Bayer` pattern** image - _not RGB!_
  - output: at each pixel ...
    - a likelihood of the ball center (trained with **binary cross-entropy** loss)
    - a `2D` local offset to accommodate **sub-pixel resolution**
    - a `2D` estimate of the **ball velocity** in pixels
  - training
    - uses **local patch** (`64x64`) to match the **receptive field** of the architecture
  - > "GPU data buffering (?), raw `Bayer` pattern detection, and patch based training substantially increase the performance of high frequency perception"
- sensor fusion: **triangulation**
  - the **direct linear transform (`DLT`)** method for binocular stereo vision estimates a **`3D` position** from these **image locations** in the **table's coordinate frame**
- post-processing
  - **recursive Kalman filter** to refine the estimated ball state before its `3D` position
  - The system uses the Savitzky-Golay `FIR` filter for **`observation` smoothing**


robot: `2+6` = **`8`-DOF system**
- `2`-DOF **linear actuator**: `Festo`
  - controlled through a `Modbus` interface at approximately `125Hz`
  - up to `2 m/s` in both axes
- `6`-DOF arm: `ABB`
  - **industrial robot** (not co-bot)
    - > "one major limitation of working with **off-the-shelf industrial systems** is that they may **contain proprietary, "closed-box software**"
  - joints rotate up to **`420` or `600` degrees/s**
    - for comparison: joints of the 7-dof franka emika fr3 have joint speed limits in [150, 300] degrees/s
- control
  - `ABB` **Externally Guided Motion (`EGM`)** interface
    - commands: `position` and `velocity` target per joint
    - response: joint feedback
    - frequency: `248Hz`
  - a lot of **parameter identification** has been performed on this **black-box controller**
- what happens:
  - observation -> `policy` -> `action` = joint-speeds
    - simulator: joint-speeds are direclty applied
    - real-robot:
      - joint-speeds -> `safety layer` -> target `position` and `speed` per joint
      - these targets are passed to the `ABB` `EGM`
- automated real world **environment `resets`**
  - using **blowing air** to return them to the hopper
  - impressive and useful for the **ablation study** where **`22.5k+` balls thrown** are performed

`gym` API also for the **real hardware** setup
- enabling fine-tuning on the real robot and efficient evaluation
- init state
  - a start state with controllers based on standard `S`-curve trajectory planning at the end of the episode or just after a paddle hit.
  - using `ABB` controller

evaluation
- **zero-shot `sim-to-real` transfer**
- maximum episode return = `2.0`
  - `+1`: making **contact with the ball**
  - `+1`: **landing on the opposing side**
  - a single evaluation is the average return over `50` episodes

policy
- trained for **`zero-shot` transfer**
- input:
  - > "**history** of the past **eight robot joint** and **ball states**"
  - _what len of history?_
  - _what exactly? are values normalized?_
- output
  - `joint`-space:
    - **`speed`** for each joint
  - `task`-space:
    - **`position`** in `3` dimensions
    - the **surface normal** of the paddle in `2` dimensions (`roll` and `yaw`)
- frequency: `100Hz`
- inference runs **on `CPU`**
  - Most policies are **compact**, represented as a **three layer**, 1D, fully convolutional gated dilated CNN with `≈1k` parameters
  - > The standard robot policies are **so small** that the time to transfer the input data from **`CPU` to accelerator** exceeds any savings in running inference on the accelerator"
- approaches for **agent-learning**
  - **Blackbox Gradient Sensing (`BGS`)**
    - **evolutionary strategy**
    - > "despite **poor sample efficiency**, `ES` methods are simple to implement, scalable, and robust optimizers that can even **fine-tune real world** performance"
  - other mentioned approaches - _no result reported :(_
  - `PPO` and `SAC` (reinforcement learning)
  - `GoalsEye` (behavior cloning)

simulator
- **`PyBullet`** is the **physics engine**
- **`gym` API** for environment interface
- compatible with **agent learning frameworks**, for example, `TF-Agents`, `ACME`, `Stable-Baselines3`
- _not released?_

some lessons learnt
- 1- **Modeling the `latency`** is crucial
  - `latency` is a major source of the `sim-to-real` gap in robotics
  - how to **mitigate** the various **sources of `latency`**?
    - **store `XXX` + interpolate to generate `XXX` for a given timestep, with the desired `latency`**
    - `observation`:
      - the history of `observations` are stored and **linearly interpolated** to produce an **observation with a desired latency**
      - > "`observation` interpolation on the physical system as a **useful technique** for increasing the robustness of deployed policies to latency variation."
      - to address noise and jitter, a **bandpass filter** is applied to the `observation` buffer **before interpolation**
    - `action`:
      - the action `latency` is implemented by **storing the raw `actions`** produced by the policy in a buffer, and **linearly interpolating** the `action` sent to the robot to the **desired `latency`**.
- 2- physics modelling and physical parameters
  - Policies are **sensitive to physical parameters**, which can have complex interactions with each other
    - > "One challenge of a complex system with many interacting components is that **multiple errors can compensate for each other**"
  - parameters: 
    - `restitution coefficients` of the ball, table, and paddle
    - the `friction` of the paddle
  - identification methods
    - ref: [`Optimal Stroke Learning with Policy Gradient Approach for Robotic Table Tennis`](https://arxiv.org/pdf/2109.03100.pdf)
  - setting these params to the **"identified" values** is not as good as using results of grid search (**"tuned" values**)
    - > "We hypothesize this is because ball spin is not correctly modelled in the simulator and that the tuned values compensate for this for the particular ball distributions used in the real world"
    - **`ball spin` is not accurately modeled** - important to determined if the balled is returned correctly
- 3- policies are **robust to `observation` noise** provided it has **zero mean**
  - the **interpolation** of `observations` likely serves as a buffer against **low levels** of zero-mean noise
  - but an **offset** due e.g. to a mis-calibration would be critical
- 4- policies are **robust to variations in ball throwing distributions** provided **the _real world_ distribution is contained within the _training_ distribution**
- 5- `latency` vs `FPS` vs `accuracy`
  - > "For both `frame-rate` and `latency`, the performance stays consistent with the baseline until there is a heavy dropoff at `50 FPS` and `150 ms` respectively, at which point the robot likely **no longer has sufficient time to react** to the ball and **swings too late**."
- 6- **automatic resets** of the real system is helpful
  - enables autonomous training and evaluation in the real world

</details>

---

**`"TossingBot: Learning to Throw Arbitrary Objects with Residual Physics"`**

- **[** `2019` **]**
  **[[:memo:](https://arxiv.org/abs/1903.11239)]**
  **[[🎞️](https://www.youtube.com/watch?v=f5Zn2Up2RjQ)]**
  **[[🎞️](https://www.youtube.com/watch?v=-O-E1nFm6-A)]**

- **[** _`not RL`, `1-step`, `policy distillation`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2019_zeng_1.gif) | 
|:--:| 
| *description [source](https://www.youtube.com/watch?v=f5Zn2Up2RjQ)* |

**`1`-step supervised** learning where the "ground truth" is the **result of the trial**

authors of the [`Transporter Networks: Rearranging the Visual World for Robotic Manipulation`](https://transporternets.github.io/)

</details>

---

**`"PaLM-E: An Embodied Multimodal Language Model"`**

- **[** `2023` **]**
  **[[:memo:](https://palm-e.github.io/assets/palm-e.pdf)]**
  **[[🎞️](https://palm-e.github.io/)]**
- **[** _`not RL`, `LLM`_ **]**

<details>
  <summary>Click to expand</summary>

> Given `<img>`
> Q: How to grasp the green object?
> A: First grasp the orange object and place it on the table, then grasp the green object.

The low-level policies are also obtained with [[`Deep Visual Reasoning: Learning to Predict Action Sequences for Task and Motion Planning from an Initial Scene Image`](https://arxiv.org/abs/2006.05398)][[🎞](https://www.youtube.com/watch?v=3Nguz6sg_1M)]

| ![](media/2023_driess_1.gif) | 
|:--:| 
| *description [source](https://palm-e.github.io/)* |

</details>

---

**`"High Acceleration Reinforcement Learning for Real-World Juggling with Binary Rewards"`**

- **[[:memo:](https://arxiv.org/abs/2010.13483)]**
  **[[🎞️](https://sites.google.com/view/jugglingbot)]**
- **[** _`no sim`, `1-step`, `expert init`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2020_ploeger_1.gif) | 
|:--:| 
| *the training on the **single real robot** takes up to `5 h`: **`20` optimization steps** are performed. For each, `25` rollouts are performed. [source](https://sites.google.com/view/jugglingbot)* |

| ![](media/2020_ploeger_1.png) | 
|:--:| 
| *a **normal distribution** (`mu`,`co-var`) from which parameters are sampled at the start of each rollout. These parameters define the `4` switching points, called **`via-points`**, between the `4` juggling movements. A **`PD` controller** computes the torques to follow the **spline** fitted on these `4` `via-points`. [source](https://arxiv.org/abs/2010.13483)* |

| ![](media/2020_ploeger_2.png) | 
|:--:| 
| *the **end-effector helps** to passively compensate for minor differences in ball trajectories. [source](https://arxiv.org/abs/2010.13483)* |

| ![](media/2020_ploeger_3.png) | 
|:--:| 
| *the **initialization** of the `via-points` is key (since the **`reward` is sparse**) and requires **hand-tuning** and **engineering**. [source](https://arxiv.org/abs/2010.13483)* |

| ![](media/2020_ploeger_4.png) | 
|:--:| 
| *__one-step `MDP`__: the current `mu`, `co-var` define a `Normal` distribution. Parameters are sampled to **generated the `4` via-points**. A **spline** is fitted. A **`PD` controller** tracks this trajectory. [source](https://arxiv.org/abs/2010.13483)* |

_RL?_
- **one-step** MDP
- **no interaction** with the environment
- **no `observation`!!**
- > "This framing is identical to a **bandit setting** with high-dimensional and continuous `actions`"
- _to me_:
  - a **control problem**, where
    - **initial parameters** are already good (manually tuned)
    - **parameters are optimized** with a complex search procedure

task
- `2`-ball juggling
- high accelerations required: of up to `8 g`
- [historic overview of robot juggling](https://www.youtube.com/watch?v=2ZfaADDlH4w)

hw
- [Barrett WAM manipulator](https://advanced.barrett.com/wam-arm-1)
  - `4`-DoF
  - **no simulation** possible:
    - > "For the **tendon driven** `Barrett WAM`, rigid-body-simulators also cannot model the **dominating cable dynamics** at high accelerations"
- [`optitrack`](https://optitrack.com/) ball tracker
  - not perfect though:
  - > "we had a hard time achieving good tracking performance during the experiments due to the wear and tear on the reflective tape and the frequent occlusions"
- juggling balls
  - `75 mm`
  - [Russian style](https://juggle.fandom.com/wiki/Russian_style_juggling_balls):
    - hollow plastic shell: **do not deform**
    - partially filled with `37.5 g` of sand: **do not bounce**
- end-effector designed to make the task **robust**
  - the **funnel-shaped** end-effector passively compensates for minor differences in ball trajectories

ideas
- **before** the episode
  - define a **normal distribution** from the current estimated `mu` and `co-var` (**multivariate**)
  - **sample parameters** to define **`4` "via-points"**
  - note: this is the only time in the episode that `actions` are taken (afterwards **open-loop tracking** control)
  - compute a **cubic spline** from them (interpolation)
- in **transient state** = **stroke-based** movement
  - initial **stroke movement** that quickly enters the limit cycle without dropping a ball
    - ???
    - **hand-tuned**
  - the second ball is released via a launching mechanism `3 m` above the floor (`1.8 m` on `fig.2`)
- in **steady state**:
  - keep tracking this spline (interpolated via-points) with a **`PD` controller** with **gravity compensation**
  - terminate on a ball fall
  - truncation after `10 s` juggle
- after the episode
  - estimate performance with **binary `reward`** (sparse):
    - `+1` each time one ball reaches `60 cm`
- after `20` episodes:
  - optimize the **parameters distribution** and repeat

_how many trials?_
- `500` rollouts:
  - `25` randomly sampled parameters executed per "episode"
  - `20` "episodes"
- `56 min` of **experience**
- the **learning** still takes **up to `5 h`**

open-loop
- `closed-loop` is difficult:
  - **noisy ball observation** and potential **outliers** might **cause instabilities**
- human can do `open-loop`:
  - > "well trained jugglers can juggle basic patterns **blindfolded**"

`via point` parameters used to define a **cubic spline**
- position `qi`
- velocity `q˙i`
  - enforced to be `0` (which reduces the dimensionality)
- duration `ti`

safety
- safer to **predict** a trajectory, rather than **torques**
  - **tight box constraints** in **joint space** are enforced for the desired trajectory
  - against **self-collisions** and hitting the **joint limits**

**`policy` (parameter distribution) initialization**
- **expert** initialization
- **engineering required!**
- initially the robot on average achieves **between `1` to `3` repeated catches**
- **`via-points` are manually initialized**
  - advantage: they are **interpretable**
  - > "tuning the **stroke-based movement** is the hardest part of the manual parameter tuning"

policy optimizer
- information-theoretic policy search approach
  - **_episodic Relative Entropy Policy Search_ (`eREPS`)**
- quite complex!
- > "We use `eREPS` instead of `EM-based` or `gradient-based` policy search as the **`KL` constraint** prevents premature convergence and **large, possibly unsafe, jumps** of the `policy` mean"

sparse rewards
- the **delay** between `action` and `reward`, i.e., **"credit assignment"**, is a challenge:
  - a **bad `action`** within the throwing will cause a drop, i.e. `reward = 0`, **seconds after the `action`**
- requires a **very good `policy` initialization!**

</details>

---

**`"Excavation Reinforcement Learning Using Geometric Representation"`**

- **[** `2022` **]**
  **[[:memo:](https://www.semanticscholar.org/paper/Excavation-Reinforcement-Learning-Using-Geometric-Lu-Zhu/f3bad843e86a37dd29e86ef5582eced26c82e028)]**
  **[[🎞️](https://drive.google.com/drive/folders/19n-V573He55i6WqnoINukvoaU1oqSUmQ)]**
- **[** _`1-step`, `franka emika`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2022_lu_1.gif) | 
|:--:| 
| *description [source](https://drive.google.com/drive/folders/1rx1mcAEe8eVNy8omeNM3R794tgZ2rDBf)* |

| ![](media/2022_lu_2.png) | 
|:--:| 
| *`action`: the excavation trajectory `T` is simplified to its **attacking pose**: (`x`, `y`, `α`) [source](https://drive.google.com/drive/folders/1rx1mcAEe8eVNy8omeNM3R794tgZ2rDBf)* |

| ![](media/2022_lu_3.png) | 
|:--:| 
| *`state`: feature extraction is done before the `RL`: predict the `normal`, `curvature`, and the `number of objects` in the digging tray [source](https://drive.google.com/drive/folders/1rx1mcAEe8eVNy8omeNM3R794tgZ2rDBf)* |

motivations:

- long-term goal
  - maximize the accumulative volume of the excavated rigid objects
  - plan N digging trajectories, not just one excavation
- learn a `280-dim` geometric representation for point clouds, which allows us to use a small policy network
  - based on PointNet++
  - in: point cloud
  - out: its normal, curvature, and the number of objects in the digging tray

excavation
- `attacking`
- `digging`
- `dragging`
- `closing`
- `lifting`
- we **simplify** the excavation trajectory `T` as its **attacking pose**: (`x`, `y`, `α`), which we also refer to as the **attacking pose**
  - **the other parameters (`dragging length`, `lifting h`, `closing angle`) are fixed**

real-world
- > "resistive force during rigid objects excavation can be large and only limited amount of force and torque can be applied by the Franka arm in real world"

</details>

---
---

# :books: theory & reviews

**`"Deep Reinforcement Learning for Robotics: A Survey of Real-World Successes"`**

- **[** `2024` **]**
  **[[:memo:](https://www.arxiv.org/abs/2408.03539)]**

- **[** _`review`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                                           ![](media/2024_tang_1.png)                                                                            | 
|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *The **tables** in the appendix list the references with the level of real-world success as colours. These are very insightful! [source](https://www.arxiv.org/abs/2408.03539)* |

The taxonomy spanning four axes:
- **Robot competencies** learned with DRL
  - Mobility: locomotion and navigation
  - Manipulation, i.e. moving or rearranging objects:
    - pick-and-place
    - contact-rich (assembly / articulated / deformable)
    - in-hand
    - non-prehensile (moving objects without grasping)
- Problem **formulation**
  - The action-space, observation space, and reward function of the POMDP
- **Solution** approach
  - Use of simulators / real-world data
  - Learning the world-dynamics
  - Use of expert data
  - Off/On policy
- Level of **real-world success**
  - Level 0: validated only in simulation
  - ...
  - Level 5: deployed on commercialized products

I think it would have been interesting to analyse the **action space** more in details:
- For instance: joint-**position/speed/torque** control? what frequency?

Most approaches use **Zero-shot Sim-to-Real**.
- It would have been interesting to compare the **simulators** and the **RL frameworks** as well.

About deformable objects: most about folding cloth - no soft-body manipulation:
- #134: [Sim-to-real reinforcement learning for deformable object manipulation](https://sites.google.com/view/sim-to-real-deformable)
- #135: [Learning to manipulate deformable objects without demonstrations](https://www.youtube.com/watch?v=7kxkJlPuLz4)
- #136: [Speedfolding: Learning efficient bimanual folding of garments](https://pantor.github.io/speedfolding/) and [this video](https://www.youtube.com/watch?v=UTMT2WAUlRw)  # by the author of [frankx](https://github.com/pantor/frankx)
- #137: [One policy to dress them all: Learning to dress people with diverse poses and garments](https://sites.google.com/view/one-policy-dress/)

</details>

---

**`"Deep Reinforcement Learning with Real-World Data"`**

- **[** `2022` **]**
  **[[🎞️](https://www.youtube.com/watch?v=0Kw-VTym9Pg)]**
- **[** _`offline RL`, `data-driven RL`, `conservative Q-learning`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2022_levine_1.png) | 
|:--:| 
| *do not try to copy the garbage, rather try to **understand the `dynamics`**, i.e. the **consequences of decisions** [source](https://www.youtube.com/watch?v=0Kw-VTym9Pg)* |

| ![](media/2022_levine_2.png) | 
|:--:| 
| *extract the **best parts** of **suboptimal demonstrations** [source](https://www.youtube.com/watch?v=0Kw-VTym9Pg)* |

| ![](media/2022_levine_3.png) | 
|:--:| 
| *how to know if this is good if __you do see it__ in the data? it can be __dangerous__, but also __just better [source](https://www.youtube.com/watch?v=0Kw-VTym9Pg)* |

| ![](media/2022_levine_4.gif) | 
|:--:| 
| *the big trained `PTR` can be used either for __fine-tuning__ or for initializing __new tasks__ [source](https://www.youtube.com/watch?v=0Kw-VTym9Pg)* |

additional resource
- [similar talk at GTC23](https://www.nvidia.com/en-us/on-demand/session/gtcspring23-s51826/)
- [`Imitation learning vs. offline reinforcement learning`](https://www.youtube.com/watch?v=sVPm7zOrBxM)
- [`Reinforcement Learning Pretraining for Reinforcement Learning Finetuning`](https://www.youtube.com/watch?v=zg6Tip6s_mA)
- [`Offline Reinforcement Learning: From Algorithms to Practical Challenges`](https://sites.google.com/view/offlinerltutorial-neurips2020/home)
- [Chelsea Finn's intervention in `Dexterity: Machine Learning for Robot Manipulation and Dexterity`](https://youtu.be/Vj50Z7az3TY?t=642)

**data-driven `RL`**
- as a way to use "large + cheap + garbage" data
- **do not to learn the `decision`**: _what to do?_
  - but rather **understand the `consequences`**: _how does the world work?_
  - understand the **world dynamics** from **suboptimal (w.r.t. our `task`) samples**
- then define a `task`
  - small supervised work: human-designed the `reward function`

`RL` vs `supervised-learning`
- about making **`decisions`** considering their consequences
  - not just `predictions` (learning `p(x)`) that ignore the **consequences**
- **active online** learning
  - require an interaction: `online` is costly
    - in a dialogue, you would have to talk to people in real-time

`offline RL` vs `IL`
- you do not want to **copy the garbage**
- rather **copy the best parts** of each suboptimal data, and **generalize**

issue with `offline RL`
- **counterfactual queries**
  - _"is it a good idea if I suddenly turn on the highway?"_, this **has not happened in the dataset** (out-of-distribution)
  - `online RL` could try
  - `offline RL` should, at the same time
    - **generalize** to get better than the data
    - **stay safe**
- `distributional shift`
  - apply naive `Q-learning` does not work
  - **over-estimation of `Q`**
    - predicted `Q-values` are huge
    - learnt `pi returns` are bad
  - it resembles **adversarial** example
    - ??
- sol: **`conservative Q-learning`**
  - detect where (which `action`) the `Q-value` is over-estimated compared to the current `Q-value`, and reduce it
  - a regularization similar to **adversarial** training process

applications: **few-shots**
- **fine-tuning** on task already present in the data
- **pre-training** (initialization) of down-stream tasks

[`PTR`: "Pre-Training for Robot"](https://sites.google.com/view/ptr-robotlearning)
- trained on [Bridge Data](https://sites.google.com/view/bridgedata)
- learn 1 `policy`, on all tasks (different domains)
  - conditioned with one-hot vector (task)
- **fine-tune** or **training** on a **new down-stream task**, while **preventing forgetting** (`batch-mixing`)

</details>

---

**`"Efficient Online Reinforcement Learning with Offline Data"`**

- **[** `2023` **]**
  **[[:octocat:️](https://github.com/ikostrikov/rlpd)]**
- **[** _`online RL with offline data`, `off-policy RL`, `symmetric sampling`_ **]**

<details>
  <summary>Click to expand</summary>

|                                                           ![](media/2023_ball_3.png)<br/>                                                           | 
|:---------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *key ingredients: `LayerNorm`, `large ensembles`, `symmetric sampling`. Env-specific ingredients: `multiple (2) critics` `entropy term` `3 vs 2-layer MLP` [source](https://arxiv.org/pdf/2302.02948.pdf)* |

|                                                                                                                                                                                                                                                                                   ![](media/2023_ball_2.png)<br/>                                                                                                                                                                                                                                                                                    | 
|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:| 
| *`LayerNorm` should mitigate **value divergence**. The `Q`-function (blue cone) is learnt from **online samples**. What happens when an `action`, **out of the `online` distribution**, is sampled from the **_offline_ replay buffer**? Its `Q` is **extrapolated** from the **function approximator** (`NN`), leading to **overestimation**. A solution is to apply **`LayerNorm`**, which suppresses **extreme value extrapolation**, whilst maintaining the freedom of an **unconstrained off-policy** method (`exploration` is still possible). [source](https://arxiv.org/pdf/2302.02948.pdf)* |

|                                                         ![](media/2023_ball_1.png)<br/>                                                         | 
|:-----------------------------------------------------------------------------------------------------------------------------------------------:| 
| *comparison of **regularization** methods: `ensemble` works better than `dropout` and `weight-decay`. [source](https://arxiv.org/pdf/2302.02948.pdf)* |

`RLPD` = Reinforcement Learning with **Prior Data**

related works
- **pre-training**
  - **offline RL**, followed by **online fine-tuning**
- explicit **constraints**
  - to handle issues with **distribution shift**
  - > "the online agent updates are explicitly constrained such that it exhibits behavior that **resembles the offline data**"
  - e.g. augments a policy gradient update with a weighted update that explicitly includes demonstration data

here
- > "we **do not** perform any **offline pre-training** but run **online RL _from scratch_** with offline data included in a replay buffer."
- > "we **do not** restrict the policy using a **behavior cloning term**"
- > "[no need for high quality offline data] our approach is, importantly, **agnostic to the quality** of the data"
- the environment cannot be **explored online**, therefore **distribution shift** should not be a problem

pre-collected data
- access to offline datasets
  - `D` is a collection of (`s`, `a`, `r`, `s′`) tuples generated from a particular `MDP`
  - _todo: how is the `reward` annotated?_
- **quality / quantity**
  - this general approach that is **agnostic**

algo
- `SAC`

key ingredients
- `symmetric sampling`
  - to incorporate _online_ and _offline_ data
  - for each batch:
    - `50%` of the data is sampled from the **online replay buffer**
    - `50%` from the **offline data buffer**
- **normalizing the `critic`** update
  - goal: prevent catastrophic **`value` over-extrapolation**
    - the `Q`-values of out-of-distribution (`OOD`) `action` (estimated via extrapolation) should **not be significantly greater** than those already seen in the data
  - `LayerNorm` as a **`value` extrapolation regularizer**
    - to mitigate catastrophic **overestimation** / **critic divergence**
    - it bounds the extrapolation of networks ...
    - ... but still allows exploration: it **does not explicitly constrain** the policy to remain close to the offline data
- using **large ensembles** to improve sample efficiency
  - goal: **efficiently** incorporate prior data
  - the incorporation of prior data is **implicit** through the use of **online `Bellman` backups** over **offline transitions**
  - goal: make these **`Bellman` backups** efficient
    - _pure offline_ and _constrained_ approaches have an **explicit** mechanism for that:
      - pre-training
      - auxiliary supervision term
  - here: **random `ensemble` distillation**
    - particularly efficient on **sparse `reward`** tasks.

**`env`-specific** ingredients
- > "many works in deep RL require per-environment hyperparameter tuning"
  - may **not be universally useful** (to be tried out each time)
  - > "may explain why **off-policy methods have not been competitive** thus far"
- **Clipped Double `Q`-Learning** (`CDQ`)
  - problem: the maximization objective of `Q`-learning leads to **value overestimation**
  - solution: taking a minimum of an **ensemble of two `Q`-functions** for computing **`TD`-backups**
  - issue: fitting target `Q`-values that are **`1 std.` below the actual target values** when updating the critics. This may not be universally useful as it can be **too conservative**
- `MaxEntRL` term in the objective
  - goal: maximize `reward` while **behaving as randomly as possible**
  - helps when `rewards` are **often sparse** and require **exploration**
- architecture: `2` or `3` layers in the `actor` and `critic`

</details>

---

**`"Robot Learning from Randomized Simulations : A Review"`**

- **[** `2022` **]**
  **[[:memo:](https://arxiv.org/pdf/2111.00956.pdf)]**
  **[[🎞️](https://www.ias.informatik.tu-darmstadt.de/Videos/Videos)]**

- **[** _`domain gap`, `domain parameter distribution`, `domain randomization`, `sim2sim`, `sim2real`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2022_muratore_1.png) | 
|:--:| 
| *problem formulation: a `domain parameter distribution` `p`(`ξ`) is introduced [source](https://arxiv.org/pdf/2111.00956.pdf)* |

motivations:
- **`domain randomization`** is a method for learning from randomized simulations by **adding perturbation**
  - of simulator parameters
  - state `observations`
  - or applied `actions`
- it can be seen as a **regularization method** that prevents the learner from **overfitting** to individual **simulation instances**
  - from the **Bayesian perspective**, we can interpret the `distribution over simulators` as a representation of **uncertainty**
- ideally, measure for the inter-domain transferability of controllers

example of parameters:
- bodies' inertia and geometry
- parameters of the **friction** and **contact** models
- possible **delays in the actuation**
- efficiency coefficients of motors
- levels of **sensor noise**
- visual properties (colors, illumination, position and orientation of a camera)

why a gap?
- some physical phenomena are omitted,  especially `friction`
- parameter estimation is inaccurate, which can quickly lead to unstable system dynamics
- the numerical integration is discrete in typical solvers

**physic engines**
- = `parameterized generative models`, which describe how multiple bodies or particles evolve over time by interacting with each other
  - the associated `physics parameters` can be estimated by `system identification`
    - which generally involves executing experiments on the physical platform and recording associated measurement
  - > "It is important to keep in mind that even if the `domain parameters` have been **identified very accurately**, simulators are nevertheless just **approximations of the real world** and are thus always imperfect"
  - strength: simulators are able to **model the control and kinematics** accurately
  - weakness: show deficits during **dynamic robot-object interactions**
- sources of deviations between physics engines
  - **`friction` models**
  - **`contact` models**
  - coordinate representations
  - numerical solvers
- examples
  - `ODE`
  - `DART`
  - `Bullet`
  - `MuJoCo`
  - `PhysX` (nvidia)
  - `FleX` (nvidia)
  - `Newton`
  - `SimBody`
  - `Vortex`
  - `Havok`
  - `Chrono`
  - `RaiSim`
  - `Brax`

randomization
- _which param?_
  - first identify the essential parameters
  - injecting **random latency** and *noise to the actuation** is another frequent modeling choice
  - including at least one **visually observable parameter** (e.g., an extent of a body) helps to **verify if the values are set as expected**
- _when?_
  - sample parameters at **each `episode`** _(not at each `step`!)_
- physical plausibility
  - `DR` can cause **numerical instabilities**
- good practice
  - test an `DR` on **at least two different `sim-to-real` tasks**
- three types of `DR`
  - **static**
    - a set of `domain parameters` is sampled at each episode
    - requires **hand-tuning** the `domain parameter distribution`
    - variation: randomizing dynamics using real-world data at runtime
      - incorporate the **simulator** as a prior for the **probabilistic model**, and subsequently use this information of the **`policy` updates** with `PILCO`
  - **adaptive**
    - a set of `domain parameters` is sampled at each episode
    - the `domain parameter distribution` is **updated during learning**
    - requires **data from the `target domain`**, typically the real robot, which is expensive
    - variation: `pi`(`obs`, `ξ`), the `policy` is **conditioned** on the **`state`** and the **`domain parameters`**
      - parameters are not assumed to be known, they have to be estimated
      - the **online system identification** module can enable an **adaption to sudden changes** in the environment
    - variation: `bilevel` optimization
      - `SimOpt`
      - `BayRL`
        - an `upper` level problem: find the `domain params` that lead to a `policy` with maximal **real-world `return`**
        - a `lower` level problem: find a `policy` in the **current** randomized `source` domain
  - **adversarial**
    - apply **adversarial disturbances** during the training process
    - too much disturbance can make the task unsolvable
    - variation (`zero-sum game`): the `model-player` agent hinders the `policy-player` agent from fulfilling its task by applying force disturbances at predefined locations

promising directions
- `real-to-sim-to-real` transfer
  - reduce workload
  - bring a meaningful initialization for `domain distribution parameters`
- pair the expressiveness of deep `NN` with **physically-grounded prior** knowledge
- hierarchical `RL`
  - tasks consist of (disconnected) segments
    - e.g. a robot needs to _turn the knob_ before it can _open a door_
  - sol: splitting the control into `high` and `low` level policies, similar to the **`options` framework**
  - > "Existing approaches typically realize this with **discrete switches** between the **low-level policies**, leading to undesirable abrupt changes in the behavior"

other concepts
- `curriculum learning`gap
  - > to increase the **sample efficiency** by **scheduling** the training process such that the agent first encounters **"easier" tasks** and gradually progresses to **"harder"** ones.
  - `curriculum learning` **does not aim** at finding solutions **robust to model uncertainty**
- `meta learning`
  - one can view `domain randomization` as a special form of `meta learning` where the robot's **task remains qualitatively unchanged** but the **environment varies**
  - but actually different
    - The goal of `meta learning` is to **find a suitable set of initial weights**, which when updated generalizes well to a new task
    - `domain randomization` on the other hand strives to directly **solve a single task**, generalizing over `domain` instances
- `transfer learning`
  - `DR`: case of `TL` where
    - ground truth information is only available in the `target domain`
    - `target domain` different from `source domain`
    - the task remains the same
- `knowledge distillation`
  - `KD` can be used to **compress the learned behavior** of one or more **teachers** into **a single student**
  - Based on samples generated by the **teachers**, the **student** is trained in a **supervised manner** to **imitate** them
- `system identification`
  - The goal is to find the set of **model parameters** which **fit the observed data best**
  - simple linear regression: it was quickly observed that the inferred parameters may be **physically implausible**

</details>

---

**`"Time limits in reinforcement learning"`**

- **[** `2022` **]**
  **[[:memo:](https://arxiv.org/pdf/1712.00378.pdf)]**
  **[[🎞️](https://www.youtube.com/watch?v=UUqidOSMKLE)]**
  **[[🎞️](https://fabiopardo.github.io/posters/time_limits_in_rl.pdf)]**
  **[[🎞️](https://sites.google.com/view/time-limits-in-rl)]**
- **[** _`Markov`, `time-awareness`, `truncation`, `termination`, `bootstrappig`, `episode`, `gymnasium`_ **]**

<details>
  <summary>Click to expand</summary>

| ![](media/2022_pardo_1.gif) | 
|:--:| 
| *The `infinite cube pusher` task [source](https://youtu.be/ckgVLgFi-sc?t=56)* |

| ![](media/2022_pardo_1.png) | 
|:--:| 
| *in __`episodic`__ task, add `remaining-time` to `obs` // in __`non-episodic`__ task, make them `episodic` to __increase exploration__, but make sure to __boostrap when `truncating`__ to let the agent know that __more `rewards` would actually be available__ thereafter [source](https://arxiv.org/abs/1712.00378)* |

| ![](media/2022_pardo_2.png) | 
|:--:| 
| *`gymnasium` approach: distinguish `MDP`-`termination` and external-`truncation` [source](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits)* |

ideas
- especially for `bootstrapping` methods 
- [`episodic` task]
  - respect Markov
  - add `remaining time` to `observation`
- [`time-unlimited` task]
  - make it `episodic` (`timeout`) **during training** to help exploration
  - but be careful: [`timeout` termination] `!=` [`env` (`MDP`) termination]
  - therefore: `bootstrap` at `states` where `termination` is due to **time limits**

note:
- `bootstrapping` = using **estimated values** of `V[s+1]` to update the **estimate** of the `V[s]`

case1: **time-_limited_** tasks
- i.e. "episodic"
  - the _maximum length_ of an episode is fixed
- possible issues:
  - **`state aliasing`**, i.e. unperceived remaining time
    - a time-**unaware** agent has to act in a `POMDP` where `states` (that only differ by their **remaining time**) appear identical
    - perception of **non-stationarity** of the `env`: the `terminations` due to **time limits** can only be **interpreted** as part of the **`env`'s stochasticity**
    - infeasibility of correct **`credit assignment`**: **conflicting updates** for estimating the *`value` of the same `state`* result in an inaccurate average
    - suboptimal policies and **instability**
    - "out-of-reach cells leaking" (when `bootstrapping` `values` from `states` that were **out-of-reach**): letting the agent **falsely believe** that more `rewards` were available after
- solution: **`time-awareness`** (`TA`)
  - add a notion of the `remaining time` to the `observation`, to avoid violation of the **`Markov` property**
  - learn to take **more risky** actions leading to higher expected returns as **approaching the time limit**

case2: **time-_unlimited_** tasks
- indefinite period, possibly infinite
  - no underlying `episodic` structure
  - e.g. `Hopper-v1` / `highway`
- `exploration` requires sufficiently **randomized initial `states`**
  - therefore, **"partial episodes"**
    - frequently `reset` the `env` to **diversify experiences**
    - introduce `truncations` to learn from **time-limited interactions**
  - but: the `time limits` are not part of the `env` and are **only used to facilitate learning**
- idea/solution: **differentiate** between the types of `terminations`
  - those from the `env` (e.g. `off-road`)
    - do not boostrap!
  - those due to `time limits` (`truncation` in `gymnasium`)
    - boostrap
    - `partial-episode bootstrapping` (`PEB`)
    - otherwise: **forgetting** that more `rewards` would actually be available thereafter
- note: `PEB` helps large `experience replay buffers`

[open question](https://youtu.be/UUqidOSMKLE?t=594) by Richard Sutton I guess:
- `partial-episode bootstrapping` makes `on-policy` methods `off-policy`
  - because the applied bootstrap (at `truncation`) uses estimates that **may not come from the current policy**
  - _how to deal with that?_

why do common **`time-unaware` agents** still often manage to perform relatively well?
- if **`time limits` are so long** that timeouts are hardly ever experienced
- if there are **clues in the `observations`** that are **correlated with time** (e.g. the `forward distance`)
- if it is **not likely** to observe the **same `states` at different remaining times**
- if the **`discount factor` is sufficiently small** to reduce the impact of the **confusion**

[`gymnasium` approach](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits)
- in old `gym`, **time limits** are incorrectly handled
  - > "The `done` signal received (in previous versions of OpenAI Gym < 0.26) from `env.step` indicated whether an **episode has ended**. However, this `done` signal did not distinguish whether the `episode` ended due to **`termination` or `truncation`**"
- new `gymnasium` distinguishes
  - `termination` = `MDP` end
    - **terminal `states`** are defined as part of the **`env` definition**
    - task `success`, task `failure`, robot falling down, etc.
    - [in `finite-horizon` `env`]
      - episodes ending due to **a time-limit inherent to the `env`**
      - here the `remaining time` should be added to the `observation`
  - **`truncation` = external interruption**
    - not defined by the `MDP`
      - e.g. **time-limit**, a robot going **out of bounds**
    - [`infinite-horizon` `env`]
      - `truncation` is needed
      - > "We cannot **wait forever** for the **episode to complete**, so we set a **practical time-limit** after which we forcibly halt the episode."

</details>

---

**`"Curriculum Learning for Reinforcement Learning Domains: A Framework and Survey"`**

- **[[:memo:](https://arxiv.org/pdf/2003.04960.pdf)]**
- **[** _`todo`, `curriculum learning`_ **]**

<details>
  <summary>Click to expand</summary>

todo

| ![](media/2020_narvekar_1.png) | 
|:--:| 
| *todo [source](https://arxiv.org/pdf/2003.04960.pdf)* |

</details>

---
---

# :yum: other good resources

<details>
  <summary>Click to expand</summary>

## papers

- [`Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning`](https://leggedrobotics.github.io/legged_gym/)
  - training the quadruped `ANYmal` robot in `Isaac Gym`
  - [presentation](https://www.youtube.com/watch?v=Afi17BnSuBM)

- [`On the Role of the Action Space in Robot Manipulation Learning and Sim-to-Real Transfer`](https://arxiv.org/abs/2312.03673)
  - "Joint velocity-based action spaces show very favorable properties, and overall the best transfer performance."

- [`Optimal Stroke Learning with Policy Gradient Approach for Robotic Table Tennis`](https://arxiv.org/pdf/2109.03100.pdf)

- [`Agile Catching with Whole-Body MPC and Blackbox Policy Learning`](https://sites.google.com/view/agile-catching)

## labs

- [`Intelligent Autonomous Systems - TU Darmstadt`](https://www.ias.informatik.tu-darmstadt.de/Videos/Videos)

- [OpenAI](https://www.youtube.com/playlist?list=PLOXw6I10VTv_CcTXlvHmGbWH-_wUOoRoO)

- [`CoRL`](https://corl2022.org/previous-conferences/)

## sim

[`nvidia Isaac Orbit`](https://isaac-orbit.github.io/orbit/source/tutorials_envs/00_gym_env.html)

- [`isaac gym`](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_isaac_gym.html)
  - [`OmniIsaacGymEnvs`](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)

- [`SimuRLacra`](https://github.com/famura/SimuRLacra)

## lectures

- [`CS285 Deep Reinforcement Learning`](https://rail.eecs.berkeley.edu/deeprlcourse/)

- [`CS 224R Deep Reinforcement Learning`](https://cs224r.stanford.edu//)

- [`CS287 Advanced Robotics at UC Berkeley Fall 2019`](https://www.youtube.com/playlist?list=PLwRJQ4m4UJjNBPJdt8WamRAt4XKc639wF)

## good practices

- finding and executing RL project
  - [`winder.ai 1`](https://winder.ai/rl-presentation-finding-and-executing-reinforcement-learning-projects/)
  - [`winder.ai 2`](https://winder.ai/rl-presentation-how-to-overcome-reinforcement-learning-challenges/)

- [`RL in practice: tips & tricks`](https://www.youtube.com/watch?v=Ikngt0_DXJg)
  - about concrete considerations when learning on real robots
  - [slides](https://araffin.github.io/slides/rlvs-tips-tricks/)

- [`Nuts and Bolts of Deep RL Experimentation`](https://www.youtube.com/watch?v=8EcdaCk9KaQ)

- [`Deep Reinforcement Learning Doesn't Work Yet`](https://www.alexirpan.com/2018/02/14/rl-hard.html)

- [`reward shaping`](https://gibberblot.github.io/rl-notes/single-agent/reward-shaping.html)
  - reward shaping: takes in some domain knowledge that "nudges" the learning algorithm towards more positive actions

</details>
