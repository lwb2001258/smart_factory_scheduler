# Wenbin Lin MSc Dissertation Presentation — Slide Outline

1. **Title** — Dissertation identity and implemented Webots scope.
2. **Background and research question** — Why task allocation and physical coordination interact.
3. **Objectives and contributions** — Proposed, implemented and experimentally evaluated methodology.
4. **Smart-factory environment** — Verified layout, robot, grid, stations and safety configuration.
5. **Scenes A/B/C** — 3/5/8 robots and 30/15/8 s mean Poisson arrival intervals.
6. **System architecture** — Task generation through scheduling, planning, robot control and feedback.
7. **Algorithm taxonomy** — Twelve implemented schedulers grouped by method family.
8. **Classical and metaheuristic allocation** — Hungarian, Auction, GA and SA mechanisms and parameters.
9. **Learning-based scheduling** — State, action, reward, exploration and policy/value learning.
10. **Reward and training pipeline** — Actual champion DQN/SARSA reward terms and update loop.
11. **RL training evidence** — DQN, SARSA and PPO champion training curves.
12. **Measured performance** — Webots throughput across A/B/C.
13. **Scene C trade-off** — Throughput, completion time and waiting time.
14. **Coordination and scalability** — Throughput trend from 3 to 8 robots and safety evidence.
15. **Overall comparison** — Scene C normalised heatmap and H1–H5 assessment.
16. **Conclusions and future work** — Defensible findings, limitations, next experiments and Q&A.

## Figures and charts generated

- Repository-derived factory layout diagram.
- Closed-loop system architecture diagram.
- GA and SA process diagrams.
- RL state/action/reward/training diagram.
- DQN/SARSA/PPO episode-reward curves with moving averages.
- A/B/C measured-throughput grouped bar chart.
- Scene C throughput/completion/waiting comparison.
- Fleet-size versus throughput line chart.
- Scene C normalised multi-metric heatmap.

## External references cited

- Kuhn, H. W. (1955). *The Hungarian Method for the Assignment Problem*.
- Kirkpatrick, S., Gelatt, C. D. and Vecchi, M. P. (1983). *Optimization by Simulated Annealing*.
- Mnih, V. et al. (2015). *Human-level control through deep reinforcement learning*.
- Schulman, J. et al. (2017). *Proximal Policy Optimization Algorithms*.
- Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.).

No external figures or experimental datasets are used. The title visual is a project-owned illustrative asset and is not presented as a simulator screenshot.

## Evidence limitations before defence

- Collect at least five complete, recorded seeds for every algorithm–scene pair.
- Store seed identifiers in `experiment_info` and calculate confidence intervals.
- Log GA best fitness by generation and SA best/current cost by iteration.
- Standardise conflict, collision and minimum-separation event recording.
- Add per-robot productive utilisation distinct from the current near-zero idle statistic.
- Run reward and coordination-layer ablations for DQN, SARSA and PPO.
