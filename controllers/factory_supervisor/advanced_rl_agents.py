"""Advanced masked-discrete RL agents for the scheduling contract."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from rl_contract import RL_SCHEDULING_CONTRACT
from rl_environment import ENVIRONMENT_VERSION
from schedulers import ModelValidationError
from advanced_rl_common import (Adam, NStepAccumulator, PrioritizedReplay,
                                masked_softmax)


@dataclass
class SarsaLambdaConfig:
    learning_rate: float = 0.08
    gamma: float = 0.99
    trace_lambda: float = 0.8
    epsilon_start: float = 1.0
    epsilon_end: float = 0.02
    epsilon_decay: float = 0.997
    replacing_traces: bool = True


class SarsaLambdaAgent:
    """Tabular on-policy SARSA(λ) with masked actions and sparse traces."""

    def __init__(self, action_dim: int, no_op_action: int,
                 config: Optional[SarsaLambdaConfig] = None, seed=42):
        self.action_dim, self.no_op_action = int(action_dim), int(no_op_action)
        self.config = config or SarsaLambdaConfig()
        if not 0 <= self.config.trace_lambda <= 1:
            raise ValueError("trace_lambda must be in [0, 1]")
        self.q_table: Dict[Tuple[int, ...], np.ndarray] = {}
        self.traces: Dict[Tuple[int, ...], np.ndarray] = {}
        self.rng = np.random.default_rng(seed)
        self.seed, self.episode = int(seed), 0
        self.epsilon = self.config.epsilon_start

    @staticmethod
    def discretize(observation):
        obs = np.asarray(observation, np.float32)
        if obs.size < 6 or not np.isfinite(obs).all():
            raise ValueError("invalid observation")
        return (int(np.clip(obs[1]*5, 0, 5)),
                int(np.clip(obs[2]*5, 0, 5)),
                int(np.clip(obs[3]*4, 0, 4)),
                int(np.clip(obs[4]*10, 0, 10)),
                int(np.clip(obs[0]*5, 0, 20)))

    def values(self, state):
        if state not in self.q_table:
            self.q_table[state] = np.zeros(self.action_dim, np.float32)
        return self.q_table[state]

    def select_action(self, state, mask, training=True):
        legal = np.flatnonzero(np.asarray(mask, bool))
        if not legal.size:
            return self.no_op_action
        if training and self.rng.random() < self.epsilon:
            return int(self.rng.choice(legal))
        q = self.values(state)
        return int(legal[np.argmax(q[legal])])

    def update(self, state, action, reward, next_state, next_action, done,
               bootstrap_discount=None):
        q = self.values(state)
        target = float(reward)
        if not done:
            discount = (self.config.gamma if bootstrap_discount is None else
                        float(bootstrap_discount))
            target += discount*self.values(next_state)[next_action]
        delta = target-float(q[action])
        trace = self.traces.setdefault(
            state, np.zeros(self.action_dim, np.float32))
        if self.config.replacing_traces:
            trace[action] = 1.0
        else:
            trace[action] += 1.0
        decay = (self.config.gamma if bootstrap_discount is None else
                 float(bootstrap_discount))*self.config.trace_lambda
        for key in list(self.traces):
            self.q_table[key] += self.config.learning_rate*delta*self.traces[key]
            self.traces[key] *= decay
            if float(np.max(np.abs(self.traces[key]))) < 1e-8:
                del self.traces[key]
        if done:
            self.traces.clear()
        return delta

    def end_episode(self):
        self.traces.clear()
        self.episode += 1
        self.epsilon = max(self.config.epsilon_end,
                           self.epsilon*self.config.epsilon_decay)

    def save(self, path):
        payload = {
            "algorithm": "SARSA_LAMBDA",
            "environment_version": ENVIRONMENT_VERSION,
            "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint(),
            "action_dim": self.action_dim, "no_op_action": self.no_op_action,
            "config": asdict(self.config), "seed": self.seed,
            "episode": self.episode, "epsilon": self.epsilon,
            "q_table": {"|".join(map(str, key)): value.tolist()
                        for key, value in self.q_table.items()},
        }
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(payload), encoding="utf-8")

    def load(self, path):
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:
            raise ModelValidationError(f"SARSA_LAMBDA checkpoint unreadable: {exc}")
        expected = ("SARSA_LAMBDA", ENVIRONMENT_VERSION,
                    RL_SCHEDULING_CONTRACT.fingerprint(), self.action_dim,
                    self.no_op_action)
        actual = (payload.get("algorithm"), payload.get("environment_version"),
                  payload.get("contract_fingerprint"), payload.get("action_dim"),
                  payload.get("no_op_action"))
        if actual != expected:
            raise ModelValidationError("SARSA_LAMBDA checkpoint mismatch")
        self.q_table = {}
        for key, values in payload.get("q_table", {}).items():
            array = np.asarray(values, np.float32)
            if array.shape != (self.action_dim,) or not np.isfinite(array).all():
                raise ModelValidationError("invalid SARSA_LAMBDA Q table")
            self.q_table[tuple(map(int, key.split("|")))] = array
        self.episode, self.epsilon = int(payload["episode"]), float(payload["epsilon"])
        self.traces.clear()


@dataclass
class A2CConfig:
    learning_rate: float = 3e-4
    gamma: float = 0.99
    value_coefficient: float = 0.5
    entropy_coefficient: float = 0.01
    max_grad_norm: float = 5.0


class A2CAgent:
    """Masked discrete Advantage Actor-Critic with linear heads."""

    algorithm = "A2C"

    def __init__(self, state_dim, action_dim, no_op_action,
                 config: Optional[A2CConfig] = None, seed=42):
        self.state_dim, self.action_dim = int(state_dim), int(action_dim)
        self.no_op_action, self.config = int(no_op_action), config or A2CConfig()
        self.seed, self.rng, self.training_step = int(seed), np.random.default_rng(seed), 0
        scale = 1/np.sqrt(self.state_dim)
        self.params = {
            "policy_w": self.rng.normal(0, scale, (self.state_dim, self.action_dim)).astype(np.float32),
            "policy_b": np.zeros(self.action_dim, np.float32),
            "value_w": self.rng.normal(0, scale, self.state_dim).astype(np.float32),
            "value_b": np.zeros(1, np.float32),
        }
        self.optimizer = Adam(self.params, self.config.learning_rate,
                              max_grad_norm=self.config.max_grad_norm)

    def distribution(self, state, mask):
        state = np.asarray(state, np.float32)
        return masked_softmax(state@self.params["policy_w"]+self.params["policy_b"], mask)

    def value(self, state):
        return float(np.asarray(state, np.float32)@self.params["value_w"]+self.params["value_b"][0])

    def select_action(self, state, mask, training=True):
        probabilities = self.distribution(state, mask)
        return int(self.rng.choice(self.action_dim, p=probabilities) if training
                   else np.argmax(probabilities))

    def update(self, state, action, reward, next_state, done, mask,
               next_mask=None, bootstrap_discount=None):
        state = np.asarray(state, np.float32)
        probabilities = self.distribution(state, mask)
        discount = (self.config.gamma if bootstrap_discount is None else
                    float(bootstrap_discount))
        target = float(reward)+(0.0 if done else discount*self.value(next_state))
        advantage = target-self.value(state)
        one_hot = np.zeros(self.action_dim, np.float32); one_hot[int(action)] = 1
        # Gradient descent on -adv*log(pi); illegal logits remain unchanged.
        grad_logits = -advantage*(one_hot-probabilities)
        grad_logits *= np.asarray(mask, bool)
        entropy = -float(np.sum(probabilities[probabilities > 0]*np.log(probabilities[probabilities > 0])))
        # Entropy gradient for softmax: p*(log p + H), legal actions only.
        safe_log = np.zeros_like(probabilities)
        legal = probabilities > 0; safe_log[legal] = np.log(probabilities[legal])
        grad_logits += self.config.entropy_coefficient*probabilities*(safe_log+entropy)
        value_error = self.value(state)-target
        grads = {
            "policy_w": np.outer(state, grad_logits), "policy_b": grad_logits,
            "value_w": self.config.value_coefficient*value_error*state,
            "value_b": np.array([self.config.value_coefficient*value_error], np.float32),
        }
        self.optimizer.step(self.params, grads); self.training_step += 1
        return {"advantage": float(advantage), "value_loss": 0.5*value_error**2,
                "entropy": entropy}

    def save(self, path):
        _save_npz_agent(path, self.algorithm, self, self.params)

    def load(self, path):
        _load_npz_agent(path, self.algorithm, self, self.params)


@dataclass
class DiscreteSACConfig:
    learning_rate: float = 3e-4
    gamma: float = 0.99
    tau: float = 0.01
    alpha: float = 0.2
    max_grad_norm: float = 5.0


class DiscreteSACAgent:
    """Masked discrete Soft Actor-Critic with twin linear Q critics."""

    algorithm = "DISCRETE_SAC"

    def __init__(self, state_dim, action_dim, no_op_action,
                 config: Optional[DiscreteSACConfig] = None, seed=42):
        self.state_dim, self.action_dim = int(state_dim), int(action_dim)
        self.no_op_action, self.config = int(no_op_action), config or DiscreteSACConfig()
        self.seed, self.rng, self.training_step = int(seed), np.random.default_rng(seed), 0
        scale = 1/np.sqrt(self.state_dim)
        self.params = {name: self.rng.normal(0, scale, (self.state_dim, self.action_dim)).astype(np.float32)
                       for name in ("policy_w", "q1_w", "q2_w")}
        self.params.update({name: np.zeros(self.action_dim, np.float32)
                            for name in ("policy_b", "q1_b", "q2_b")})
        self.target = {"q1_w": self.params["q1_w"].copy(), "q1_b": self.params["q1_b"].copy(),
                       "q2_w": self.params["q2_w"].copy(), "q2_b": self.params["q2_b"].copy()}
        self.optimizer = Adam(self.params, self.config.learning_rate,
                              max_grad_norm=self.config.max_grad_norm)

    def _linear(self, state, prefix, source=None):
        source = source or self.params
        return np.asarray(state, np.float32)@source[prefix+"_w"]+source[prefix+"_b"]

    def distribution(self, state, mask):
        return masked_softmax(self._linear(state, "policy"), mask)

    def select_action(self, state, mask, training=True):
        p = self.distribution(state, mask)
        return int(self.rng.choice(self.action_dim, p=p) if training else np.argmax(p))

    def update(self, state, action, reward, next_state, done, mask, next_mask,
               bootstrap_discount=None):
        state, next_state = np.asarray(state, np.float32), np.asarray(next_state, np.float32)
        next_p = self.distribution(next_state, next_mask)
        next_log = np.zeros_like(next_p); legal = next_p > 0; next_log[legal] = np.log(next_p[legal])
        tq1, tq2 = self._linear(next_state, "q1", self.target), self._linear(next_state, "q2", self.target)
        soft_value = float(np.sum(next_p*(np.minimum(tq1, tq2)-self.config.alpha*next_log)))
        discount = (self.config.gamma if bootstrap_discount is None else
                    float(bootstrap_discount))
        target = float(reward)+(0.0 if done else discount*soft_value)
        q1, q2 = self._linear(state, "q1"), self._linear(state, "q2")
        e1, e2 = float(q1[action]-target), float(q2[action]-target)
        gq1 = np.zeros(self.action_dim, np.float32); gq1[action] = e1
        gq2 = np.zeros(self.action_dim, np.float32); gq2[action] = e2
        p = self.distribution(state, mask); logp = np.zeros_like(p); legal = p > 0; logp[legal] = np.log(p[legal])
        min_q = np.minimum(q1, q2); objective = self.config.alpha*logp-min_q
        baseline = float(np.sum(p*objective))
        gp = p*(objective-baseline); gp *= np.asarray(mask, bool)
        grads = {"policy_w": np.outer(state, gp), "policy_b": gp,
                 "q1_w": np.outer(state, gq1), "q1_b": gq1,
                 "q2_w": np.outer(state, gq2), "q2_b": gq2}
        self.optimizer.step(self.params, grads); self.training_step += 1
        for key in self.target:
            self.target[key] = ((1-self.config.tau)*self.target[key]
                                + self.config.tau*self.params[key])
        return {"critic_loss": 0.5*(e1*e1+e2*e2), "target": target}

    def save(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _save_npz_agent(path, self.algorithm, self, values)

    def load(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _load_npz_agent(path, self.algorithm, self, values)
        for key in self.target: self.target[key][...] = values["target_"+key]


@dataclass
class QRDQNConfig:
    learning_rate: float = 3e-4
    gamma: float = 0.99
    quantiles: int = 32
    kappa: float = 1.0
    tau: float = 0.01
    max_grad_norm: float = 5.0


class QRDQNAgent:
    """Masked linear Quantile-Regression DQN with Double-DQN selection."""
    algorithm = "QR_DQN"

    def __init__(self, state_dim, action_dim, no_op_action,
                 config: Optional[QRDQNConfig] = None, seed=42):
        self.state_dim, self.action_dim = int(state_dim), int(action_dim)
        self.no_op_action, self.config = int(no_op_action), config or QRDQNConfig()
        if self.config.quantiles < 2: raise ValueError("quantiles must be >= 2")
        self.seed, self.rng, self.training_step = int(seed), np.random.default_rng(seed), 0
        shape = (self.state_dim, self.action_dim, self.config.quantiles)
        self.params = {"w": self.rng.normal(0, 1/np.sqrt(self.state_dim), shape).astype(np.float32),
                       "b": np.zeros((self.action_dim, self.config.quantiles), np.float32)}
        self.target = {k: v.copy() for k, v in self.params.items()}
        self.optimizer = Adam(self.params, self.config.learning_rate,
                              max_grad_norm=self.config.max_grad_norm)

    def quantile_values(self, state, source=None):
        source = source or self.params
        return np.tensordot(np.asarray(state, np.float32), source["w"], axes=(0, 0))+source["b"]

    def select_action(self, state, mask, training=False):
        legal = np.flatnonzero(np.asarray(mask, bool))
        if not legal.size: return self.no_op_action
        means = self.quantile_values(state).mean(axis=1)
        return int(legal[np.argmax(means[legal])])

    def update(self, state, action, reward, next_state, done, mask, next_mask,
               bootstrap_discount=None):
        state = np.asarray(state, np.float32)
        legal = np.flatnonzero(np.asarray(next_mask, bool))
        if done or not legal.size:
            target = np.full(self.config.quantiles, float(reward), np.float32)
        else:
            next_online = self.quantile_values(next_state).mean(axis=1)
            next_action = int(legal[np.argmax(next_online[legal])])
            discount = (self.config.gamma if bootstrap_discount is None else
                        float(bootstrap_discount))
            target = float(reward)+discount*self.quantile_values(next_state, self.target)[next_action]
        current = self.quantile_values(state)[action]
        delta = target[None, :]-current[:, None]
        abs_delta = np.abs(delta)
        huber_grad = np.where(abs_delta <= self.config.kappa, -delta,
                              -self.config.kappa*np.sign(delta))
        taus = (np.arange(self.config.quantiles)+0.5)/self.config.quantiles
        weights = np.abs(taus[:, None]-(delta < 0).astype(np.float32))
        grad_quantiles = np.mean(weights*huber_grad, axis=1)/self.config.kappa
        gw = np.zeros_like(self.params["w"]); gb = np.zeros_like(self.params["b"])
        gw[:, action, :] = state[:, None]*grad_quantiles[None, :]; gb[action] = grad_quantiles
        self.optimizer.step(self.params, {"w": gw, "b": gb}); self.training_step += 1
        for key in self.target:
            self.target[key] = (1-self.config.tau)*self.target[key]+self.config.tau*self.params[key]
        return {"quantile_loss_proxy": float(np.mean(abs_delta)), "target_mean": float(target.mean())}

    def save(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _save_npz_agent(path, self.algorithm, self, values)

    def load(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _load_npz_agent(path, self.algorithm, self, values)
        for key in self.target: self.target[key][...] = values["target_"+key]


@dataclass
class RainbowConfig:
    learning_rate: float = 3e-4
    gamma: float = 0.99
    atoms: int = 51
    v_min: float = -100.0
    v_max: float = 100.0
    n_steps: int = 3
    replay_capacity: int = 100000
    replay_alpha: float = 0.6
    replay_beta: float = 0.4
    noisy_sigma: float = 0.1
    tau: float = 0.01
    max_grad_norm: float = 5.0
    double: bool = True
    dueling: bool = True
    prioritized_replay: bool = True
    distributional: bool = True
    noisy: bool = True


class RainbowDQNAgent:
    """Masked C51 Rainbow agent with Double, Dueling, PER, n-step and Noisy heads."""

    algorithm = "RAINBOW_DQN"

    def __init__(self, state_dim, action_dim, no_op_action,
                 config: Optional[RainbowConfig] = None, seed=42):
        self.state_dim, self.action_dim = int(state_dim), int(action_dim)
        self.no_op_action, self.config = int(no_op_action), config or RainbowConfig()
        if self.config.atoms < 2 or self.config.v_max <= self.config.v_min:
            raise ValueError("invalid C51 support")
        required = (self.config.double, self.config.dueling,
                    self.config.prioritized_replay, self.config.distributional,
                    self.config.noisy)
        if not all(required):
            raise ValueError("RAINBOW_DQN requires all declared Rainbow components")
        self.seed, self.rng, self.training_step = int(seed), np.random.default_rng(seed), 0
        atoms, scale = self.config.atoms, 1/np.sqrt(self.state_dim)
        shapes = {"value_w": (self.state_dim, atoms), "value_b": (atoms,),
                  "adv_w": (self.state_dim, self.action_dim, atoms),
                  "adv_b": (self.action_dim, atoms)}
        self.params = {}
        for name, shape in shapes.items():
            self.params[name] = (self.rng.normal(0, scale, shape) if name.endswith("_w")
                                 else np.zeros(shape)).astype(np.float32)
            self.params[name+"_sigma"] = np.full(
                shape, self.config.noisy_sigma/np.sqrt(max(1, shape[0])), np.float32)
        self.target = {key: value.copy() for key, value in self.params.items()}
        self.optimizer = Adam(self.params, self.config.learning_rate,
                              max_grad_norm=self.config.max_grad_norm)
        self.support = np.linspace(self.config.v_min, self.config.v_max,
                                   atoms, dtype=np.float32)
        self.replay = PrioritizedReplay(self.config.replay_capacity,
                                        self.config.replay_alpha, seed)
        self.nstep = NStepAccumulator(self.config.n_steps, self.config.gamma)

    def _effective(self, source, noisy):
        result, noise = {}, {}
        for base in ("value_w", "value_b", "adv_w", "adv_b"):
            epsilon = (self.rng.standard_normal(source[base].shape).astype(np.float32)
                       if noisy else np.zeros(source[base].shape, np.float32))
            result[base] = source[base]+source[base+"_sigma"]*epsilon
            noise[base] = epsilon
        return result, noise

    def distributions(self, state, source=None, noisy=False, return_cache=False):
        source = source or self.params
        effective, noise = self._effective(source, noisy)
        state = np.asarray(state, np.float32)
        value = state@effective["value_w"]+effective["value_b"]
        advantage = np.tensordot(state, effective["adv_w"], axes=(0, 0))+effective["adv_b"]
        logits = value[None, :]+advantage-advantage.mean(axis=0, keepdims=True)
        logits -= logits.max(axis=1, keepdims=True)
        exp = np.exp(logits); probabilities = exp/exp.sum(axis=1, keepdims=True)
        return (probabilities, noise) if return_cache else probabilities

    def q_values(self, state, source=None, noisy=False):
        return self.distributions(state, source, noisy)@self.support

    def select_action(self, state, mask, training=False):
        legal = np.flatnonzero(np.asarray(mask, bool))
        if not legal.size: return self.no_op_action
        values = self.q_values(state, noisy=bool(training))
        return int(legal[np.argmax(values[legal])])

    def observe(self, state, action, reward, next_state, done, next_mask,
                bootstrap_discount=None):
        emitted = self.nstep.add(
            state, action, reward, next_state, done, next_mask,
            bootstrap_discount=bootstrap_discount)
        for transition in emitted: self.replay.add(transition)
        return len(emitted)

    def _project(self, reward, discount, next_distribution):
        target = np.zeros(self.config.atoms, np.float32)
        transformed = np.clip(float(reward)+float(discount)*self.support,
                              self.config.v_min, self.config.v_max)
        positions = (transformed-self.config.v_min)/(self.config.v_max-self.config.v_min)*(self.config.atoms-1)
        lower, upper = np.floor(positions).astype(int), np.ceil(positions).astype(int)
        for index, probability in enumerate(next_distribution):
            if lower[index] == upper[index]: target[lower[index]] += probability
            else:
                target[lower[index]] += probability*(upper[index]-positions[index])
                target[upper[index]] += probability*(positions[index]-lower[index])
        return target

    def _update_transition(self, transition, importance=1.0):
        legal = np.flatnonzero(np.asarray(transition.next_mask, bool))
        if transition.done or not legal.size or transition.bootstrap_discount == 0:
            # C51 still linearly projects a terminal Dirac return between its
            # neighbouring atoms; nearest-atom rounding biases the target.
            projected = self._project(
                transition.reward, 0.0,
                np.full(self.config.atoms, 1/self.config.atoms, np.float32))
        else:
            online = self.q_values(transition.next_state)
            next_action = int(legal[np.argmax(online[legal])])
            target_dist = self.distributions(transition.next_state, self.target)[next_action]
            projected = self._project(transition.reward, transition.bootstrap_discount, target_dist)
        probabilities, noise = self.distributions(transition.state, noisy=True, return_cache=True)
        action, chosen = transition.action, probabilities[transition.action]
        grad_logits = float(importance)*(chosen-projected)
        state = np.asarray(transition.state, np.float32)
        grads = {key: np.zeros_like(value) for key, value in self.params.items()}
        grads["value_w"] = np.outer(state, grad_logits); grads["value_b"] = grad_logits.copy()
        coefficient = np.full(self.action_dim, -1/self.action_dim, np.float32)
        coefficient[action] += 1.0
        grads["adv_w"] = state[:, None, None]*coefficient[None, :, None]*grad_logits[None, None, :]
        grads["adv_b"] = coefficient[:, None]*grad_logits[None, :]
        for base in ("value_w", "value_b", "adv_w", "adv_b"):
            grads[base+"_sigma"] = grads[base]*noise[base]
        self.optimizer.step(self.params, grads); self.training_step += 1
        for key in self.target:
            self.target[key] = (1-self.config.tau)*self.target[key]+self.config.tau*self.params[key]
        loss = -float(np.sum(projected*np.log(np.maximum(chosen, 1e-8))))
        return loss

    def train_batch(self, batch_size=32):
        transitions, indices, weights = self.replay.sample(batch_size, self.config.replay_beta)
        losses = [self._update_transition(row, weight)
                  for row, weight in zip(transitions, weights)]
        self.replay.update_priorities(indices, losses)
        return {"distributional_loss": float(np.mean(losses))}

    def save(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _save_npz_agent(path, self.algorithm, self, values)

    def load(self, path):
        values = dict(self.params); values.update({"target_"+k: v for k, v in self.target.items()})
        _load_npz_agent(path, self.algorithm, self, values)


ADVANCED_AGENT_TYPES = {
    "A2C": (A2CAgent, A2CConfig),
    "DISCRETE_SAC": (DiscreteSACAgent, DiscreteSACConfig),
    "QR_DQN": (QRDQNAgent, QRDQNConfig),
    "RAINBOW_DQN": (RainbowDQNAgent, RainbowConfig),
}


def advanced_agent_from_checkpoint(algorithm, path, state_dim, action_dim,
                                   no_op_action, seed=42):
    """Construct an advanced agent with the checkpoint's exact architecture."""
    if algorithm == "SARSA_LAMBDA":
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            config = SarsaLambdaConfig(**payload["config"])
        except Exception as exc:
            raise ModelValidationError(f"SARSA_LAMBDA checkpoint unreadable: {exc}")
        agent = SarsaLambdaAgent(action_dim, no_op_action, config, seed)
    else:
        try:
            agent_type, config_type = ADVANCED_AGENT_TYPES[algorithm]
            with np.load(path, allow_pickle=False) as data:
                config = config_type(**json.loads(str(data["config_json"][0])))
        except Exception as exc:
            raise ModelValidationError(f"{algorithm} checkpoint metadata unreadable: {exc}")
        agent = agent_type(state_dim, action_dim, no_op_action, config, seed)
    agent.load(path)
    return agent


def _save_npz_agent(path, algorithm, agent, values):
    metadata = {"algorithm": np.array([algorithm]),
                "environment_version": np.array([ENVIRONMENT_VERSION]),
                "contract_fingerprint": np.array([RL_SCHEDULING_CONTRACT.fingerprint()]),
                "state_dim": np.array([agent.state_dim]), "action_dim": np.array([agent.action_dim]),
                "no_op_action": np.array([agent.no_op_action]), "seed": np.array([agent.seed]),
                "training_step": np.array([agent.training_step]),
                "config_json": np.array([json.dumps(asdict(agent.config))])}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **metadata, **values)


def _load_npz_agent(path, algorithm, agent, values):
    try:
        with np.load(path, allow_pickle=False) as data:
            expected = (algorithm, ENVIRONMENT_VERSION, RL_SCHEDULING_CONTRACT.fingerprint(),
                        agent.state_dim, agent.action_dim, agent.no_op_action)
            actual = (str(data["algorithm"][0]), str(data["environment_version"][0]),
                      str(data["contract_fingerprint"][0]), int(data["state_dim"][0]),
                      int(data["action_dim"][0]), int(data["no_op_action"][0]))
            if actual != expected: raise ModelValidationError(f"{algorithm} checkpoint mismatch")
            for key, destination in values.items():
                source = np.asarray(data[key], np.float32)
                if source.shape != destination.shape or not np.isfinite(source).all():
                    raise ModelValidationError(f"invalid {algorithm} parameter {key}")
                destination[...] = source
            agent.training_step = int(data["training_step"][0])
    except ModelValidationError: raise
    except Exception as exc: raise ModelValidationError(f"{algorithm} checkpoint unreadable: {exc}")
