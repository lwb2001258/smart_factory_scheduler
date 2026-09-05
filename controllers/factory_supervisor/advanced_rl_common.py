"""Shared, dependency-light primitives for advanced discrete RL schedulers."""

from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


def masked_softmax(logits, mask):
    values = np.asarray(logits, np.float64)
    legal = np.asarray(mask, bool)
    if values.shape != legal.shape:
        raise ValueError("logit/mask shape mismatch")
    if not legal.any():
        raise ValueError("masked distribution has no legal action")
    shifted = np.where(legal, values, -np.inf)
    maximum = float(np.max(shifted[legal]))
    weights = np.where(legal, np.exp(shifted - maximum), 0.0)
    total = float(weights.sum())
    if not np.isfinite(total) or total <= 0:
        raise ValueError("invalid masked distribution")
    return (weights / total).astype(np.float32)


class Adam:
    def __init__(self, params: Dict[str, np.ndarray], learning_rate=3e-4,
                 beta1=0.9, beta2=0.999, epsilon=1e-8,
                 max_grad_norm=5.0):
        self.learning_rate = float(learning_rate)
        self.beta1, self.beta2 = float(beta1), float(beta2)
        self.epsilon = float(epsilon)
        self.max_grad_norm = float(max_grad_norm)
        self.step_count = 0
        self.m = {key: np.zeros_like(value) for key, value in params.items()}
        self.v = {key: np.zeros_like(value) for key, value in params.items()}

    def step(self, params, grads):
        if set(params) != set(grads):
            raise ValueError("optimizer parameter/gradient mismatch")
        norm = np.sqrt(sum(float(np.sum(np.asarray(g) ** 2))
                           for g in grads.values()))
        scale = min(1.0, self.max_grad_norm / max(norm, 1e-12))
        self.step_count += 1
        for key in params:
            grad = np.asarray(grads[key], params[key].dtype) * scale
            if grad.shape != params[key].shape or not np.isfinite(grad).all():
                raise ValueError(f"invalid gradient: {key}")
            self.m[key] = self.beta1*self.m[key] + (1-self.beta1)*grad
            self.v[key] = self.beta2*self.v[key] + (1-self.beta2)*grad*grad
            m_hat = self.m[key] / (1-self.beta1**self.step_count)
            v_hat = self.v[key] / (1-self.beta2**self.step_count)
            params[key] -= self.learning_rate*m_hat/(np.sqrt(v_hat)+self.epsilon)


@dataclass(frozen=True)
class NStepTransition:
    state: np.ndarray
    action: int
    reward: float
    next_state: np.ndarray
    done: bool
    next_mask: np.ndarray
    bootstrap_discount: float


class NStepAccumulator:
    def __init__(self, n_steps=3, gamma=0.99):
        if int(n_steps) < 1:
            raise ValueError("n_steps must be positive")
        self.n_steps, self.gamma = int(n_steps), float(gamma)
        self.pending = deque()

    def _emit(self):
        reward, discount = 0.0, 1.0
        end = None
        for row in list(self.pending)[:self.n_steps]:
            reward += discount * row[2]
            discount *= row[6]
            end = row
            if row[4]:
                break
        first = self.pending.popleft()
        return NStepTransition(
            first[0], first[1], reward, end[3], end[4], end[5],
            0.0 if end[4] else discount)

    def add(self, state, action, reward, next_state, done, next_mask,
            bootstrap_discount=None):
        step_discount = (self.gamma if bootstrap_discount is None else
                         float(bootstrap_discount))
        if not 0.0 <= step_discount <= 1.0:
            raise ValueError("bootstrap_discount must be in [0, 1]")
        self.pending.append((np.asarray(state, np.float32).copy(), int(action),
                             float(reward), np.asarray(next_state, np.float32).copy(),
                             bool(done), np.asarray(next_mask, bool).copy(),
                             step_discount))
        output = []
        if len(self.pending) >= self.n_steps:
            output.append(self._emit())
        if done:
            while self.pending:
                output.append(self._emit())
        return output


class PrioritizedReplay:
    def __init__(self, capacity=100000, alpha=0.6, seed=42):
        if int(capacity) < 1 or not 0.0 <= float(alpha) <= 1.0:
            raise ValueError("invalid prioritized replay configuration")
        self.capacity, self.alpha = int(capacity), float(alpha)
        self.rows = []
        self.priorities = np.zeros(self.capacity, np.float64)
        self.position = 0
        self.rng = np.random.default_rng(seed)

    def __len__(self):
        return len(self.rows)

    def add(self, transition, priority: Optional[float] = None):
        value = (float(priority) if priority is not None else
                 (float(self.priorities[:len(self.rows)].max())
                  if self.rows else 1.0))
        value = max(abs(value), 1e-6)
        if len(self.rows) < self.capacity:
            self.rows.append(transition)
        else:
            self.rows[self.position] = transition
        self.priorities[self.position] = value
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, beta=0.4):
        size = len(self.rows)
        if size < batch_size:
            raise ValueError("not enough prioritized replay samples")
        scaled = self.priorities[:size] ** self.alpha
        probabilities = scaled / scaled.sum()
        indices = self.rng.choice(size, batch_size, replace=False,
                                  p=probabilities)
        weights = (size * probabilities[indices]) ** (-float(beta))
        weights /= weights.max()
        return [self.rows[int(index)] for index in indices], indices, (
            weights.astype(np.float32))

    def update_priorities(self, indices, priorities):
        for index, value in zip(indices, priorities):
            if not 0 <= int(index) < len(self.rows):
                raise IndexError("priority index out of range")
            self.priorities[int(index)] = max(abs(float(value)), 1e-6)
