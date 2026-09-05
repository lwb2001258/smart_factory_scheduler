"""Time-aware discount helpers for event-driven scheduling transitions."""

import math


def elapsed_bootstrap_discount(elapsed_seconds: float, *,
                               gamma_base: float = 0.99,
                               reference_seconds: float = 10.0,
                               terminal: bool = False) -> float:
    if terminal:
        return 0.0
    elapsed = max(0.0, float(elapsed_seconds))
    gamma = float(gamma_base)
    reference = float(reference_seconds)
    if not 0.0 < gamma <= 1.0 or not math.isfinite(gamma):
        raise ValueError("gamma_base must be finite in (0, 1]")
    if reference <= 0.0 or not math.isfinite(reference):
        raise ValueError("reference_seconds must be positive and finite")
    return gamma ** (elapsed / reference)

