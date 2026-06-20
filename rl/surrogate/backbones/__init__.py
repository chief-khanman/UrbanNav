"""
backbones/
===========
Surrogate-model backbones. Add a new backbone by:
1. Subclassing SurrogateModel (surrogate_template.py) and implementing
   either or both of predict_next_state / predict_episode_outcome.
2. Registering it in BACKBONE_REGISTRY below.

The training loop in rl/surrogate/train.py is driven by BACKBONE_REGISTRY,
so new entries become trainable without any code change there.
"""

from __future__ import annotations

from typing import Dict, Type

from rl.surrogate.backbones.gnn_backbone import GNNSurrogateBackbone
from rl.surrogate.backbones.surrogate_template import SurrogateModel

BACKBONE_REGISTRY: Dict[str, Type[SurrogateModel]] = {
    'GNN': GNNSurrogateBackbone,
}

__all__ = ['SurrogateModel', 'GNNSurrogateBackbone', 'BACKBONE_REGISTRY']
