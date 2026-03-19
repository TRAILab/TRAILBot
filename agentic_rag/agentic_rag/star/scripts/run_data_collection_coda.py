import os, sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
sys.path.append(sys.path[0] + '/..')
import logging

import hydra
from omegaconf import DictConfig

from agentic_rag.star.some_class.datasets_class_CODa import CODaDataset
from scenegraph.helpers import iter_by_dataset
from scenegraph.scenegraph_constructor import run_scenegraph_generation

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg : DictConfig):
    cfg = cfg['scenegraph']
    
    dataset = CODaDataset(cfg.basedir, cfg.sequence, stride=cfg.stride, start=cfg.start, end=cfg.end)
    run_scenegraph_generation(cfg, dataset, None, None, None, iter_by_dataset)

if __name__ == "__main__":
    logging.warn("This is just a test script!")
    main()
    