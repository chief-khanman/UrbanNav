# UrbanNav: A Comprehensive Simulation Framework for Urban Air Mobility Research

[![ResearchGate](https://img.shields.io/badge/ResearchGate-Preprint-00CCBB.svg)](https://www.researchgate.net/publication/396179890_UrbanNav_A_Comprehensive_Simulation_Framework_for_Urban_Air_Mobility_Research)
[![AeroConf 2026](https://img.shields.io/badge/IEEE%20AeroConf-2026-00629B.svg)](https://aeroconf.org/conference/digest)
![Repo code size](https://img.shields.io/github/languages/code-size/chief-khanman/UrbanNav)

## Overview

Urban Air Mobility (UAM) promises to revolutionize urban transportation through autonomous aerial vehicles, but safe integration requires sophisticated simulation capabilities for validation before deployment. Current UAM simulation platforms suffer from critical limitations including oversimplified airspace models, lack of real geographic data integration, insufficient control algorithm support, and inadequate modeling of vehicle-infrastructure interactions. This paper presents UrbanNav, an open-source simulation framework addressing these limitations through comprehensive real-world modeling and extensible modular architecture. UrbanNav incorporates OpenStreetMap data for realistic urban landscapes, supports multiple UAV dynamics models from point-mass to six-degree-of-freedom implementations, provides advanced sensor modeling including camera and LiDAR systems with fusion capabilities, implements collision avoidance through external integration of ORCA algorithm, and enables seamless reinforcement learning integration via Gymnasium environments. Experimental validation demonstrates robust performance with multiple UAVs in realistic urban environments, successful RL agent training for goal-directed navigation, and comprehensive safety analysis across diverse operational scenarios. UrbanNav provides the research community with an extensible platform bridging theoretical UAM research and practical implementation challenges.

## Setup

### Getting Started (ubuntu):

```
mkdir folder_name
```

```
cd folder_name
```

```
git clone https://github.com/chief-khanman/UrbanNav.git
```

```
cd folder_name/UrbanNav
```

```
conda config --add channels conda-forge
```

```
conda env create -f environment_ubuntu.yml
```

```
conda activate AAM_AMOD
```

You will want to activate this conda environment anytime you want to run somthing in this repo.

## 📚 Citation

```bibtex
@article{khanurbannav,
  title={UrbanNav: A Comprehensive Simulation Framework for Urban Air Mobility Research},
  author={Khan, Aadit-Farhan and Haroon, Adam and Shoukry, Yasser and Fleming, Cody},
  year={2025},
  url={https://doi.org/10.13140/RG.2.2.29996.32646},
}
```

## 🙏 Acknowledgments

This work is supported by the National Science Foundation under grant number CNS-2313104 and the Iowa Space Grant Consortium under NASA Award No. 80NSSC25M7126.

## 👥 Contributors

- [Aadit-Farhan Khan](https://github.com/chief-khanman) (Iowa State University)
- [Adam Haroon](https://github.com/oadamharoon) (Iowa State University)
- [Yasser Shoukry](https://scholar.google.com/citations?user=KRXEPKAAAAAJ&hl=en) (University of California, Irvine)
- [Cody Fleming](https://scholar.google.com/citations?user=Pefc1GgAAAAJ&hl=en) (Iowa State University)

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🔗 Links

- [Paper (ResearchGate)](https://doi.org/10.13140/RG.2.2.29996.32646)
- [IEEE Aerospace Conference 2026](https://aeroconf.org/conference/digest)
- [Iowa State University](https://www.iastate.edu/)

---

*For questions or issues, please open a GitHub issue or contact the authors.*
