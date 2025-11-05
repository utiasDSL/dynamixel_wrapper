# Dynamixel-SDK Wrapper

A Python wrapper for the Dynamixel SDK that simplifies control of Dynamixel servo motors.

## Getting Started

### Prerequisites

This project uses [pixi](https://pixi.sh/) for package management. Install pixi:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd dynamixel_wrapper
```

2. Install dependencies using pixi:
```bash
pixi install
```

3. Activate the environment:
```bash
pixi shell
```

### Environments

The project supports multiple environments:
- `dynamixel-sdk` - Basic Dynamixel SDK environment
- `humble` - ROS 2 Humble environment
- `jazzy` - ROS 2 Jazzy environment

Activate a specific environment:
```bash
pixi shell -e humble
```
