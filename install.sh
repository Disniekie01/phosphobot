#!/bin/bash
set -e

# IRL Robotics - Installation Script
# Installs the complete IRL Robotics stack on a fresh Ubuntu machine
# Usage: curl -sSL https://raw.githubusercontent.com/Disniekie01/phosphobot/main/install.sh | bash
#   OR: git clone https://github.com/Disniekie01/phosphobot.git && cd phosphobot && bash install.sh

echo "============================================"
echo "  IRL Robotics - Installation Script"
echo "============================================"
echo ""

# Determine install directory
INSTALL_DIR="${IRL_INSTALL_DIR:-$HOME/phosphobot}"

# If we're already inside the repo, use that
if [ -f "$(pwd)/phosphobot/pyproject.toml" ]; then
    INSTALL_DIR="$(pwd)"
    echo "Using existing repo at: $INSTALL_DIR"
else
    echo "Install directory: $INSTALL_DIR"
fi

# ============================================
# 1. System dependencies
# ============================================
echo ""
echo "[1/6] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    git curl wget build-essential \
    python3-dev python3-pip \
    libgl1-mesa-glx libglib2.0-0 \
    libudev-dev \
    v4l-utils \
    2>/dev/null

# Add user to dialout group for serial port access (SO-100 robots)
if ! groups "$USER" | grep -q dialout; then
    sudo usermod -aG dialout "$USER"
    echo "  Added $USER to dialout group (re-login required for serial access)"
fi

# ============================================
# 2. Install uv (Python package manager)
# ============================================
echo ""
echo "[2/6] Installing uv..."
if ! command -v uv &> /dev/null; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    echo "  uv installed"
else
    echo "  uv already installed ($(uv --version))"
fi

# ============================================
# 3. Install Node.js (for dashboard)
# ============================================
echo ""
echo "[3/6] Installing Node.js..."
if ! command -v node &> /dev/null; then
    # Install via nvm
    if [ ! -d "$HOME/.nvm" ]; then
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
    fi
    export NVM_DIR="$HOME/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
    nvm install 20
    nvm use 20
    echo "  Node.js $(node --version) installed"
else
    echo "  Node.js already installed ($(node --version))"
fi

# ============================================
# 4. Clone/update the repository
# ============================================
echo ""
echo "[4/6] Setting up repository..."
if [ ! -f "$INSTALL_DIR/phosphobot/pyproject.toml" ]; then
    git clone https://github.com/Disniekie01/phosphobot.git "$INSTALL_DIR"
    echo "  Cloned to $INSTALL_DIR"
else
    echo "  Repository already exists at $INSTALL_DIR"
fi
cd "$INSTALL_DIR"

# ============================================
# 5. Install Python dependencies & build dashboard
# ============================================
echo ""
echo "[5/6] Installing Python dependencies..."
cd "$INSTALL_DIR/phosphobot"
uv sync --python 3.10
echo "  Python dependencies installed"

echo ""
echo "Building dashboard..."
cd "$INSTALL_DIR/dashboard"

# Ensure nvm is loaded
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"

npm install --silent 2>/dev/null
npm run build
mkdir -p "$INSTALL_DIR/phosphobot/resources/dist/"
cp -r ./dist/* "$INSTALL_DIR/phosphobot/resources/dist/"
echo "  Dashboard built"

# ============================================
# 6. Install ROS2 bridge (optional - only if ROS2 is available)
# ============================================
echo ""
echo "[6/6] ROS2 bridge setup..."
ROS_DISTRO_USED=""
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO_USED="jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO_USED="humble"
fi
if [ -n "$ROS_DISTRO_USED" ]; then
    echo "  ROS2 $ROS_DISTRO_USED detected, building bridge..."
    source "/opt/ros/$ROS_DISTRO_USED/setup.bash"
    cd "$INSTALL_DIR/so-arm101-ros2-bridge"
    colcon build 2>/dev/null
    echo "  ROS2 bridge built"
    echo "  Install topic_tools for relays: sudo apt install -y ros-$ROS_DISTRO_USED-topic-tools"
    echo "  Source it with: source $INSTALL_DIR/so-arm101-ros2-bridge/install/setup.bash"
else
    echo "  ROS2 not found - skipping bridge build (optional)"
    echo "  Install ROS2 Jazzy or Humble and ros-<distro>-topic-tools if you need Isaac Sim integration"
fi

# ============================================
# Done!
# ============================================
echo ""
echo "============================================"
echo "  Installation complete!"
echo "============================================"
echo ""
echo "To start IRL Robotics:"
echo "  cd $INSTALL_DIR/phosphobot"
echo "  uv run --python 3.10 irlrobotics run"
echo ""
echo "Or use make:"
echo "  cd $INSTALL_DIR"
echo "  make prod"
echo ""
echo "Dashboard will be at: http://localhost:8020"
echo ""
echo "NOTE: If this is first run, you may need to:"
echo "  1. Re-login for serial port access (dialout group)"
echo "  2. Run 'sudo chmod 666 /dev/ttyACM*' for immediate access"
echo "  3. Calibrate your robots through the dashboard"
echo ""
