#!/bin/bash

set -e

# ─── Colors ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
BOLD='\033[1m'
RESET='\033[0m'

info()    { echo -e "${CYAN}${BOLD}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}${BOLD}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}${BOLD}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}${BOLD}[ERROR]${RESET} $*"; exit 1; }

WS_DIR="/ws"
WS_REPO="git@github.com:hivebotics/ws.git"
WS_BRANCH="staging-v2.3.0"

SUBREPOS=(
    "frontend|git@github.com:hivebotics/frontend.git|feat-common-features"
    "backend|git@github.com:hivebotics/backend.git|develop"
    "nav2_ws|git@github.com:hivebotics/nav2_ws.git|develop"
    "hivebotics_moveit_pro|git@github.com:hivebotics/hivebotics_moveit_pro.git|feat-seat-lifter-exec-service"
    "hivebotics_mission_manager|git@github.com:hivebotics/hivebotics_mission_manager.git|develop"
)

echo -e "${BOLD}${CYAN}"
echo "  ┌──────────────────────────────────────────┐"
echo "  │        Hivebotics — Repository Setup       │"
echo "  │   git@github.com:hivebotics/ws.git         │"
echo "  └──────────────────────────────────────────┘"
echo -e "${RESET}"

# ─── Pre-flight: Check sudo ───────────────────────────────────────────────────
if ! sudo -v 2>/dev/null; then
    error "This script requires sudo privileges. Run with: sudo bash $0"
fi

# ─── Step 1: Check SSH key ────────────────────────────────────────────────────
info "Checking SSH access to GitHub..."
SSH_TEST=$(ssh -o StrictHostKeyChecking=no -T git@github.com 2>&1 || true)
if echo "$SSH_TEST" | grep -q "successfully authenticated"; then
    success "SSH access confirmed"
else
    warn "SSH key not authenticated. Setting up SSH key..."
    if [ ! -f "$HOME/.ssh/id_ed25519" ]; then
        ssh-keygen -t ed25519 -C "hivebotics" -f "$HOME/.ssh/id_ed25519" -N ""
        success "SSH key generated at ~/.ssh/id_ed25519"
    fi
    echo ""
    echo -e "${YELLOW}  Add this public key to GitHub → Settings → SSH Keys:${RESET}"
    echo -e "${BOLD}$(cat "$HOME/.ssh/id_ed25519.pub")${RESET}"
    echo ""
    echo -e "${YELLOW}  Then re-run this script.${RESET}"
    exit 1
fi

# ─── Step 2: Clone ws repo ───────────────────────────────────────────────────
info "Setting up main repository..."
if [ ! -d "$WS_DIR/.git" ]; then
    TEMP_DIR="$HOME/ws_clone_temp"
    git clone --branch "$WS_BRANCH" "$WS_REPO" "$TEMP_DIR"
    sudo mv "$TEMP_DIR" "$WS_DIR"
    sudo chown -R "$USER:$USER" "$WS_DIR"
    success "Repository cloned and moved to $WS_DIR"
else
    cd "$WS_DIR"
    git pull origin "$WS_BRANCH"
    success "Repository updated → $WS_BRANCH"
fi

# ─── Step 3: Permissions ─────────────────────────────────────────────────────
info "Setting permissions..."
sudo chmod -R 777 "$WS_DIR"
success "Permissions set"

# ─── Step 4: Sub-repositories ────────────────────────────────────────────────
info "Syncing sub-repositories..."
for entry in "${SUBREPOS[@]}"; do
    FOLDER=$(echo "$entry" | cut -d'|' -f1)
    REPO=$(echo "$entry"   | cut -d'|' -f2)
    BRANCH=$(echo "$entry" | cut -d'|' -f3)
    DIR="$WS_DIR/$FOLDER"

    if [ -d "$DIR/.git" ]; then
        cd "$DIR"
        CURRENT=$(git rev-parse --abbrev-ref HEAD)
        if [ "$CURRENT" != "$BRANCH" ]; then
            git fetch origin
            git checkout "$BRANCH"
        fi
        git pull origin "$BRANCH"
        success "$FOLDER → $BRANCH"
    else
        TEMP_DIR="$HOME/${FOLDER}_clone_temp"
        git clone --branch "$BRANCH" "$REPO" "$TEMP_DIR"
        sudo mv "$TEMP_DIR" "$DIR"
        sudo chown -R "$USER:$USER" "$DIR"
        success "$FOLDER cloned → $BRANCH"
    fi
done

# ─── Step 5: Shell environment ───────────────────────────────────────────────
info "Configuring shell environment..."
if ! grep -q "WS_DIRECTORY" "$HOME/.bashrc"; then
    echo "export WS_DIRECTORY=\"/ws\"" >> "$HOME/.bashrc"
    info "Added WS_DIRECTORY to .bashrc"
fi
if ! grep -q "init_shell.sh" "$HOME/.bashrc"; then
    echo 'source $WS_DIRECTORY/system/scripts/init_shell.sh' >> "$HOME/.bashrc"
    info "Added init_shell.sh sourcing to .bashrc"
fi
success "Shell environment configured"


# ─── Summary ─────────────────────────────────────────────────────────────────
cd "$WS_DIR"
COMMIT=$(git log -1 --format="%h — %s (%cr)" 2>/dev/null)

echo ""
echo -e "${BOLD}${GREEN}════════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${GREEN}  Repository ready!${RESET}"
echo -e "${GREEN}  Location : $WS_DIR${RESET}"
echo -e "${GREEN}  Branch   : $WS_BRANCH${RESET}"
echo -e "${GREEN}  Latest   : $COMMIT${RESET}"
echo ""
echo -e "${YELLOW}  Fill system parameters in: /ws/system/params/${RESET}"
echo -e "${YELLOW}    backend.yaml, hardware.yaml, moveit_pro.yaml,${RESET}"
echo -e "${YELLOW}    ros.yaml, site.yaml, version.yaml${RESET}"
echo ""
echo -e "${YELLOW}  Then run: source ~/.bashrc${RESET}"
echo -e "${BOLD}${GREEN}════════════════════════════════════════════════════════${RESET}"
