#!/bin/bash
# Isolated Test Script for DUPLO Train Controller
# 
# This script creates a separate git worktree for testing different versions
# without affecting your current development branches.
#
# Usage: Run from the test/ directory
# The script will create a separate worktree in ./testing_env/
#
# Test Cases:
# 1. Basic Initial Version: Original implementation (commit c2142b5)
# 2. Extended Sensor: Full sensor integration with bidirectional queues
# 3. Compare Versions: Side-by-side analysis of implementations
# 4. Branch Status: Git repository state overview
# 5. Multi-Task Implementation: FreeRTOS multi-core architecture
# 6. Reconnect Integrated: Connection recovery testing (commit 248efdb)
# 7. Comprehensive Testing: Build verification for all versions

echo "🚂 DUPLO Train Controller - Isolated Test Script"
echo "================================================"

# Get the main project directory (parent of test directory)
MAIN_DIR="$(cd .. && pwd)"
TEST_WORKTREE_DIR="./testing_env"

# Find PlatformIO executable
PIO_CMD=""
if command -v pio &> /dev/null; then
    PIO_CMD="pio"
elif [ -f "$HOME/.platformio/penv/bin/pio" ]; then
    PIO_CMD="$HOME/.platformio/penv/bin/pio"
elif [ -f "$HOME/.local/bin/pio" ]; then
    PIO_CMD="$HOME/.local/bin/pio"
else
    echo "⚠️  PlatformIO not found in PATH. Please install PlatformIO first."
    echo "💡 Install with: pip install platformio"
    exit 1
fi

echo "📡 Using PlatformIO: $PIO_CMD"

# Helper function to navigate to the project directory in testing environment
navigate_to_project() {
    cd "$TEST_WORKTREE_DIR"
    if [ -d "DuploTrain" ]; then
        cd DuploTrain
    fi
}

# Helper function to return to test directory
return_to_test_dir() {
    cd "$MAIN_DIR/test"
}

# Function to setup isolated testing environment
setup_test_environment() {
    echo "🔧 Setting up isolated testing environment..."
    
    # Change to main directory to create worktree
    cd "$MAIN_DIR"
    
    # Remove existing testing worktree if it exists
    if [ -d "test/testing_env" ]; then
        echo "🧹 Cleaning up existing testing environment..."
        git worktree remove test/testing_env --force 2>/dev/null || true
        rm -rf test/testing_env 2>/dev/null || true
    fi
    
    # Create new worktree for testing - use a commit instead of branch to avoid conflicts
    echo "📁 Creating new testing worktree..."
    
    # Try to create worktree with the latest commit from extended sensor support
    LATEST_COMMIT=$(git rev-parse feature/extended-sensor-support)
    git worktree add test/testing_env "$LATEST_COMMIT"
    
    if [ $? -eq 0 ]; then
        echo "✅ Testing environment created successfully at test/testing_env/"
        echo "🔍 This is completely isolated from your development branches"
        echo "📍 Testing environment is at commit: $LATEST_COMMIT"
        
        # Switch to testing_env and create a local testing branch
        cd test/testing_env
        git checkout -b testing-branch 2>/dev/null || true
        echo "🌿 Created local testing branch for safe operations"
        
    else
        echo "❌ Failed to create testing environment"
        echo "💡 Trying alternative setup method..."
        
        # Alternative: clone the repository to test directory
        echo "📁 Creating testing environment via clone..."
        git clone . test/testing_env
        
        if [ $? -eq 0 ]; then
            echo "✅ Testing environment created successfully via clone"
            cd test/testing_env
            git checkout -b testing-branch 2>/dev/null || true
        else
            echo "❌ Failed to create testing environment"
            echo "💡 Please ensure you have proper Git permissions and are in a Git repository"
            exit 1
        fi
    fi
    
    # Return to test directory
    cd "$MAIN_DIR/test"
}

# Function to test basic version (initial implementation)
test_basic() {
    echo "📦 Testing Basic Initial Version..."
    navigate_to_project
    git checkout c2142b5
    echo "✅ Switched to initial version (c2142b5)"
    
    echo ""
    echo "🔌 HARDWARE CONNECTION CHECK:"
    echo "Please ensure:"
    echo "1. ESP32 is connected via USB"
    echo "2. USB cable supports data transfer (not power-only)"
    echo "3. ESP32 is powered on"
    echo "4. Press and hold BOOT button on ESP32 during upload if needed"
    echo ""
    
    read -p "Continue with upload? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Upload cancelled. Running compile-only test..."
        "$PIO_CMD" run
        return_to_test_dir
        return
    fi
    
    echo "💾 Building and uploading..."
    "$PIO_CMD" run --target upload --upload-port /dev/cu.usbserial-0001
    
    if [ $? -eq 0 ]; then
        echo "✅ Upload successful!"
        echo ""
        echo "📊 Starting monitor..."
        echo "🔍 Expected: Original basic implementation without multi-task architecture"
        echo "💡 Press Ctrl+C to stop monitoring"
        echo ""
        
        # Try to start monitor with better error handling
        "$PIO_CMD" device monitor --baud 115200 2>/dev/null || {
            echo "⚠️  Monitor failed to start. You can manually monitor with:"
            echo "   screen /dev/cu.usbserial-0001 115200"
            echo "   or use Arduino IDE Serial Monitor"
        }
    else
        echo "❌ Upload failed!"
        echo ""
        echo "🔧 TROUBLESHOOTING STEPS:"
        echo "1. Check ESP32 connection and power"
        echo "2. Try holding BOOT button during upload"
        echo "3. Check if another program is using the serial port"
        echo "4. Try different USB cable"
        echo "5. Reset ESP32 and try again"
        echo ""
        echo "📋 Hardware details:"
        ls -la /dev/cu.* 2>/dev/null || echo "No serial devices found"
    fi
    
    return_to_test_dir
}

# Function to test extended version  
test_extended() {
    echo "🔬 Testing Extended Sensor Version..."
    navigate_to_project
    git checkout feature/extended-sensor-support
    echo "✅ Switched to extended version"
    echo "💾 Building and uploading..."
    "$PIO_CMD" run --target upload --upload-port /dev/cu.usbserial-0001
    echo "📊 Starting monitor (Press Ctrl+C to stop)..."
    echo "🔍 Expected: All basic features + color/distance/button sensors"
    "$PIO_CMD" device monitor
    return_to_test_dir
}

# Function to compare versions
compare_versions() {
    echo "🔄 Comparing Versions..."
    navigate_to_project
    echo "📋 Basic initial version (c2142b5):"
    git show c2142b5 --name-only | head -10
    echo ""
    echo "📋 Reconnect integrated version (248efdb):"
    git show 248efdb --name-only | head -10
    echo ""
    echo "📋 Multi-task implementation:"
    git show feature/multi-core-implementation --name-only | head -10
    echo ""
    echo "📋 Extended sensor implementation:"
    git show feature/extended-sensor-support --name-only | head -10
    echo ""
    echo "📊 Code differences (basic initial vs reconnect integrated):"
    git diff --stat c2142b5..248efdb
    echo ""
    echo "📊 Code differences (multi-task vs extended sensor):"
    git diff --stat feature/multi-core-implementation..feature/extended-sensor-support
    return_to_test_dir
}

# Function to test multi-task implementation
test_multi_task() {
    echo "🌱 Testing Multi-Task Implementation..."
    navigate_to_project
    git checkout feature/multi-core-implementation
    echo "✅ Switched to multi-task branch"
    echo "💾 Building and uploading..."
    "$PIO_CMD" run --target upload --upload-port /dev/cu.usbserial-0001
    echo "📊 Starting monitor (Press Ctrl+C to stop)..."
    echo "🔍 Expected: Multi-task architecture, non-blocking BLE operations"
    "$PIO_CMD" device monitor
    return_to_test_dir
}

# Function to test reconnect integration
test_reconnect_integrated() {
    echo "🔄 Testing Connection Recovery Integration..."
    navigate_to_project
    git checkout 248efdb
    echo "✅ Switched to 'reconnect integrated' version (248efdb)"
    echo "💾 Building and uploading..."
    "$PIO_CMD" run --target upload --upload-port /dev/cu.usbserial-0001
    
    echo ""
    echo "🧪 RECONNECTION TEST PROCEDURE:"
    echo "1. Wait for initial hub connection"
    echo "2. Turn off DUPLO hub to simulate disconnection"
    echo "3. Observe automatic reconnection attempts"
    echo "4. Turn hub back on to test reconnection"
    echo "5. Verify reconnection functionality works"
    echo ""
    echo "📊 Starting monitor for reconnect testing (Press Ctrl+C to stop)..."
    echo "🔍 Expected: Reconnect integrated functionality, connection recovery"
    
    "$PIO_CMD" device monitor
    return_to_test_dir
}

# Function to run comprehensive testing sequence
test_comprehensive() {
    echo "🧪 Comprehensive Testing Sequence..."
    echo "This will test all versions sequentially for comparison"
    echo ""
    
    read -p "⚠️  This will switch branches multiple times in testing environment. Continue? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Test cancelled."
        return
    fi
    
    cd "$TEST_WORKTREE_DIR"
    if [ -d "DuploTrain" ]; then
        cd DuploTrain
    fi
    
    echo ""
    echo "🔍 TESTING SEQUENCE:"
    echo "1. Basic initial version (c2142b5)"
    echo "2. Reconnect integrated version (248efdb)"
    echo "3. Multi-task implementation (feature/multi-core-implementation)"  
    echo "4. Extended sensor support (feature/extended-sensor-support)"
    echo ""
    
    # Test 1: Basic initial
    echo "📋 TEST 1: Basic Initial Version"
    git checkout c2142b5
    echo "Building basic initial version..."
    "$PIO_CMD" run
    if [ $? -eq 0 ]; then
        echo "✅ Basic initial implementation builds successfully"
    else
        echo "❌ Basic initial implementation build failed"
    fi
    echo ""
    
    # Test 2: Reconnect integrated
    echo "📋 TEST 2: Reconnect Integrated"
    git checkout 248efdb
    echo "Building reconnect integrated version..."
    "$PIO_CMD" run
    if [ $? -eq 0 ]; then
        echo "✅ Reconnect integrated implementation builds successfully"
    else
        echo "❌ Reconnect integrated implementation build failed"
    fi
    echo ""
    
    # Test 3: Multi-task
    echo "📋 TEST 3: Multi-Task Implementation"
    git checkout feature/multi-core-implementation
    echo "Building multi-task version..."
    "$PIO_CMD" run
    if [ $? -eq 0 ]; then
        echo "✅ Multi-task implementation builds successfully"
    else
        echo "❌ Multi-task implementation build failed"
    fi
    echo ""
    
    # Test 4: Extended sensor
    echo "📋 TEST 4: Extended Sensor Support"
    git checkout feature/extended-sensor-support  
    echo "Building extended sensor version..."
    "$PIO_CMD" run
    if [ $? -eq 0 ]; then
        echo "✅ Extended sensor implementation builds successfully"
    else
        echo "❌ Extended sensor implementation build failed"
    fi
    echo ""
    
    echo "🎯 COMPREHENSIVE TEST RESULTS:"
    echo "All versions compile-tested. Choose specific version for upload testing."
    echo "Use options 1, 2, 5, or 6 for individual version testing with upload."
    
    return_to_test_dir
}

# Function to show status of both main repo and test environment
show_status() {
    echo "📊 Repository Status:"
    echo ""
    echo "🏠 MAIN DEVELOPMENT REPOSITORY:"
    echo "Current directory: $MAIN_DIR"
    cd "$MAIN_DIR"
    echo "Current branch: $(git branch --show-current)"
    git status --short
    echo ""
    
    if [ -d "test/testing_env" ]; then
        echo "🧪 TESTING ENVIRONMENT:"
        echo "Testing directory: test/testing_env"
        cd test/testing_env
        echo "Current branch: $(git branch --show-current)"
        git status --short
        cd ../..
    else
        echo "🧪 TESTING ENVIRONMENT: Not set up yet"
        echo "💡 Run option 8 to set up testing environment"
    fi
    
    echo ""
    echo "🌳 All branches:"
    git branch -v
    echo ""
    echo "📈 Recent commits:"
    git log --oneline --graph --all -10
    
    # Return to test directory  
    cd "$MAIN_DIR/test"
}

# Function to cleanup testing environment
cleanup_test_environment() {
    echo "🧹 Cleaning up testing environment..."
    cd "$MAIN_DIR"
    
    if [ -d "test/testing_env" ]; then
        git worktree remove test/testing_env --force
        echo "✅ Testing environment cleaned up"
    else
        echo "ℹ️  No testing environment found to clean up"
    fi
    
    cd test
}

# Check if we're in the right directory
if [[ ! -f "../platformio.ini" ]]; then
    echo "❌ Error: This script should be run from the test/ directory"
    echo "💡 Please navigate to the test/ directory and run: ./test_versions_isolated.sh"
    exit 1
fi

# Check if testing environment exists
if [ ! -d "$TEST_WORKTREE_DIR" ]; then
    echo "⚠️  Testing environment not found."
    echo "💡 Setting up isolated testing environment first..."
    setup_test_environment
    echo ""
fi

# Main menu
echo "Choose testing option:"
echo "1) Test Basic Initial Version (c2142b5)"
echo "2) Test Extended Sensor Version"  
echo "3) Compare Versions"
echo "4) Show Repository Status (both main & test)"
echo "5) Test Multi-Task Implementation"
echo "6) Test Reconnect Integrated (248efdb)"
echo "7) Comprehensive Testing (All Versions)"
echo "8) Setup/Reset Testing Environment"
echo "9) Cleanup Testing Environment"

read -p "Enter choice (1-9): " choice

case $choice in
    1) test_basic ;;
    2) test_extended ;;
    3) compare_versions ;;
    4) show_status ;;
    5) test_multi_task ;;
    6) test_reconnect_integrated ;;
    7) test_comprehensive ;;
    8) setup_test_environment ;;
    9) cleanup_test_environment ;;
    *) echo "Invalid choice" ;;
esac
