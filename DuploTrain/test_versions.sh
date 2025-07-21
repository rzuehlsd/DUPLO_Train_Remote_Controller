#!/bin/bash
# Quick Test Script for DUPLO Train Controller
# 
# Test Cases:
# 1. Basic Initial Version: Original implementation (commit c2142b5)
# 2. Extended Sensor: Full sensor integration with bidirectional queues
# 3. Compare Versions: Side-by-side analysis of implementations
# 4. Branch Status: Git repository state overview
# 5. Multi-Task Implementation: FreeRTOS multi-core architecture
# 6. Reconnect Integrated: Connection recovery testing (commit 248efdb)
# 7. Comprehensive Testing: Build verification for all versions

echo "🚂 DUPLO Train Controller - Quick Test Script"
echo "=============================================="

# Function to test basic version (initial implementation)
test_basic() {
    echo "📦 Testing Basic Initial Version..."
    git checkout c2142b5
    echo "✅ Switched to initial version (c2142b5)"
    echo "💾 Building and uploading..."
    pio run --target upload
    echo "📊 Starting monitor (Press Ctrl+C to stop)..."
    echo "🔍 Expected: Original basic implementation without multi-task architecture"
    pio device monitor
}

# Function to test extended version  
test_extended() {
    echo "🔬 Testing Extended Sensor Version..."
    git checkout feature/extended-sensor-support
    echo "✅ Switched to extended version"
    echo "💾 Building and uploading..."
    pio run --target upload
    echo "📊 Starting monitor (Press Ctrl+C to stop)..."
    echo "🔍 Expected: All basic features + color/distance/button sensors"
    pio device monitor
}

# Function to compare versions
compare_versions() {
    echo "🔄 Comparing Versions..."
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
}

# Function to test multi-task implementation (original main branch)
test_multi_task() {
    echo "🌱 Testing Multi-Task Implementation..."
    git checkout feature/multi-core-implementation
    echo "✅ Switched to multi-task branch"
    echo "💾 Building and uploading..."
    pio run --target upload
    echo "📊 Starting monitor (Press Ctrl+C to stop)..."
    echo "🔍 Expected: Multi-task architecture, non-blocking BLE operations"
    pio device monitor
}

# Function to test reconnect integration
test_reconnect_integrated() {
    echo "🔄 Testing Connection Recovery Integration..."
    git checkout 248efdb
    echo "✅ Switched to 'reconnect integrated' version (248efdb)"
    echo "💾 Building and uploading..."
    pio run --target upload
    
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
    
    pio device monitor
}

# Function to run comprehensive testing sequence
test_comprehensive() {
    echo "🧪 Comprehensive Testing Sequence..."
    echo "This will test all versions sequentially for comparison"
    echo ""
    
    read -p "⚠️  This will switch branches multiple times. Continue? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Test cancelled."
        return
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
    pio run --target compile
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
    pio run --target compile
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
    pio run --target compile
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
    pio run --target compile
    if [ $? -eq 0 ]; then
        echo "✅ Extended sensor implementation builds successfully"
    else
        echo "❌ Extended sensor implementation build failed"
    fi
    echo ""
    
    echo "🎯 COMPREHENSIVE TEST RESULTS:"
    echo "All versions compile-tested. Choose specific version for upload testing."
    echo "Use options 1, 2, 5, or 6 for individual version testing with upload."
}

# Main menu
echo "Choose testing option:"
echo "1) Test Basic Initial Version (c2142b5)"
echo "2) Test Extended Sensor Version"  
echo "3) Compare Versions"
echo "4) Show Branch Status"
echo "5) Test Multi-Task Implementation"
echo "6) Test Reconnect Integrated (248efdb)"
echo "7) Comprehensive Testing (All Versions)"

read -p "Enter choice (1-7): " choice

case $choice in
    1) test_basic ;;
    2) test_extended ;;
    3) compare_versions ;;
    4) git branch -v && git log --oneline --graph --all -10 ;;
    5) test_multi_task ;;
    6) test_reconnect_integrated ;;
    7) test_comprehensive ;;
    *) echo "Invalid choice" ;;
esac
