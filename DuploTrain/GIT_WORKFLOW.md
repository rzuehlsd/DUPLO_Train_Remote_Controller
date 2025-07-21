# Git Workflow Guide - DUPLO Train Controller

## Branch Overview

### Current Branches
- **`main`**: Original basic implementation 
- **`feature/multi-core-implementation`**: Multi-task architecture with basic motor control
- **`feature/extended-sensor-support`**: Full sensor integration with bidirectional processing

## Testing Workflow

### Switch to Basic Multi-Task Version
```bash
git checkout feature/multi-core-implementation
pio run --target upload    # Upload basic version
pio device monitor         # Test basic motor control
```

**Features Available:**
- ✅ Multi-task architecture (BLE on Core 0)
- ✅ Basic motor control (setMotorSpeed, stopMotor)
- ✅ LED control (setLedColor)
- ✅ Connection callbacks
- ✅ Automatic connection recovery
- ❌ No sensor support

### Switch to Extended Sensor Version
```bash
git checkout feature/extended-sensor-support
pio run --target upload    # Upload extended version
pio device monitor         # Test full sensor integration
```

**Features Available:**
- ✅ All basic features PLUS:
- ✅ Color sensor with speed control callbacks
- ✅ Distance sensor with emergency stop
- ✅ Button sensor for manual control
- ✅ Bidirectional FreeRTOS queues
- ✅ Real-time sensor processing (~75ms latency)

### Compare Implementations Side-by-Side
```bash
# Compare basic vs extended files
git diff feature/multi-core-implementation..feature/extended-sensor-support -- src/

# Check file differences
git show feature/extended-sensor-support:include/DuploHub_Extended.h
git show feature/multi-core-implementation:include/DuploHub.h
```

## Development Workflow

### Creating Feature Branches
```bash
# From multi-core base
git checkout feature/multi-core-implementation
git checkout -b feature/new-sensor-type

# From extended base  
git checkout feature/extended-sensor-support
git checkout -b feature/advanced-control
```

### Testing Protocol

### Testing Protocol

#### Automated Test Script
```bash
./test_versions.sh

# Available test cases:
# 1. Basic Multi-Task Version - Core FreeRTOS implementation
# 2. Extended Sensor Version - Full bidirectional sensor processing  
# 3. Compare Versions - Side-by-side code analysis
# 4. Show Branch Status - Git repository overview
# 5. Initial Implementation - Original single-task version
# 6. Reconnect Integrated - Connection recovery testing
# 7. Comprehensive Testing - Build verification for all versions
```

#### Manual Testing Sequence
```bash
# Test original implementation
git checkout main
pio run --target upload
# Verify: Single-task, blocking BLE operations

# Test basic multi-task
git checkout feature/multi-core-implementation
pio run --target upload  
# Verify: Non-blocking BLE, basic motor control

# Test extended sensor support
git checkout feature/extended-sensor-support
pio run --target upload
# Verify: Full sensor integration, bidirectional queues

# Test connection recovery (use extended version)
git checkout feature/extended-sensor-support
pio run --target upload
# Test procedure:
# 1. Wait for hub connection
# 2. Power off hub (simulate disconnection)
# 3. Observe reconnection attempts in serial monitor
# 4. Power on hub (test reconnection)
# 5. Verify sensor callbacks resume
```

### Integration Strategy

#### Option 1: Merge Extended into Multi-Core
```bash
git checkout feature/multi-core-implementation
git merge feature/extended-sensor-support
# Resolve any conflicts
git commit -m "integrate: Merge extended sensor support"
```

#### Option 2: Create Unified Feature Branch
```bash
git checkout feature/multi-core-implementation
git checkout -b feature/unified-implementation
git merge feature/extended-sensor-support
# Test thoroughly
# If successful, this becomes the main integration branch
```

#### Option 3: Selective Feature Integration
```bash
# Cherry-pick specific commits from extended branch
git checkout feature/multi-core-implementation
git cherry-pick <specific-commit-hash>
```

## Recommended Testing Sequence

### 1. Hardware Validation
```bash
# Test basic functionality first
git checkout feature/multi-core-implementation
# Verify: motor control, LED control, connection stability

# Then test extended features
git checkout feature/extended-sensor-support  
# Verify: all sensors working, timing requirements met
```

### 2. Performance Comparison
```bash
# Monitor memory usage and timing
# Compare: basic vs extended versions
# Document: performance differences
```

### 3. Integration Testing
```bash
# Create integration branch
git checkout -b integration/final-testing
git merge feature/multi-core-implementation
git merge feature/extended-sensor-support
# Resolve conflicts, test thoroughly
```

## Merge to Main Strategy

### When Ready for Production
```bash
# Option 1: Merge multi-core first, then extended
git checkout main
git merge feature/multi-core-implementation
git tag v2.0.0
git merge feature/extended-sensor-support  
git tag v2.1.0

# Option 2: Direct merge of extended (includes all features)
git checkout main
git merge feature/extended-sensor-support
git tag v2.1.0
```

## Branch Maintenance

### Keep Branches Updated
```bash
# Update feature branches with latest main
git checkout feature/multi-core-implementation
git rebase main

git checkout feature/extended-sensor-support
git rebase main
```

### Clean Up After Integration
```bash
# After successful merge to main
git branch -d feature/multi-core-implementation
git branch -d feature/extended-sensor-support
```
