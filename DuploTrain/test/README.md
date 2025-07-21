# Isolated Testing Environment

This directory contains tools for testing different versions of the DUPLO Train Controller without affecting your development branches.

## How It Works

The `test_versions_isolated.sh` script uses **Git Worktrees** to create a completely separate working directory for testing. This means:

- ✅ Your main development branches remain untouched
- ✅ No risk of accidentally committing test changes
- ✅ Can switch between versions freely in the test environment
- ✅ Build artifacts are isolated from main development
- ✅ Complete separation of development and testing workflows

## Usage

1. **Navigate to the test directory:**
   ```bash
   cd test/
   ```

2. **Run the isolated test script:**
   ```bash
   ./test_versions_isolated.sh
   ```

3. **First time setup:**
   - The script will automatically create a testing environment in `test/testing_env/`
   - This is a separate Git worktree linked to your main repository
   - Choose option 8 to manually setup/reset the testing environment

## Directory Structure

```
DuploTrain/
├── src/                    # Your main development files
├── test/
│   ├── test_versions_isolated.sh    # The isolated testing script
│   ├── testing_env/                 # Git worktree for testing (auto-created)
│   │   ├── src/                     # Isolated copy of source files
│   │   ├── platformio.ini           # PlatformIO config for testing
│   │   └── .pio/                    # Isolated build artifacts
│   └── README.md                    # This file
└── ...                     # Your other development files
```

## Testing Options

1. **Test Basic Initial Version** - Original implementation (c2142b5)
2. **Test Extended Sensor Version** - Latest sensor features
3. **Compare Versions** - Side-by-side analysis
4. **Show Repository Status** - Status of both main repo and test environment
5. **Test Multi-Task Implementation** - FreeRTOS architecture
6. **Test Reconnect Integrated** - Connection recovery (248efdb)
7. **Comprehensive Testing** - Build verification for all versions
8. **Setup/Reset Testing Environment** - Create/recreate the test worktree
9. **Cleanup Testing Environment** - Remove the test worktree

## Benefits of This Approach

### 🛡️ Safety
- No risk of messing up your development branches
- Isolated build environment prevents conflicts
- Easy to reset/cleanup if something goes wrong

### 🔄 Flexibility  
- Switch between any commit/branch for testing
- Multiple versions can be tested quickly
- Compare implementations side-by-side

### 🧹 Clean Development
- Keep your main workspace focused on development
- Test artifacts don't clutter your main .pio directory
- Clear separation of concerns

## Git Worktree Benefits

Git worktrees allow you to have multiple working directories from the same repository:
- Each worktree can be on a different branch/commit
- Share the same Git history and objects
- Independent working directories and staging areas
- Perfect for testing without affecting main development

## Cleanup

When you're done testing, you can:
- Use option 9 to cleanup the testing environment
- Or manually remove: `rm -rf testing_env/`
- The main repository remains completely unaffected

## Troubleshooting

**"This script should be run from the test/ directory"**
- Navigate to the test/ directory: `cd test/`

**"Failed to create testing environment"**
- Make sure you're in a Git repository
- Check that all branches exist: `git branch -a`
- Try option 8 to reset the testing environment

**Build errors in testing environment**
- The testing environment uses the same platformio.ini
- Make sure PlatformIO is properly installed
- Check that all dependencies are available
