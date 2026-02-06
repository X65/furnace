# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Furnace is a multi-system chiptune tracker - music production software for creating music using vintage sound chips from retro computers and game consoles. It supports over 200+ sound chip configurations and can load/save various tracker formats including DefleMask (.dmf).

## Build System

Furnace uses CMake as its build system. The project requires C++14 and has extensive platform support (Windows, macOS, Linux).

### Building

```bash
# Initial setup
git clone --recursive https://github.com/tildearrow/furnace.git
cd furnace

# Standard build process
mkdir build
cd build
cmake ..
make                # or 'ninja' on Windows/MSYS2
```

### Platform-Specific Build Notes

- **Windows (MSYS2/MinGW64)**: Use `ninja` instead of `make`, ensure MINGW64 environment (NOT UCRT64)
- **Windows (Visual Studio)**: Use `msbuild ALL_BUILD.vcxproj` or open the generated solution
- **macOS**: Can use Xcode generator with `cmake -G Xcode ..`

### Important CMake Options

- `BUILD_GUI=ON/OFF` - Build GUI tracker vs headless player only
- `USE_RTMIDI=ON/OFF` - Include MIDI support
- `WARNINGS_ARE_ERRORS=ON/OFF` - Treat warnings as errors (recommended for development)
- `SYSTEM_*` options - Use system libraries instead of vendored ones

After initial CMake configuration, you only need to run the build command (make/ninja/msbuild) for subsequent builds.

## Testing

The project includes a delta-based audio regression testing system:

```bash
./test/furnace-test.sh
```

This script:
1. Renders all test songs in `test/songs/` to WAV files
2. Calculates audio deltas compared to previous test run
3. Verifies deltas are within acceptable thresholds

Requires GNU parallel and ffmpeg. The test compares rendered audio output to detect playback regressions.

## Architecture

### Core Components

**Engine (`src/engine/`)** - The heart of Furnace
- `engine.cpp/h` - Main engine class (DivEngine) coordinating all subsystems
- `playback.cpp` - Pattern playback and timing logic (131KB - complex!)
- `song.cpp/h` - Song data structure and management
- `dispatch.h` - Command interface for chip communication (DIV_CMD_*)
- `platform/` - Sound chip implementations (70+ files, one per chip/chip family)

**Platform Dispatch System**
Each sound chip has a "platform" implementation (DivPlatform*) that:
- Handles note on/off, volume, panning via dispatch commands
- Manages chip-specific registers and state
- Provides audio output through acquire() method
- Located in `src/engine/platform/`

**GUI (`src/gui/`)** - ImGui-based interface
- Large number of specialized files for different windows/features
- `gui.cpp` (not shown but exists) - Main GUI coordinator
- Font files embedded as C++ arrays (font_*.cpp)
- Pattern editor, instrument editor, mixer, etc. as separate components

**Audio Backend (`src/audio/`)** - Platform audio I/O
- Supports multiple backends: SDL, PortAudio, JACK, ASIO, Pipe
- `taAudio.h` - Abstract audio interface

**File Operations (`src/engine/fileOps/` and `src/engine/`)**
- Module loading/saving spread across multiple files
- Format compatibility is CRITICAL - see contribution guidelines
- `fileOpsIns.cpp` - Instrument file operations
- `vgmOps.cpp`, `wavOps.cpp` - Export operations

### Key Abstractions

1. **DivEngine** - Central coordinator managing song, instruments, playback, audio output
2. **DivDispatch** - Abstract interface for sound chip implementations
3. **DivInstrument** - Instrument definition with chip-specific parameters
4. **DivSample** - PCM sample data and metadata
5. **DivWavetable** - Wavetable data for wavetable synthesis
6. **DivSong** - Complete song data including patterns, orders, subsongs

### Data Flow

```
User Input (GUI/CLI) → DivEngine → Playback Engine → DivDispatch (per chip) → Audio Backend → Output
                                   ↓
                              Pattern Data
                              Instruments
                              Samples
```

## Coding Conventions (from CONTRIBUTING.md)

**Critical Rules:**
- **Two spaces for indentation** - NO TABS
- Modified 1TBS brace style
- No spaces in function calls: `foo(a,b)` not `foo( a, b )`
- C++ pointer style: `void* variable` not `void *variable`
- Always specify signedness for char types except for C strings
- Prefer `NULL` over `nullptr`
- Use `String` typedef for `std::string` (from ta-utils.h)

**Type Preferences:**
- `bool`, `signed char`, `unsigned char` (8-bit)
- `short`, `unsigned short` (16-bit)
- `int`, `unsigned int` (32-bit)
- `float` (32-bit), `double` (64-bit)
- `int64_t`, `uint64_t` for 64-bit (avoid when possible, target is 32-bit)
- `size_t` for sizes

**Compatibility Requirements:**
- **NEVER break file format compatibility** without extreme care
- **NEVER bump version number** in DIV_ENGINE_VERSION
- **DO NOT modify loadFur/saveFur** without deep understanding
- New fields must be added at END of blocks for forward compatibility
- Test with old songs after playback changes
- Do not use `#pragma once` (use include guards)
- Do not use `goto` unless absolutely necessary
- Default case must be LAST in switch statements

**Additional Guidelines:**
- Use safe C string operations only (snprintf, strncpy, strncat)
- Always use decimal with `f` suffix for floats: `1.0f` not `1`
- Only use `auto` when necessary

## Common Development Workflow

1. **Adding a New Sound Chip:**
   - Create platform implementation in `src/engine/platform/`
   - Inherit from DivDispatch, implement required methods
   - Add chip definition to `sysDef.cpp`
   - Add to CMakeLists.txt

2. **Modifying Playback:**
   - Main logic is in `src/engine/playback.cpp` (massive file)
   - Always test with old songs for regression
   - May require adding compatibility flags

3. **GUI Changes:**
   - Find relevant file in `src/engine/gui/`
   - ImGui is the UI framework
   - GUI coordinates with engine via DivEngine interface

4. **Adding Effects/Commands:**
   - Add to DivDispatchCmds enum in `dispatch.h`
   - Update cmdName[] array in `playback.cpp`
   - Implement in relevant platform files

## External Dependencies

Located in `extern/` directory (git submodules):
- SDL2 - Windowing and audio
- ImGui - GUI framework
- Various sound chip emulation cores (Nuked, MAME, ymfm, etc.)
- Audio codec libraries (FLAC, Vorbis, Opus, LAME)
- libsndfile - Audio file I/O

**Important:** Always clone with `--recursive` to fetch submodules.

## Current Version

Version 0.6.8 (dev244) as of engine.h. Project targets version 0.7 with features listed in TODO.md.
