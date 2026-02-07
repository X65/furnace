# Furnace AGENTS Guide

## Purpose
- Furnace is a C++14/CMake multi-system chiptune tracker.
- This guide is for autonomous coding agents working in this repository.
- Primary priorities: preserve playback and file-format compatibility, follow project style, and verify changes before handoff.

## Quick Start (Linux default)
```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWARNINGS_ARE_ERRORS=ON
cmake --build build --parallel
./build/furnace -help
```
- Submodules are mandatory; `--recursive` is required for a valid build.
- An existing `build/` directory is normal; reuse it.
- `Ninja` is preferred when available.

## Platform Build Notes
- Windows (MSVC): configure with a Visual Studio generator, then build with `msbuild`.
- Windows (MSYS2/MinGW): use the **MINGW64** environment (not UCRT64), then build with `ninja`.
- macOS: `cmake -G Xcode` is supported.
- See `README.md` for the full platform/build matrix and troubleshooting.

## Validation Workflow
- Baseline verification for all code changes:
```bash
cmake --build build --parallel
./build/furnace -help
```
- Playback/audio regression verification for engine/platform changes:
```bash
./test/furnace-test.sh
```
- Regression script caveat: requires `ffmpeg`, GNU `parallel`, and a `test/songs/` corpus. The songs corpus may be absent because it is gitignored.
- Translation verification when touching i18n strings or `po/`:
```bash
./scripts/build-po.sh
```

## Repo Map (Where to Edit)
- `src/engine/`: core playback, song model, and engine state.
- `src/engine/platform/`: platform/chip wrappers and dispatch glue.
- `src/engine/platform/sound/`: low-level chip cores (C/C++).
- `src/gui/`: ImGui UI and editor windows.
- `src/audio/`: audio backends and I/O plumbing.
- `src/main.cpp`: CLI/entrypoint wiring.
- `scripts/`: release/build helper scripts, cross-toolchain files, translation helpers.
- `test/`: audio delta tooling and test scripts.
- `extern/`: vendored dependencies/submodules; avoid casual edits.

## Critical Guardrails
- Do not break module/instrument format compatibility.
- Treat `loadFur`/`saveFur` and instrument serialization as high-risk areas.
- New serialized fields must be appended at block ends.
- Do not bump `DIV_ENGINE_VERSION` in `src/engine/engine.h`.
- Test older songs after playback routine changes.
- Do not use `#pragma once` (use include guards).
- Keep `default` as the last case in `switch` blocks.
- Do not force-push after submitting a PR.

## Code Style Essentials
- Use 2-space indentation and never tabs.
- Use pointer style `Type* name`.
- Prefer `NULL` over `nullptr` (project convention).
- For non-string `char`, always specify signedness (`signed char` / `unsigned char`).
- Avoid unsafe C-string functions; use bounded/safe variants.
- Some legacy files in `src/engine/platform/sound/` and `extern/` may not fully match style; do not treat them as canonical formatting examples.

## SGU/POKEY Hotspot Guide (Fork-Specific)
- Active fork hotspots:
- `src/engine/platform/sgu.cpp`
- `src/engine/platform/sgu.h`
- `src/engine/platform/sound/sgu.c`
- `src/engine/platform/sound/sgu.h`
- `src/gui/regView.cpp` (SGU/ESFM/OPM comparison UI)
- Recent SGU work has focused on POKEY-like noise/filter/frequency behavior; treat these paths as coupled.
- Keep register/waveform mapping behavior consistent between `src/engine/platform/sgu.cpp` and `src/engine/platform/sound/sgu.c`.
- Validate periodic-noise/POKEY behavior by ear and with `./test/furnace-test.sh` when corpus/deps are available.
- Preserve existing register mapping semantics unless intentionally changing them, and document intentional changes in the PR.

## Change Scoping and Hygiene
- Keep patches focused; avoid unrelated refactors.
- Do not commit generated artifacts (`build/`, test outputs, binaries).
- If changing CI/build options, mirror expectations in `.github/workflows/build.yml` (Debug + warnings-as-errors + submodules).
- Prefer small, reviewable commits.
