# Project Guidelines

## Code Style

- Follow the project style in [CONTRIBUTING.md](CONTRIBUTING.md): 2-space indent, modified 1TBS, no tabs.
- No spaces in function calls: `foo(a,b)`.
- Use `void* var` pointer style; prefer `NULL` over `nullptr`.
- Specify signedness for `char` (except C strings). Avoid `_t` types unless 64-bit or `size_t`.
- Do not use `#pragma once`; always use include guards.

## Architecture

- Core engine lives in [src/engine/](src/engine/) with playback in [src/engine/playback.cpp](src/engine/playback.cpp) and dispatch commands in [src/engine/dispatch.h](src/engine/dispatch.h).
- Chip backends are in [src/engine/platform/](src/engine/platform/); each chip implements a DivDispatch platform.
- GUI is ImGui-based under [src/gui/](src/gui/).
- Audio backends are in [src/audio/](src/audio/).

## Build and Test

- Standard build flow: `mkdir build && cd build && cmake .. && make` (or `ninja`/`xcodebuild`/`msbuild`); see [README.md](README.md).
- CMake options are listed in [README.md](README.md#L350) and defined in [CMakeLists.txt](CMakeLists.txt#L54).
- Audio regression test: run [test/furnace-test.sh](test/furnace-test.sh) (requires GNU parallel + ffmpeg).
- Translation update/build: [scripts/update-po.sh](scripts/update-po.sh), [scripts/build-po.sh](scripts/build-po.sh), and [po/README.md](po/README.md).

## Project Conventions

- Format compatibility is critical: do not modify `loadFur`/`saveFur` or bump engine version; new fields go at block ends. See [CONTRIBUTING.md](CONTRIBUTING.md) and [CLAUDE.md](CLAUDE.md).
- Avoid breaking playback; test older songs after playback changes.
- Default case must be last in `switch` blocks.

## Integration Points

- External deps are vendored as submodules in [extern/](extern/); clone with `--recursive`.
- File format references: [papers/format.md](papers/format.md), export stream details in [papers/export-tech.md](papers/export-tech.md), clipboard format in [papers/clipboard-format.md](papers/clipboard-format.md).
