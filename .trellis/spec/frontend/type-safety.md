# Type Safety

> Not applicable to frontend; C/C++ firmware type safety lives in backend specs.

This repository uses C and C++17, not TypeScript. For current work, prefer
explicit fixed-width integer types for protocol fields, `constexpr` for frame
lengths, and compile-time checks where practical.

If a frontend is added later, replace this file with real TypeScript conventions
from that codebase.
