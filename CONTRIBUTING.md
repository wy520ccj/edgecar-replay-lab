# Contributing

1. Build with `cmake --preset debug` and run `ctest --preset debug --output-on-failure`.
2. Run `scripts/run_demo.ps1` (or `scripts/run_demo.sh`) before opening a pull request.
3. Keep safety decisions deterministic and add a regression scenario for every bug.
4. Do not add vehicle credentials, private recordings, vendor SDKs, course PDFs, or model weights without a written redistribution record.
5. Format C++ with the repository `.clang-format` configuration.
