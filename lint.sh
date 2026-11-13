#!/usr/bin/env bash

set -u

cd "$(dirname "$0")"

# ament_cppcheck は cppcheck 2.x を「遅いバージョン」として拒否するため、許可する
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

LINTERS=(
  "ament_copyright"
  "ament_cppcheck"
  "ament_cpplint"
  "ament_lint_cmake"
  "ament_xmllint"
)

failed=()
for linter in "${LINTERS[@]}"; do
  echo "=== ${linter} ==="
  if ! "${linter}"; then
    failed+=("${linter}")
  fi
done

echo "=== clang-format ==="
mapfile -t cpp_files < <(git ls-files '*.c' '*.h' '*.cpp' '*.hpp')
if ! clang-format --dry-run --Werror "${cpp_files[@]}"; then
  failed+=("clang-format")
fi

echo "=== ruff check ==="
if ! ruff check .; then
  failed+=("ruff check")
fi

echo "=== ruff format ==="
if ! ruff format --check .; then
  failed+=("ruff format")
fi

echo
if [ "${#failed[@]}" -ne 0 ]; then
  echo "Failed: ${failed[*]}"
  exit 1
fi
echo "All lint checks passed."
