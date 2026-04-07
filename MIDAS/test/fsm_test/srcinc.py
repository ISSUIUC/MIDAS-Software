# Michael Karpov (2026)
# Includes fsm_test's build files in the platformio build
Import("env")

env.BuildSources(
    "$BUILD_DIR/test_sources",
    "$PROJECT_DIR/test/fsm_test/src",
    src_filter=["+<*.cpp>"]
)