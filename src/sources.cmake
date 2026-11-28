set(ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(ARDENS ${ROOT}/third_party/ardens)
set(MINIZ ${ROOT}/third_party/miniz)
set(LODEPNG ${ROOT}/third_party/lodepng)
set(ARDUGO_SOURCES
    ${ROOT}/src/app.cpp
    ${ROOT}/src/catalog.cpp
    ${ROOT}/src/core.cpp
    ${ROOT}/src/files.cpp
    ${ROOT}/src/ui.cpp
    ${ARDENS}/src/absim_arduboy.cpp
    ${ARDENS}/src/absim_cpu_data.cpp
    ${ARDENS}/src/absim_decode.cpp
    ${ARDENS}/src/absim_disassemble.cpp
    ${ARDENS}/src/absim_execute.cpp
    ${ARDENS}/src/absim_jit_compiler.cpp
    ${ARDENS}/src/absim_load_file.cpp
    ${ARDENS}/src/absim_merge_instrs.cpp
    ${ARDENS}/src/absim_reset.cpp
    ${ARDENS}/src/absim_snapshot.cpp
    ${ARDENS}/src/boot/boot_flashcart.c
    ${ARDENS}/src/boot/boot_game_d1.c
    ${ARDENS}/src/boot/boot_game_d2.c
    ${ARDENS}/src/boot/boot_game_e2.c
    ${ARDENS}/src/boot/boot_menu_d1.c
    ${ARDENS}/src/boot/boot_menu_d2.c
    ${ARDENS}/src/boot/boot_menu_e2.c
    ${ARDENS}/yyjson/yyjson.c
    ${MINIZ}/miniz.c
    ${MINIZ}/miniz_tdef.c
    ${MINIZ}/miniz_tinfl.c
    ${MINIZ}/miniz_zip.c
    ${LODEPNG}/lodepng.cpp
)
set(ARDUGO_INCLUDES ${ROOT}/src ${ARDENS} ${ARDENS}/src ${ARDENS}/bitsery/include ${MINIZ} ${LODEPNG})
set(ARDUGO_DEFINES
    ARDUGO_EMBEDDED ARDENS_NO_DEBUGGER ARDENS_NO_SNAPSHOTS ARDENS_NO_ARDUBOY_FILE
    ARDENS_VERSION_MAJOR=0 ARDENS_VERSION_MINOR=25 ARDENS_VERSION_PATCH=12
    MINIZ_NO_ZLIB_COMPATIBLE_NAMES MINIZ_NO_ARCHIVE_WRITING_APIS
    LODEPNG_NO_COMPILE_ENCODER LODEPNG_NO_COMPILE_DISK LODEPNG_NO_COMPILE_CPP
)
