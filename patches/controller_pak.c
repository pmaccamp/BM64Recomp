#include "patches.h"
#include "patch_helpers.h"

enum {
    GAME_PAK_NOT_CONNECTED = 0,
    GAME_PAK_NO_COSTUME_FILE = 7,
};

// These are game-level queries, separate from libultra's controller status.
// No Controller Pak storage is emulated, including when a physical adapter is used.
RECOMP_PATCH s32 func_8027BEF0(void) {
    return 0;
}

RECOMP_PATCH s32 func_8027C8A4(s32 controller) {
    return GAME_PAK_NOT_CONNECTED;
}

RECOMP_PATCH s32 func_8027C88C(s32 controller) {
    return GAME_PAK_NOT_CONNECTED;
}

// Battle character select accepts 7 (no costume file) or 9 (loaded costumes).
// Its original no-Pak result (0) opens an error dialog before character select.
// Adapt only this screen's query so it uses built-in characters without claiming
// that a Pak exists or that any costume data was loaded or saved.
RECOMP_PATCH s32 func_8027C88C__0x0303__secure_call(s32 controller) {
    return GAME_PAK_NO_COSTUME_FILE;
}
