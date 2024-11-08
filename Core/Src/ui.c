//
// Created by wine on 24-10-24.
//

#include "ui.h"

//在项目根目录的CMakeLists.txt中添加以下内容才会正常显示文字
//set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")



void updateUI(u8g2_t *u8g2, float voltage, float current, float power) {

    u8g2_ClearBuffer(u8g2);

    u8g2_DrawFrame(u8g2, 0, 0, u8g2_GetDisplayWidth(u8g2), u8g2_GetDisplayHeight(u8g2));

    for (int i = 0; i < 2; i++) { // Adjust the loop count for desired thickness
        u8g2_DrawLine(u8g2, 81+i, 0, 81+i, 32);
    }
    char buffer[16];

    u8g2_SetFont(u8g2, u8g2_font_timR18_tr);
    snprintf(buffer, sizeof(buffer), "W");
    u8g2_DrawStr(u8g2, 55, 24, buffer);
    snprintf(buffer, sizeof(buffer), "%.1f", power);
    u8g2_DrawStr(u8g2, 3, 24, buffer);

    u8g2_SetFont(u8g2, u8g2_font_ncenB10_tf);
    snprintf(buffer, sizeof(buffer), "V");
    u8g2_DrawStr(u8g2, 114, 14, buffer);
    snprintf(buffer, sizeof(buffer), "%.1f", voltage);
    u8g2_DrawStr(u8g2, 85, 14, buffer);

    snprintf(buffer, sizeof(buffer), "A");
    u8g2_DrawStr(u8g2, 114, 29, buffer);
    snprintf(buffer, sizeof(buffer), "%.1f", current);
    u8g2_DrawStr(u8g2, 85, 29, buffer);

    u8g2_SendBuffer(u8g2);
}
