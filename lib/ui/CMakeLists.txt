SET(SOURCES ui_ChargingScreen.c
    ui_MainScreen.c
    ui_TherapyScreen.c
    ui_FilterScreen.c
    ui_SettingScreen.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_battery_charging_png.c
    ui_img_2022142161.c
    ui_img_732690875.c
    ui_img_33364130.c
    ui_img_1713356312.c
    ui_img_548664401.c
    ui_img_1546845897.c
    ui_img_1759250347.c
    ui_img_719203222.c
    ui_img_battery_low_png.c
    ui_img_battery_discharge_png.c
    ui_img_battery_full_png.c
    ui_img_bluetooth_icon_png.c
    fonts/ui_font_Number_Medium.c
    fonts/ui_font_Number_Small.c
    fonts/ui_font_Number.c
    fonts/ui_font_Number_SMedium.c)

add_library(ui ${SOURCES})
