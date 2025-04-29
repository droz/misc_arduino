
#include <lvgl.h>
#include <TFT_eSPI.h>

// Screen resolution and rotation
#define TFT_HOR_RES   320
#define TFT_VER_RES   240
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

// LVGL draws into these buffers (This is double buffered)
// Ref: https://docs.lvgl.io/8.0/porting/display.html
// "A larger buffer results in better performance but above 1/10 screen
// sized buffer(s) there is no significant performance improvement.
// Therefore it's recommended to choose the size of the draw buffer(s)
// to be at least 1/10 screen sized.
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10)
static lv_color_t buf_1[DRAW_BUF_SIZE];
static lv_color_t buf_2[DRAW_BUF_SIZE];
static lv_disp_draw_buf_t disp_buf;

// Provides access to the timer for lvgl
static uint32_t lv_tick(void)
{
    return millis();
}

// Provides access to the serial port for lvgl
static void lv_print( lv_log_level_t level, const char * buf )
{
    Serial.print("LVGL ")
    switch(level) {
        case LV_LOG_LEVEL_TRACE:
            Serial.print("TRACE: ");
            break;
        case LV_LOG_LEVEL_INFO:
            Serial.print("INFO: ");
            break;
        case LV_LOG_LEVEL_WARN:
            Serial.print("WARN: ");
            break;
        case LV_LOG_LEVEL_ERROR:
            Serial.print("ERROR: ");
            break;
        case LV_LOG_LEVEL_USER:
            Serial.print("USER: ");
            break;
        default:
            Serial.print("UNKNOWN: ");
        break;
    }
    Serial.println(buf);
    Serial.flush();
}

// Provides access to the touchpad for lvgl
// This is polled by lvgl on a regular basis
void lv_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    /*For example  ("my_..." functions needs to be implemented by you)
    int32_t x, y;
    bool touched = my_get_touch( &x, &y );

    if(!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        data->state = LV_INDEV_STATE_PRESSED;

        data->point.x = x;
        data->point.y = y;
    }
     */
}

// Initialization
void setup() {

    // Serial port initialization
    Serial.begin( 115200 );
    String LVGL_Ver = "LVGL ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println( LVGL_Arduino );

    // Init LVGL core
    lv_init();

    // Set a tick source so that LVGL will know how much time elapsed.
    lv_tick_set_cb(lv_tick);

    // Set a print function for logging
    lv_log_register_print_cb(lv_print);

    lv_display_t * disp;
    disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    lv_display_set_rotation(disp, TFT_ROTATION);

    // There is no native LVGL support for the FT6336U chip, so we use
    // a dummy driver, and we will implement touch callback ourselves.
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lv_touchpad_read);

    /* Create a simple label
     * ---------------------
     lv_obj_t *label = lv_label_create( lv_screen_active() );
     lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
     lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );

     * Try an example. See all the examples
     *  - Online: https://docs.lvgl.io/master/examples.html
     *  - Source codes: https://github.com/lvgl/lvgl/tree/master/examples
     * ----------------------------------------------------------------

     lv_example_btn_1();

     * Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMO_WIDGETS
     * -------------------------------------------------------------------------------------------

     lv_demo_widgets();
     */

    lv_obj_t *label = lv_label_create( lv_screen_active() );
    lv_label_set_text( label, "Hello Teensy, I'm LVGL!" );
    lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );

    Serial.println( "Setup done" );
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5); /* let this time pass */
}