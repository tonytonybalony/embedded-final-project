/*//////////////////////////////////////////////////////////////////////
INCLUDE FILES AND LIBRARIES
//////////////////////////////////////////////////////////////////////*/


#include "lv_examples.h"
#include "../../src/assets/icons/paused.c"
#include "../../src/assets/icons/next.c"
#include "../../src/assets/icons/previous.c"
#include "../../src/assets/icons/play.c"
#include "../../src/assets/icons/gnx_cover.c"
#include <string.h>   // strcmp

/*//////////////////////////////////////////////////////////////////////
DECLARATIONS
//////////////////////////////////////////////////////////////////////*/


LV_IMG_DECLARE(paused);
LV_IMG_DECLARE(next);
LV_IMG_DECLARE(previous);
LV_IMG_DECLARE(play);
LV_IMG_DECLARE(gnx_cover);

/*//////////////////////////////////////////////////////////////////////
REFERENCE FUNCTION
//////////////////////////////////////////////////////////////////////*/


/*  RADIAL GRADIENT BACKGROUND EXAMPLE FUNCTION

void lv_example_style_17(void)
{
    static const lv_color_t grad_colors[2] = {
        LV_COLOR_MAKE(0x9B, 0x18, 0x42),
        LV_COLOR_MAKE(0x00, 0x00, 0x00),
    };

    int32_t width = lv_display_get_horizontal_resolution(NULL);
    int32_t height = lv_display_get_vertical_resolution(NULL);

    static lv_style_t style;
    lv_style_init(&style);

    //First define a color gradient. In this example we use a purple to black color map.
    static lv_grad_dsc_t grad;

    lv_grad_init_stops(&grad, grad_colors, NULL, NULL, sizeof(grad_colors) / sizeof(lv_color_t));

    //Make a radial gradient with the center in the middle of the object, extending to the farthest corner
    lv_grad_radial_init(&grad, LV_GRAD_CENTER, LV_GRAD_CENTER, LV_GRAD_RIGHT, LV_GRAD_BOTTOM, LV_GRAD_EXTEND_PAD);

    //Set gradient as background
    lv_style_set_bg_grad(&style, &grad);

    //Create an object with the new style
    lv_obj_t * obj = lv_obj_create(lv_screen_active());
    lv_obj_add_style(obj, &style, 0);
    lv_obj_set_size(obj, width, height);
    lv_obj_center(obj);
}

*/



/*  SLIDER WITH LABEL EXAMPLE FUNCTION

static void slider_event_cb(lv_event_t * e);
static lv_obj_t * slider_label;
// A default slider with a label displaying the current value
void lv_example_slider_1(void)
{
    //Create a slider in the center of the display
    lv_obj_t * slider = lv_slider_create(lv_screen_active());
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_set_style_anim_duration(slider, 2000, 0);
    //Create a label below the slider
    slider_label = lv_label_create(lv_screen_active());
    lv_label_set_text(slider_label, "0%");

    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target_obj(e);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(slider_label, buf);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

*/



/*  CLICK AND TOGGLE BUTTON EXAMPLE FUNCTION

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked");
    }
    else if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
    }
}

void lv_example_button_1(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);

}

*/

/*  STYLE WITH OPACITY AND TRANSFORMATIONS EXAMPLE FUNCTION

void lv_example_style_15(void)
{
    lv_obj_t * btn;
    lv_obj_t * label;

    // Normal button
    btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 100, 40);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, -70);

    label = lv_label_create(btn);
    lv_label_set_text(label, "Normal");
    lv_obj_center(label);

    // Set opacity
    // The button and the label is rendered to a layer first and that layer is blended
    btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 100, 40);
    lv_obj_set_style_opa(btn, LV_OPA_50, 0);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);

    label = lv_label_create(btn);
    lv_label_set_text(label, "Opa:50%");
    lv_obj_center(label);

    // Set transformations
    // The button and the label is rendered to a layer first and that layer is transformed
    btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 100, 40);
    lv_obj_set_style_transform_rotation(btn, 150, 0);        // 15 deg
    lv_obj_set_style_transform_scale(btn, 256 + 64, 0);   // 1.25x
    lv_obj_set_style_transform_pivot_x(btn, 50, 0);
    lv_obj_set_style_transform_pivot_y(btn, 20, 0);
    lv_obj_set_style_opa(btn, LV_OPA_50, 0);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 70);

    label = lv_label_create(btn);
    lv_label_set_text(label, "Transf.");
    lv_obj_center(label);
}

*/


/*//////////////////////////////////////////////////////////////////////
RADIAL GRADIENT BACKGROUND FUNCTION
//////////////////////////////////////////////////////////////////////*/


void radial_gradient_background(void)
{
    static const lv_color_t grad_colors[2] = {
        LV_COLOR_MAKE(0xEE, 0xEE, 0xEE),
        LV_COLOR_MAKE(0x80, 0x80, 0x80),
    };

    static lv_style_t style;
    lv_style_init(&style);

    //First define a color gradient.
    static lv_grad_dsc_t grad;

    lv_grad_init_stops(&grad, grad_colors, NULL, NULL, sizeof(grad_colors) / sizeof(lv_color_t));

    //Make a radial gradient with the center in the middle of the object, extending to the farthest corner
    lv_grad_radial_init(&grad, LV_GRAD_CENTER, LV_GRAD_CENTER, LV_GRAD_RIGHT, LV_GRAD_BOTTOM, LV_GRAD_EXTEND_PAD);

    //Set gradient as background
    lv_style_set_bg_grad(&style, &grad);

    lv_obj_t * scr = lv_screen_active();
    lv_obj_add_style(scr, &style, 0);
}

/*//////////////////////////////////////////////////////////////////////
PROGRESS BAR WITH LABELS AND BUTTONS FUNCTION
//////////////////////////////////////////////////////////////////////*/


static void slider_event_cb(lv_event_t * e);
static lv_obj_t * slider;            // progress slider
static lv_obj_t * slider_label;      // time elapsed label
static lv_obj_t * slider_label2;     // total time label
static lv_obj_t * g_slider;          // store slider for button control
static lv_obj_t * song_ticker;       // container
static lv_obj_t * song_label;        // text
static lv_obj_t * artist_label;      // smaller grey text below title
static lv_obj_t * btn_prev;
static lv_obj_t * btn_pause;
static lv_obj_t * btn_next;
static lv_obj_t * img_pause;
static lv_obj_t * album_img;
static bool is_paused = false;       // pause state

// Timer Declarations
static const int track_len_sec = 225;          // total length (3:45)
static int elapsed_sec = 0;                    // elapsed seconds
static lv_timer_t * progress_timer;            // timer advancing progress




// Layout update
static void layout_update(void)
{
    LV_LOG_INFO("Layout update called");
    if(slider)                      lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -120);
    if(song_ticker && slider)       lv_obj_align_to(song_ticker, slider, LV_ALIGN_OUT_TOP_MID, 0, -12);
    if(slider_label && slider)      lv_obj_align_to(slider_label,  slider, LV_ALIGN_OUT_BOTTOM_LEFT, -10, 10);
    if(slider_label2 && slider)     lv_obj_align_to(slider_label2, slider, LV_ALIGN_OUT_BOTTOM_RIGHT, 10, 10);
    if(btn_prev && slider)          lv_obj_align_to(btn_prev,  slider, LV_ALIGN_OUT_BOTTOM_LEFT, 10, 40);
    if(btn_pause && slider)         lv_obj_align_to(btn_pause, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 40);
    if(btn_next && slider)          lv_obj_align_to(btn_next,  slider, LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 40);
    if(album_img && slider)         lv_obj_align_to(album_img, slider, LV_ALIGN_OUT_TOP_MID, 0, -120);
    if(artist_label && song_ticker) lv_obj_align_to(artist_label, song_ticker, LV_ALIGN_OUT_BOTTOM_LEFT, 0, -10);
}




// Hide or show album based on window height
static void update_album_visibility(void)
{
    if(!album_img) return;
    int32_t h = lv_display_get_vertical_resolution(NULL);
    if(h < 450) {
        lv_obj_add_flag(album_img, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(album_img, LV_OBJ_FLAG_HIDDEN);
    }
}




// Get the resize event to update layout
static void screen_resize_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    update_album_visibility();
    layout_update();
}




// Start the song title ticker
static void start_song_ticker(void)
{
    // Ensure sizes are computed
    lv_obj_update_layout(song_ticker);

    int w_text = lv_obj_get_width(song_label);
    int w_view = lv_obj_get_width(song_ticker);

    if(w_text <= w_view) {
        lv_obj_set_x(song_label, 0);
        return;
    }

    lv_obj_set_x(song_label, 0);
    lv_anim_del(song_label, (lv_anim_exec_xcb_t)lv_obj_set_x);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, song_label);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_values(&a, 0, -(w_text - w_view + 8));  // full travel with padding
    lv_anim_set_time(&a, 10000);                        // slower = larger
    lv_anim_set_delay(&a, 3000);                        // initial pause
    lv_anim_set_repeat_delay(&a, 5000);                 // pause between loops
    lv_anim_set_path_cb(&a, lv_anim_path_linear);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}




// Restart the song title ticker after pressing prev or next
static void restart_song_title(const char * title)
{
    lv_anim_del(song_label, (lv_anim_exec_xcb_t)lv_obj_set_x);
    if(title) lv_label_set_text(song_label, title);
    lv_obj_set_x(song_label, 0);
    start_song_ticker();
}




// Update time labels based on seconds elapsed
static void update_time_labels(int seconds)
{
    if(slider_label) {
        int mins = seconds / 60;
        int secs = seconds % 60;
        char buf[16];
        lv_snprintf(buf, sizeof(buf), "%d:%02d", mins, secs);
        lv_label_set_text(slider_label, buf);
    }
}




// Sync Slider with elapsed time every second, also restart timer when hitting the end.
static void progress_timer_cb(lv_timer_t * t)
{
    LV_UNUSED(t);
    if(is_paused) return;

    elapsed_sec++;
    if(elapsed_sec > track_len_sec) elapsed_sec = 0;
    int32_t vmax = lv_slider_get_max_value(g_slider);
    int32_t v = (int32_t)((int64_t)elapsed_sec * vmax / track_len_sec);
    lv_slider_set_value(g_slider, v, LV_ANIM_OFF);
    update_time_labels(elapsed_sec);
}




// Function of prev, pause, and next buttons
static void button_event_cb(lv_event_t * e)
{
    const char * tag = (const char *)lv_event_get_user_data(e);
    if(!tag) return;
    if(strcmp(tag, "prev") == 0) {
        elapsed_sec = 0;
        is_paused = false;
        if(img_pause) lv_img_set_src(img_pause, is_paused ? &play : &paused);
        lv_slider_set_value(g_slider, 0, LV_ANIM_OFF);
        update_time_labels(0);
        restart_song_title(NULL);
        LV_LOG_USER("Prev: slider + title reset");
    }
    else if(strcmp(tag, "pause") == 0) {
        is_paused = !is_paused;
        LV_LOG_USER("Pause toggled. Now %s", is_paused ? "PAUSED" : "PLAYING");
        lv_obj_set_style_bg_opa(g_slider, is_paused ? LV_OPA_80 : LV_OPA_COVER, LV_PART_INDICATOR);
        if(img_pause) lv_img_set_src(img_pause, is_paused ? &play : &paused);
    }
    else if(strcmp(tag, "next") == 0) {
        elapsed_sec = 0;
        is_paused = false;
        if(img_pause) lv_img_set_src(img_pause, is_paused ? &play : &paused);
        lv_slider_set_value(g_slider, 0, LV_ANIM_OFF);
        update_time_labels(0);restart_song_title(NULL);
        LV_LOG_USER("Next: slider + title reset");
    }
}




// Function to handle manual slider changes
static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target_obj(e);
    int32_t v    = lv_slider_get_value(slider);
    int32_t vmax = lv_slider_get_max_value(slider);
    int32_t seconds = (int32_t)((int64_t)v * track_len_sec / vmax);
    elapsed_sec = seconds;
    update_time_labels(seconds);
}




// Main function to create the progress bar with labels and buttons
void progress_bar(void)
{
    // Create a slider in the lower center of the display
    slider = lv_slider_create(lv_screen_active());
    g_slider = slider;
    lv_obj_set_width(slider, 260);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_slider_set_range(slider, 0, 1000);
    lv_obj_set_style_anim_duration(slider, 2000, 0);

    if(!progress_timer)
        progress_timer = lv_timer_create(progress_timer_cb, 1000, NULL);




    // TICKER CONTAINER
    song_ticker = lv_obj_create(lv_screen_active());
    lv_obj_set_size(song_ticker, lv_obj_get_width(slider), LV_SIZE_CONTENT);
    lv_obj_clear_flag(song_ticker, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(song_ticker, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(song_ticker, LV_OPA_TRANSP, 0);

    lv_font_t * font = lv_freetype_font_create("./lvgl/examples/libs/freetype/NotoSerifCJK-Regular.ttc",
                                               LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                               12,
                                               LV_FREETYPE_FONT_STYLE_NORMAL);
    if(!font) {
        LV_LOG_ERROR("freetype font create failed.");
        return;
    }

    lv_font_t * fontlarge = lv_freetype_font_create("./lvgl/examples/libs/freetype/NotoSerifCJK-Regular.ttc",
                                                    LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                                    20,
                                                    LV_FREETYPE_FONT_STYLE_NORMAL);
    if(!fontlarge) {
        LV_LOG_ERROR("freetype fontlarge create failed.");
        return;
    }


    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_font(&style, font);

    static lv_style_t stylelarge;
    lv_style_init(&stylelarge);
    lv_style_set_text_font(&stylelarge, fontlarge);

    // Song title label
    song_label = lv_label_create(song_ticker);
    lv_obj_set_width(song_label, LV_SIZE_CONTENT);
    lv_label_set_long_mode(song_label, LV_LABEL_LONG_CLIP);
    lv_obj_add_style(song_label, &stylelarge, 0);
    lv_label_set_text(song_label, "dodger blue - (feat. Wallie the Sensei, Siete7x & Roddy Ricch)");
    lv_obj_set_style_pad_left(song_ticker, 0, 0);
    lv_obj_set_style_pad_right(song_ticker, 0, 0);
    lv_obj_set_style_pad_left(song_label, 0, 0);
    lv_obj_set_style_pad_right(song_label, 0, 0);
    lv_obj_align(song_label, LV_ALIGN_LEFT_MID, 0, 0);

    start_song_ticker();

    // Artist label below the song title (smaller, grey)
    artist_label = lv_label_create(lv_screen_active());
    lv_obj_add_style(artist_label, &style, 0);
    lv_label_set_text(artist_label, "Kendrick Lamar");
    lv_obj_set_style_text_color(artist_label, lv_color_hex(0x555555), 0);

    album_img = lv_img_create(lv_screen_active());
    lv_img_set_src(album_img, &gnx_cover);

    // Slider styles
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x999999), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xD9D9D9), LV_PART_INDICATOR);
    lv_obj_set_style_opa(slider, LV_OPA_TRANSP, LV_PART_KNOB);

    // Labels
    slider_label = lv_label_create(lv_screen_active());
    lv_label_set_text(slider_label, "0:00");
    slider_label2 = lv_label_create(lv_screen_active());
    lv_label_set_text(slider_label2, "3:45");

    // PREVIOUS BUTTON
    btn_prev = lv_btn_create(lv_screen_active());
    lv_obj_set_size(btn_prev, 50, 50);
    lv_obj_add_event_cb(btn_prev, button_event_cb, LV_EVENT_CLICKED, (void*)"prev");
    lv_obj_set_style_bg_color(btn_prev, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(btn_prev, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(btn_prev, LV_RADIUS_CIRCLE, 0);
    lv_obj_t * img_prev = lv_img_create(btn_prev);
    lv_img_set_src(img_prev, &previous);
    lv_obj_center(img_prev);

    // PAUSE BUTTON
    btn_pause = lv_btn_create(lv_screen_active());
    lv_obj_set_size(btn_pause, 50, 50);
    lv_obj_add_event_cb(btn_pause, button_event_cb, LV_EVENT_CLICKED, (void*)"pause");
    lv_obj_set_style_bg_color(btn_pause, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(btn_pause, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(btn_pause, LV_RADIUS_CIRCLE, 0);
    img_pause = lv_img_create(btn_pause);
    lv_img_set_src(img_pause, &paused);
    lv_obj_center(img_pause);

    // NEXT BUTTON
    btn_next = lv_btn_create(lv_screen_active());
    lv_obj_set_size(btn_next, 50, 50);
    lv_obj_add_event_cb(btn_next, button_event_cb, LV_EVENT_CLICKED, (void*)"next");
    lv_obj_set_style_bg_color(btn_next, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(btn_next, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(btn_next, LV_RADIUS_CIRCLE, 0);
    lv_obj_t * img_next = lv_img_create(btn_next);
    lv_img_set_src(img_next, &next);
    lv_obj_center(img_next);




    // Add resize event callback
    static bool resize_evt_added = false;
    if(!resize_evt_added) {
        resize_evt_added = true;
        lv_obj_add_event_cb(lv_screen_active(), screen_resize_cb, LV_EVENT_SIZE_CHANGED, NULL);
    }



    // After creating all related objects call:
    layout_update();

    // Ensure correct initial visibility based on current height
    update_album_visibility();
}
