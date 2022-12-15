#include "main.h"

lv_style_t myButtonStyleREL2; //relesed style
lv_style_t myButtonStylePR2; //pressed style

int nextFree = 0;

lv_obj_t* createButton(int x, int y, int w, int h, lv_action_t cb, lv_obj_t* parent, const char* text)
{
    // Button Styles
    lv_style_copy(&myButtonStyleREL2, &lv_style_plain);
    myButtonStyleREL2.body.main_color = LV_COLOR_MAKE(105, 159, 224);
    myButtonStyleREL2.body.grad_color = LV_COLOR_MAKE(105, 159, 224);
    myButtonStyleREL2.body.radius = 5;
    myButtonStyleREL2.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR2, &lv_style_plain);
    myButtonStylePR2.body.main_color = LV_COLOR_MAKE(105 - 20, 159 - 20, 224 - 20);
    myButtonStylePR2.body.grad_color = LV_COLOR_MAKE(105 - 20, 159 - 20, 224 - 20);
    myButtonStylePR2.body.radius = 5;
    myButtonStylePR2.text.color = LV_COLOR_MAKE(255, 255, 255);


    // Button creation
    lv_obj_t * btn1 = lv_btn_create(parent, NULL);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, x, y);
    lv_obj_set_size(btn1, w, h);
    lv_btn_set_style(btn1, LV_BTN_STYLE_REL, &myButtonStyleREL2);
    lv_btn_set_style(btn1, LV_BTN_STYLE_PR, &myButtonStylePR2);
    lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, cb);
    lv_obj_set_free_num(btn1, nextFree); 
    nextFree += 1;

    lv_obj_t * label;
    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, text);
    return btn1;

}